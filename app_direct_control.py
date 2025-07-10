#!/usr/bin/env python3
"""
Modified Flask app for WaveShare Robotaxi with Direct Motor Control
Uses GPIO pins directly instead of ESP32 communication
"""

# Import the direct motor controller instead of base_ctrl
from direct_motor_control import DirectBaseController
import threading
import yaml
import os

# Use direct motor control (no ESP32 required)
base = DirectBaseController()

# Start the breath light effect (mock)
threading.Thread(target=lambda: base.breath_light(15), daemon=True).start()

# Load configuration
curpath = os.path.realpath(__file__)
thisPath = os.path.dirname(curpath)
with open(thisPath + '/ugv_rpi/config.yaml', 'r') as yaml_file:
    f = yaml.safe_load(yaml_file)

# Display startup messages
base.base_oled(0, "Robotaxi Direct Control")
base.base_oled(1, f"sbc_version: {f['base_config']['sbc_version']}")
base.base_oled(2, f"{f['base_config']['main_type']}{f['base_config']['module_type']}")
base.base_oled(3, "Starting...")

# Import necessary modules
from flask import Flask, render_template, Response, request, jsonify, redirect, url_for, send_from_directory, send_file
from flask_socketio import SocketIO, emit
from werkzeug.utils import secure_filename
from aiortc import RTCPeerConnection, RTCSessionDescription
import json
import uuid
import asyncio
import time
import logging
import cv_ctrl
import audio_ctrl
import os_info

# Get system info
UPLOAD_FOLDER = thisPath + '/ugv_rpi/sounds/others'
si = os_info.SystemInfo()

# Create a Flask app instance
app = Flask(__name__)
socketio = SocketIO(app)

# Set to keep track of RTCPeerConnection instances
active_pcs = {}
MAX_CONNECTIONS = 1
pcs = set()

# Camera functions (using existing cv_ctrl)
cvf = cv_ctrl.OpencvFuncs(thisPath, base)

# Command actions (same as original)
cmd_actions = {
    f['code']['zoom_x1']: lambda: cvf.scale_ctrl(1),
    f['code']['zoom_x2']: lambda: cvf.scale_ctrl(2),
    f['code']['zoom_x4']: lambda: cvf.scale_ctrl(4),

    f['code']['pic_cap']: cvf.picture_capture,
    f['code']['vid_sta']: lambda: cvf.video_record(True),
    f['code']['vid_end']: lambda: cvf.video_record(False),

    f['code']['cv_none']: lambda: cvf.set_cv_mode(f['code']['cv_none']),
    f['code']['cv_moti']: lambda: cvf.set_cv_mode(f['code']['cv_moti']),
    f['code']['cv_face']: lambda: cvf.set_cv_mode(f['code']['cv_face']),
    f['code']['cv_objs']: lambda: cvf.set_cv_mode(f['code']['cv_objs']),
    f['code']['cv_clor']: lambda: cvf.set_cv_mode(f['code']['cv_clor']),
    f['code']['mp_hand']: lambda: cvf.set_cv_mode(f['code']['mp_hand']),
    f['code']['cv_auto']: lambda: cvf.set_cv_mode(f['code']['cv_auto']),
    f['code']['mp_face']: lambda: cvf.set_cv_mode(f['code']['mp_face']),
    f['code']['mp_pose']: lambda: cvf.set_cv_mode(f['code']['mp_pose']),

    f['code']['re_none']: lambda: cvf.set_detection_reaction(f['code']['re_none']),
    f['code']['re_capt']: lambda: cvf.set_detection_reaction(f['code']['re_capt']),
    f['code']['re_reco']: lambda: cvf.set_detection_reaction(f['code']['re_reco']),

    f['code']['mc_lock']: lambda: cvf.set_movtion_lock(True),
    f['code']['mc_unlo']: lambda: cvf.set_movtion_lock(False),

    f['code']['led_off']: lambda: cvf.head_light_ctrl(0),
    f['code']['led_aut']: lambda: cvf.head_light_ctrl(1),
    f['code']['led_ton']: lambda: cvf.head_light_ctrl(2),

    f['code']['release']: lambda: base.bus_servo_torque_lock(255, 0),
    f['code']['s_panid']: lambda: base.bus_servo_id_set(255, 2),
    f['code']['s_tilid']: lambda: base.bus_servo_id_set(255, 1),
    f['code']['set_mid']: lambda: base.bus_servo_mid_set(255),

    f['code']['base_of']: lambda: base.lights_ctrl(0, base.head_light_status),
    f['code']['base_on']: lambda: base.lights_ctrl(255, base.head_light_status),
    f['code']['head_ct']: lambda: cvf.head_light_ctrl(3),
    f['code']['base_ct']: base.base_lights_ctrl
}

cmd_feedback_actions = [f['code']['cv_none'], f['code']['cv_moti'],
                        f['code']['cv_face'], f['code']['cv_objs'],
                        f['code']['cv_clor'], f['code']['mp_hand'],
                        f['code']['cv_auto'], f['code']['mp_face'],
                        f['code']['mp_pose'], f['code']['re_none'],
                        f['code']['re_capt'], f['code']['re_reco'],
                        f['code']['mc_lock'], f['code']['mc_unlo'],
                        f['code']['led_off'], f['code']['led_aut'],
                        f['code']['led_ton'], f['code']['base_of'],
                        f['code']['base_on'], f['code']['head_ct'],
                        f['code']['base_ct']
                        ]

# CV info process
def process_cv_info(cmd):
    if cmd[f['fb']['detect_type']] != f['code']['cv_none']:
        print(cmd[f['fb']['detect_type']])
        pass

# Function to generate video frames from the camera
def generate_frames():
    while True:
        frame = cvf.frame_process()
        try:
            yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n') 
        except Exception as e:
            print("An [generate_frames] error occurred:", e)

# Route to render the HTML template
@app.route('/')
def index():
    audio_ctrl.play_random_audio("connected", False)
    return render_template('index.html')

@app.route('/config')
def get_config():
    with open(thisPath + '/ugv_rpi/config.yaml', 'r') as file:
        yaml_content = file.read()
    return yaml_content

# Get pictures and videos
@app.route('/<path:filename>')
def serve_static(filename):
    return send_from_directory('ugv_rpi/templates', filename)

@app.route('/get_photo_names')
def get_photo_names():
    photo_files = sorted(os.listdir(thisPath + '/ugv_rpi/templates/pictures'), key=lambda x: os.path.getmtime(os.path.join(thisPath + '/ugv_rpi/templates/pictures', x)), reverse=True)
    return jsonify(photo_files)

@app.route('/delete_photo', methods=['POST'])
def delete_photo():
    filename = request.form.get('filename')
    try:
        os.remove(os.path.join(thisPath + '/ugv_rpi/templates/pictures', filename))
        return jsonify(success=True)
    except Exception as e:
        print(e)
        return jsonify(success=False)

@app.route('/videos/<path:filename>')
def videos(filename):
    return send_from_directory(thisPath + '/ugv_rpi/templates/videos', filename)

@app.route('/get_video_names')
def get_video_names():
    video_files = sorted(
        [filename for filename in os.listdir(thisPath + '/ugv_rpi/templates/videos/') if filename.endswith('.mp4')],
        key=lambda filename: os.path.getctime(os.path.join(thisPath + '/ugv_rpi/templates/videos/', filename)),
        reverse=True
    )
    return jsonify(video_files)

@app.route('/delete_video', methods=['POST'])
def delete_video():
    filename = request.form.get('filename')
    try:
        os.remove(os.path.join(thisPath + '/ugv_rpi/templates/videos', filename))
        return jsonify(success=True)
    except Exception as e:
        print(e)
        return jsonify(success=False)

# WebRTC functions (same as original)
def manage_connections(pc_id):
    if len(active_pcs) >= MAX_CONNECTIONS:
        oldest_pc_id = min(active_pcs.keys())
        active_pcs[oldest_pc_id].close()
        del active_pcs[oldest_pc_id]

async def offer_async():
    pc = RTCPeerConnection()
    pc_id = str(uuid.uuid4())
    manage_connections(pc_id)
    active_pcs[pc_id] = pc

    @pc.on("connectionstatechange")
    async def on_connectionstatechange():
        if pc.connectionState == "failed":
            await pc.close()
            if pc_id in active_pcs:
                del active_pcs[pc_id]

    @pc.on("track")
    def on_track(track):
        if track.kind == "video":
            pc.addTrack(track)

    offer = await pc.createOffer()
    await pc.setLocalDescription(offer)
    return pc_id, offer

def offer():
    pc_id, offer = asyncio.run(offer_async())
    return jsonify({"pc_id": pc_id, "offer": offer.sdp})

# Command line control
def cmdline_ctrl(args_string):
    try:
        args = json.loads(args_string)
        if args['T'] == 1:
            base.base_speed_ctrl(args['L'], args['R'])
        elif args['T'] == 0:
            base.gimbal_emergency_stop()
        else:
            print(f"Unknown command: {args}")
    except Exception as e:
        print(f"Error processing command: {e}")

# Routes
@app.route('/offer', methods=['POST'])
def offer_route():
    return offer()

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/send_command', methods=['POST'])
def handle_command():
    data = request.get_json()
    cmdline_ctrl(json.dumps(data))
    return jsonify({"status": "success"})

@app.route('/getAudioFiles', methods=['GET'])
def get_audio_files():
    audio_files = os.listdir(UPLOAD_FOLDER)
    return jsonify(audio_files)

@app.route('/uploadAudio', methods=['POST'])
def upload_audio():
    if 'file' not in request.files:
        return jsonify({"error": "No file part"}), 400
    
    file = request.files['file']
    if file.filename == '':
        return jsonify({"error": "No selected file"}), 400
    
    if file:
        filename = secure_filename(file.filename)
        file.save(os.path.join(UPLOAD_FOLDER, filename))
        return jsonify({"success": True, "filename": filename})

@app.route('/playAudio', methods=['POST'])
def play_audio():
    data = request.get_json()
    filename = data.get('filename')
    audio_ctrl.play_audio(filename, False)
    return jsonify({"status": "success"})

@app.route('/stop_audio', methods=['POST'])
def audio_stop():
    audio_ctrl.stop_audio()
    return jsonify({"status": "success"})

@app.route('/settings/<path:filename>')
def serve_static_settings(filename):
    return send_from_directory('ugv_rpi/templates', filename)

# Socket.IO events
@socketio.on('json', namespace='/json')
def handle_socket_json(json):
    if json['T'] in cmd_actions:
        cmd_actions[json['T']]()
    elif json['T'] in cmd_feedback_actions:
        cmd_actions[json['T']]()

def update_data_websocket_single():
    # Mock data for compatibility
    data = {
        'T': 1001,
        'L': base.motor_controller.left_speed if hasattr(base, 'motor_controller') else 0,
        'R': base.motor_controller.right_speed if hasattr(base, 'motor_controller') else 0,
        'r': 0, 'p': 0, 'v': 12,
        'pan': 0, 'tilt': 0
    }
    socketio.emit('json', data, namespace='/json')

def update_data_loop():
    while True:
        update_data_websocket_single()
        time.sleep(0.1)

def base_data_loop():
    while True:
        try:
            data = base.feedback_data()
            socketio.emit('json', data, namespace='/json')
        except Exception as e:
            print(f"Error in base_data_loop: {e}")
        time.sleep(0.1)

@socketio.on('message', namespace='/ctrl')
def handle_socket_cmd(message):
    try:
        data = json.loads(message)
        cmdline_ctrl(message)
    except Exception as e:
        print(f"Error handling socket command: {e}")

def cmd_on_boot():
    base.base_oled(0, "Robotaxi Ready!")
    base.base_oled(1, "Direct Control Mode")
    base.base_oled(2, "No ESP32 Required")
    base.base_oled(3, "Web UI: Port 5000")

# Start background threads
threading.Thread(target=update_data_loop, daemon=True).start()
threading.Thread(target=base_data_loop, daemon=True).start()

# Run command on boot
cmd_on_boot()

if __name__ == '__main__':
    print("Starting Robotaxi with Direct Motor Control...")
    print("No ESP32 required - using GPIO pins directly")
    print("Web interface available at: http://localhost:5000")
    
    try:
        socketio.run(app, host='0.0.0.0', port=5000, debug=False)
    except KeyboardInterrupt:
        print("\nShutting down...")
    except Exception as e:
        print(f"Error starting server: {e}")
    finally:
        base.gimbal_dev_close() 
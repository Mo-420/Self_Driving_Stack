#!/usr/bin/env python3
"""
WaveShare Robotaxi Web Server
Handles Socket.IO communication between web app and robotaxi system
"""

from flask import Flask, render_template, request
from flask_socketio import SocketIO, emit
import json
import time
import threading
from datetime import datetime

app = Flask(__name__)
app.config['SECRET_KEY'] = 'waveshare_robotaxi_secret'
socketio = SocketIO(app, cors_allowed_origins="*")

# Global state
active_trips = {}
car_status = {
    'connected': False,
    'latitude': 51.505,
    'longitude': -0.09,
    'battery_level': 100,
    'speed': 0,
    'status': 'idle'
}

@app.route('/')
def index():
    return render_template('index.html')

@socketio.on('connect')
def handle_connect():
    print(f"Client connected: {request.sid}")
    emit('telemetry', {
        'latitude': car_status['latitude'],
        'longitude': car_status['longitude'],
        'battery_level': round(car_status['battery_level'], 1),
        'speed': car_status['speed'],
        'status': car_status['status'],
        'connected': car_status['connected']
    })

@socketio.on('disconnect')
def handle_disconnect():
    print(f"Client disconnected: {request.sid}")

@socketio.on('start_trip')
def handle_start_trip(data):
    """Handle trip start request from web app"""
    trip_id = data.get('trip_id')
    pickup = data.get('pickup')
    dropoff = data.get('dropoff')
    route = data.get('route')
    
    print(f"Starting trip {trip_id}: {pickup} -> {dropoff}")
    
    # Store trip data
    active_trips[trip_id] = {
        'pickup': pickup,
        'dropoff': dropoff,
        'route': route,
        'status': 'started',
        'start_time': datetime.now().isoformat(),
        'current_waypoint': 0
    }
    
    # Update car status
    car_status['status'] = 'navigating'
    
    # Emit trip update
    emit('trip_update', {
        'trip_id': trip_id,
        'status': 'started'
    }, broadcast=True)
    
    # TODO: Send route to robotaxi system
    # This would integrate with your existing robotaxi_enhanced.py or robotaxi_simple.py
    
    print(f"Trip {trip_id} started successfully")

@socketio.on('cancel_trip')
def handle_cancel_trip(data):
    """Handle trip cancellation"""
    trip_id = data.get('trip_id')
    
    if trip_id in active_trips:
        print(f"Cancelling trip {trip_id}")
        
        # Update trip status
        active_trips[trip_id]['status'] = 'cancelled'
        active_trips[trip_id]['end_time'] = datetime.now().isoformat()
        
        # Update car status
        car_status['status'] = 'idle'
        
        # Emit trip update
        emit('trip_update', {
            'trip_id': trip_id,
            'status': 'cancelled'
        }, broadcast=True)
        
        # TODO: Send stop command to robotaxi system
        
        print(f"Trip {trip_id} cancelled")

@socketio.on('emergency_stop')
def handle_emergency_stop():
    """Handle emergency stop request"""
    print("EMERGENCY STOP requested")
    
    # Update car status
    car_status['status'] = 'emergency_stop'
    car_status['speed'] = 0
    
    # Cancel all active trips
    for trip_id in list(active_trips.keys()):
        active_trips[trip_id]['status'] = 'emergency_stopped'
        active_trips[trip_id]['end_time'] = datetime.now().isoformat()
        
        emit('trip_update', {
            'trip_id': trip_id,
            'status': 'emergency_stopped'
        }, broadcast=True)
    
    # Clear active trips
    active_trips.clear()
    
    # TODO: Send emergency stop command to robotaxi system
    
    emit('emergency_stop_ack', {'status': 'stopped'}, broadcast=True)
    print("Emergency stop executed")

@socketio.on('update_car_position')
def handle_update_car_position(data):
    """Handle manual update of car's position from web app"""
    if 'latitude' in data and 'longitude' in data:
        car_status['latitude'] = data['latitude']
        car_status['longitude'] = data['longitude']
        print(f"Car position manually updated to: {data['latitude']}, {data['longitude']}")
        # The regular telemetry loop will broadcast the new position
    else:
        print("Invalid car position update received.")

def update_telemetry():
    """Simulate telemetry updates from robotaxi system"""
    while True:
        if car_status['status'] == 'navigating':
            # Simulate movement: slightly update position
            car_status['latitude'] += 0.0001
            car_status['longitude'] += 0.0001
            car_status['speed'] = 2.0  # m/s
        else:
            car_status['speed'] = 0.0
        
        # Simulate battery drain
        if car_status['battery_level'] > 0:
            car_status['battery_level'] -= 0.01
        
        # Emit telemetry to all connected clients
        socketio.emit('telemetry', {
            'latitude': car_status['latitude'],
            'longitude': car_status['longitude'],
            'battery_level': round(car_status['battery_level'], 1),
            'speed': car_status['speed'],
            'status': car_status['status'],
            'connected': car_status['connected']
        })
        
        time.sleep(1)  # Update every second

def start_telemetry_thread():
    """Start telemetry update thread"""
    telemetry_thread = threading.Thread(target=update_telemetry, daemon=True)
    telemetry_thread.start()

if __name__ == '__main__':
    print("Starting WaveShare Robotaxi Web Server...")
    print("Web app will be available at: http://localhost:5000")
    
    # Start telemetry thread
    start_telemetry_thread()
    
    # Start the server
    socketio.run(app, host='0.0.0.0', port=5000) 
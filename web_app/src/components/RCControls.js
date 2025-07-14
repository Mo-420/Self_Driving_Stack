import React, { useState, useEffect, useRef } from 'react';
import Joystick from 'react-nipple';

// react-nipple css (optional but gives cursor styles)
import 'react-nipple/lib/styles.css';
import './RCControls.css';

const RCControls = ({ socket }) => {
  const [direction, setDirection] = useState(null);
  const [touchMode, setTouchMode] = useState(true);
  const pressedKeys = useRef(new Set());
  const videoFeedUrl = `${window.location.protocol}//${window.location.hostname}:5000/video_feed`;

  // Keyboard controls
  useEffect(() => {
    const computeDirection = () => {
      const keys = pressedKeys.current;
      const w = keys.has('w');
      const a = keys.has('a');
      const s = keys.has('s');
      const d = keys.has('d');
      let dir = null;
      if (w && d) dir = 'FORWARD_RIGHT';
      else if (w && a) dir = 'FORWARD_LEFT';
      else if (s && d) dir = 'BACKWARD_RIGHT';
      else if (s && a) dir = 'BACKWARD_LEFT';
      else if (w) dir = 'FORWARD';
      else if (s) dir = 'BACKWARD';
      else if (d) dir = 'RIGHT';
      else if (a) dir = 'LEFT';
      return dir;
    };

    const emitDirection = (dir) => {
      setDirection(dir);
      if (socket) socket.emit('drive_command', { direction: dir || 'STOP' });
    };

    const handleKeyDown = (e) => {
      const key = e.key.toLowerCase();
      if (!['w', 'a', 's', 'd'].includes(key)) return;
      if (!pressedKeys.current.has(key)) {
        pressedKeys.current.add(key);
        emitDirection(computeDirection());
      }
    };

    const handleKeyUp = (e) => {
      const key = e.key.toLowerCase();
      if (!['w', 'a', 's', 'd'].includes(key)) return;
      if (pressedKeys.current.has(key)) {
        pressedKeys.current.delete(key);
        emitDirection(computeDirection());
      }
    };

    window.addEventListener('keydown', handleKeyDown);
    window.addEventListener('keyup', handleKeyUp);

    return () => {
      window.removeEventListener('keydown', handleKeyDown);
      window.removeEventListener('keyup', handleKeyUp);
    };
  }, [socket]);

  // DEBUG LOG
  console.log('DBG RCControls render', {direction});

  return (
    <div className="rc-overlay-container">
      <img src={videoFeedUrl} alt="Car Camera Feed" className="video-feed-background" />
      
      <div className="rc-header-overlay">
        <h2>Remote Drive</h2>
        <div className="current-direction-overlay">
          {direction ? `Moving: ${direction}` : 'Idle'}
        </div>
        <button
          className="toggle-controls-btn"
          onClick={() => setTouchMode(!touchMode)}
        >
          {touchMode ? 'Use WASD' : 'Use Touch'}
        </button>
      </div>

      {touchMode && (
      <div className="joystick-overlay">
        <Joystick
          size={200}
          options={{
            mode: 'static',
            color: 'white',
          }}
          onMove={(evt, data) => {
            if (!data.direction) return;
            // data.direction.angle can be 'up','down','left','right','up-left', etc.
            const dirMap = {
              up: 'FORWARD',
              down: 'BACKWARD',
              left: 'LEFT',
              right: 'RIGHT',
            };
            const mapped = dirMap[data.direction.angle] || null;
            if (mapped) {
              setDirection(mapped);
              if (socket) socket.emit('drive_command', { direction: mapped });
            }
          }}
          onEnd={() => {
            setDirection(null);
            if (socket) socket.emit('drive_command', { direction: 'STOP' });
          }}
        />
      </div>) }

      {!touchMode && (
      <div className="keyboard-guide-overlay">
        <h3>Use WASD to Drive</h3>
        <div className="key-layout">
          <div className="key">W</div>
          <div className="key-row">
            <div className="key">A</div>
            <div className="key">S</div>
            <div className="key">D</div>
          </div>
        </div>
      </div>
      )}
    </div>
  );
};

export default RCControls; 
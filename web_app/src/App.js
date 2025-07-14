import React, { useState, useEffect } from 'react';
import { MapContainer, TileLayer, Marker, Polyline, useMapEvents } from 'react-leaflet';
import { io } from 'socket.io-client';
import 'leaflet/dist/leaflet.css';

const DEFAULT_CENTER = [51.505, -0.09]; // Cambridge, UK
const DEFAULT_ZOOM = 15;

// Socket.IO connection
let socket = null;

function LocationSelector({ onSelect, pickUp, dropOff }) {
  useMapEvents({
    click(e) {
      if (!pickUp) {
        onSelect('pickUp', e.latlng);
      } else if (!dropOff) {
        onSelect('dropOff', e.latlng);
      }
    },
  });
  return null;
}

const ORS_API_KEY = 'eyJvcmciOiI1YjNjZTM1OTc4NTExMTAwMDFjZjYyNDgiLCJpZCI6ImZjMmFmZjU2ODFiMzQwY2RiYTUzMGExYTdhZGE1MGY4IiwiaCI6Im11cm11cjY0In0=';

const fetchRoute = async (start, end) => {
  const url = `https://api.openrouteservice.org/v2/directions/driving-car?api_key=${ORS_API_KEY}&start=${start[1]},${start[0]}&end=${end[1]},${end[0]}`;
  const response = await fetch(url);
  if (!response.ok) throw new Error('Failed to fetch route');
  const data = await response.json();
  if (!data.features || !data.features[0]) throw new Error('No route found');
  // Decode polyline to array of [lat, lng]
  return data.features[0].geometry.coordinates.map(([lng, lat]) => [lat, lng]);
};

function App() {
  const [pickUp, setPickUp] = useState(null);
  const [dropOff, setDropOff] = useState(null);
  const [route, setRoute] = useState([]);
  const [status, setStatus] = useState('idle');
  const [carPosition, setCarPosition] = useState(null);
  const [carStatus, setCarStatus] = useState('disconnected');
  const [batteryLevel, setBatteryLevel] = useState(null);
  const [currentSpeed, setCurrentSpeed] = useState(0);
  const [tripId, setTripId] = useState(null);

  // Initialize Socket.IO connection
  useEffect(() => {
    socket = io('http://localhost:5000', {
      transports: ['websocket', 'polling']
    });

    socket.on('connect', () => {
      console.log('Connected to robotaxi server');
      setCarStatus('connected');
    });

    socket.on('disconnect', () => {
      console.log('Disconnected from robotaxi server');
      setCarStatus('disconnected');
    });

    socket.on('telemetry', (data) => {
      setCarPosition([data.latitude, data.longitude]);
      setBatteryLevel(data.battery_level);
      setCurrentSpeed(data.speed);
      setCarStatus(data.status);
    });

    socket.on('trip_update', (data) => {
      setStatus(data.status);
      if (data.status === 'completed') {
        setTripId(null);
      }
    });

    return () => {
      if (socket) {
        socket.disconnect();
      }
    };
  }, []);

  const handleSelect = (type, latlng) => {
    if (type === 'pickUp') setPickUp([latlng.lat, latlng.lng]);
    if (type === 'dropOff') setDropOff([latlng.lat, latlng.lng]);
  };

  const handleReset = () => {
    setPickUp(null);
    setDropOff(null);
    setRoute([]);
    setStatus('idle');
    setTripId(null);
  };

  const handleRoute = async () => {
    if (pickUp && dropOff) {
      setStatus('routing...');
      try {
        const coords = await fetchRoute(pickUp, dropOff);
        setRoute(coords);
        setStatus('route_ready');
      } catch (e) {
        setStatus('error');
        alert('Failed to fetch route: ' + e.message);
      }
    }
  };

  const handleStartTrip = () => {
    if (route.length > 0 && socket) {
      const tripData = {
        trip_id: Date.now().toString(),
        pickup: pickUp,
        dropoff: dropOff,
        route: route,
        timestamp: new Date().toISOString()
      };
      
      socket.emit('start_trip', tripData);
      setTripId(tripData.trip_id);
      setStatus('trip_started');
    }
  };

  const handleCancelTrip = () => {
    if (socket && tripId) {
      socket.emit('cancel_trip', { trip_id: tripId });
      setStatus('cancelled');
      setTripId(null);
    }
  };

  const handleEmergencyStop = () => {
    if (socket) {
      socket.emit('emergency_stop');
      setStatus('emergency_stop');
    }
  };

  return (
    <div style={{ height: '100vh', width: '100vw' }}>
      {/* Control Panel */}
      <div style={{ 
        position: 'absolute', 
        top: 10, 
        left: 10, 
        zIndex: 1000, 
        background: 'white', 
        padding: 15, 
        borderRadius: 8,
        boxShadow: '0 2px 10px rgba(0,0,0,0.1)',
        minWidth: '250px'
      }}>
        <h3 style={{ margin: '0 0 10px 0', color: '#333' }}>WaveShare Robotaxi</h3>
        
        {/* Status Indicators */}
        <div style={{ marginBottom: 15 }}>
          <div><b>Car Status:</b> 
            <span style={{ 
              color: carStatus === 'connected' ? 'green' : 'red',
              marginLeft: 5
            }}>
              {carStatus}
            </span>
          </div>
          <div><b>Trip Status:</b> {status}</div>
          {batteryLevel && <div><b>Battery:</b> {batteryLevel}%</div>}
          {currentSpeed > 0 && <div><b>Speed:</b> {currentSpeed.toFixed(1)} m/s</div>}
        </div>

        {/* Trip Controls */}
        <div style={{ marginBottom: 15 }}>
          <button 
            onClick={handleReset}
            style={{ marginRight: 5, padding: '5px 10px' }}
          >
            Reset
          </button>
          <button 
            onClick={handleRoute} 
            disabled={!(pickUp && dropOff)}
            style={{ marginRight: 5, padding: '5px 10px' }}
          >
            Plan Route
          </button>
          <button 
            onClick={handleStartTrip}
            disabled={status !== 'route_ready'}
            style={{ marginRight: 5, padding: '5px 10px', backgroundColor: '#4CAF50', color: 'white' }}
          >
            Start Trip
          </button>
        </div>

        {/* Emergency Controls */}
        <div style={{ marginBottom: 15 }}>
          <button 
            onClick={handleCancelTrip}
            disabled={!tripId}
            style={{ marginRight: 5, padding: '5px 10px', backgroundColor: '#ff9800', color: 'white' }}
          >
            Cancel Trip
          </button>
          <button 
            onClick={handleEmergencyStop}
            style={{ padding: '5px 10px', backgroundColor: '#f44336', color: 'white' }}
          >
            E-STOP
          </button>
        </div>

        {/* Instructions */}
        <div style={{ fontSize: '12px', color: '#666' }}>
          <p><b>Instructions:</b></p>
          <p>1. Click map to set pickup point</p>
          <p>2. Click map to set dropoff point</p>
          <p>3. Plan route and start trip</p>
        </div>
      </div>

      {/* Map */}
      <MapContainer 
        center={DEFAULT_CENTER} 
        zoom={DEFAULT_ZOOM} 
        style={{ height: '100%', width: '100%' }}
      >
        <TileLayer url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png" />
        <LocationSelector onSelect={handleSelect} pickUp={pickUp} dropOff={dropOff} />
        
        {/* Markers */}
        {pickUp && <Marker position={pickUp} />}
        {dropOff && <Marker position={dropOff} />}
        {carPosition && (
          <Marker 
            position={carPosition} 
            icon={L.divIcon({
              className: 'car-marker',
              html: '🚗',
              iconSize: [30, 30]
            })}
          />
        )}
        
        {/* Route */}
        {route.length > 1 && <Polyline positions={route} color="blue" weight={3} />}
      </MapContainer>
    </div>
  );
}

export default App;

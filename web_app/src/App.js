import React, { useState, useEffect, useRef } from 'react';
import { MapContainer, TileLayer, Marker, Polyline, useMapEvents, useMap } from 'react-leaflet';
import { io } from 'socket.io-client';
import 'leaflet/dist/leaflet.css';
import L from 'leaflet';
import BottomNavigation from '@mui/material/BottomNavigation';
import BottomNavigationAction from '@mui/material/BottomNavigationAction';
import MapIcon from '@mui/icons-material/Map';
import DirectionsCarIcon from '@mui/icons-material/DirectionsCar';
import MicIcon from '@mui/icons-material/Mic';
import Paper from '@mui/material/Paper';
import './App.css'; // Make sure App.css is imported
import RCControls from './components/RCControls'; // Import the new component

const DEFAULT_CENTER = [51.505, -0.09]; // Cambridge, UK
const DEFAULT_ZOOM = 15;

// Socket.IO connection
let socket = null;

function LocationSelector({ onSelectDestination, onSetCarLocation, isSelecting, isSettingCar }) {
  useMapEvents({
    click(e) {
      if (isSettingCar) {
        onSetCarLocation(e.latlng);
      } else if (isSelecting) {
        onSelectDestination(e.latlng);
      }
    },
  });
  return null;
}

function UserLocationMarker() {
  const [position, setPosition] = useState(null);
  const map = useMap();

  useEffect(() => {
    if (navigator.geolocation) {
      const watcher = navigator.geolocation.watchPosition(
        (pos) => {
          const { latitude, longitude } = pos.coords;
          setPosition([latitude, longitude]);
          map.setView([latitude, longitude], map.getZoom());
        },
        (err) => console.error('Geolocation error:', err),
        { enableHighAccuracy: true }
      );

      return () => navigator.geolocation.clearWatch(watcher);
    }
  }, [map]);

  return position ? (
    <Marker 
      position={position} 
      icon={L.divIcon({
        className: 'user-marker',
        html: '<div style="background-color: blue; width: 15px; height: 15px; border-radius: 50%; border: 3px solid white;"></div>',
        iconSize: [15, 15],
        iconAnchor: [7.5, 7.5]
      })}
    />
  ) : null;
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

// useResizeObserver removed as no longer needed

function App() {
  const [view, setView] = useState('map');
  const [userPosition, setUserPosition] = useState(null);
  const [carPosition, setCarPosition] = useState(null);
  const [destination, setDestination] = useState(null);
  const [route, setRoute] = useState([]);
  const [status, setStatus] = useState('idle');
  const [backendStatus, setBackendStatus] = useState('disconnected');
  const [carConnected, setCarConnected] = useState(false);
  const [batteryLevel, setBatteryLevel] = useState(null);
  const [currentSpeed, setCurrentSpeed] = useState(0);
  const [tripId, setTripId] = useState(null);
  const [isSettingCarLocation, setIsSettingCarLocation] = useState(false);
  const mapRef = useRef();
  const controlPanelRef = useRef(null);

  // Initialize Socket.IO connection
  useEffect(() => {
    socket = io('http://localhost:5000', {
      transports: ['websocket', 'polling']
    });

    socket.on('connect', () => {
      console.log('Connected to backend server');
      setBackendStatus('connected');
    });

    socket.on('disconnect', () => {
      console.log('Disconnected from backend server');
      setBackendStatus('disconnected');
      setCarConnected(false);
    });

    socket.on('telemetry', (data) => {
      setCarPosition([data.latitude, data.longitude]);
      setBatteryLevel(data.battery_level);
      setCurrentSpeed(data.speed);
      setStatus(data.status);
      setCarConnected(data.connected);
    });

    socket.on('trip_update', (data) => {
      setStatus(data.status);
      if (data.status === 'completed') {
        setTripId(null);
        setRoute([]);
        setDestination(null);
      }
    });

    return () => {
      if (socket) {
        socket.disconnect();
      }
    };
  }, []);

  // Get user's initial position
  useEffect(() => {
    if (navigator.geolocation) {
      navigator.geolocation.getCurrentPosition(
        (pos) => {
          setUserPosition([pos.coords.latitude, pos.coords.longitude]);
        },
        (err) => console.error('Geolocation error:', err)
      );
    }
  }, []);

  useEffect(() => {
    if (mapRef.current) {
      mapRef.current.invalidateSize();
    }
  }, [view]); // Trigger on view change or as needed

  const handleSelectDestination = (latlng) => {
    setDestination([latlng.lat, latlng.lng]);
  };

  const handleSetCarLocation = (latlng) => {
    const newPos = [latlng.lat, latlng.lng];
    setCarPosition(newPos);
    if (socket) {
      socket.emit('update_car_position', { latitude: newPos[0], longitude: newPos[1] });
    }
    setIsSettingCarLocation(false); // Exit setting mode after placing car
  };

  const handlePlanRoute = async () => {
    if (!carPosition) {
      alert('Waiting for car position. If simulating, check backend telemetry.');
      return;
    }
    if (!destination) {
      alert('Please select a destination on the map.');
      return;
    }
    setStatus('routing...');
    try {
      const coords = await fetchRoute(carPosition, destination);
      setRoute(coords);
      setStatus('route_ready');
    } catch (e) {
      setStatus('error');
      alert('Failed to fetch route: ' + e.message + '. Check your ORS API key in App.js.');
    }
  };

  const handleStartTrip = () => {
    if (route.length > 0 && socket) {
      const tripData = {
        trip_id: Date.now().toString(),
        destination: destination,
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
      setRoute([]);
      setDestination(null);
    }
  };

  const handleEmergencyStop = () => {
    if (socket) {
      socket.emit('emergency_stop');
      setStatus('emergency_stop');
      setRoute([]);
      setDestination(null);
    }
  };

  const handleReset = () => {
    setDestination(null);
    setRoute([]);
    setStatus('idle');
    setTripId(null);
  };

  const handleRecenter = () => {
    if (mapRef.current) {
      const map = mapRef.current;
      const target = userPosition || DEFAULT_CENTER;
      map.setView(target);
    }
  };

  // Custom zoom handlers
  const handleZoomIn = () => {
    const map = mapRef.current;
    if (map && map._leaflet_id) {
      map.setZoom(map.getZoom() + 1);
    } else if (map && map.leafletElement) {
      map.leafletElement.setZoom(map.leafletElement.getZoom() + 1);
    }
  };
  const handleZoomOut = () => {
    const map = mapRef.current;
    if (map && map._leaflet_id) {
      map.setZoom(map.getZoom() - 1);
    } else if (map && map.leafletElement) {
      map.leafletElement.setZoom(map.leafletElement.getZoom() - 1);
    }
  };

  // DEBUG LOG
  console.log('DBG App render', {backendStatus, carConnected, batteryLevel, currentSpeed, carPosition, destination, route});

  return (
    <div style={{ height: '100vh', width: '100vw', display: 'flex', flexDirection: 'column' }}>
      {/* Global Page Background */}
      <div className="page-background">
        <div className="blob blob-1"></div>
        <div className="blob blob-2"></div>
        <div className="blob blob-3"></div>
        {/* Add more page blobs if defined in CSS */}
      </div>

      {/* Main Content Area */}
      <div style={{ flex: 1, position: 'relative' }}>
        {view === 'map' && (
          <div className="map-view-container">
            {/* Sidebar on the left */}
            <div className="sidebar">
              <div ref={controlPanelRef} className="control-panel">
                <div className="control-panel-content">
                  
                  {/* Status Indicators */}
                  <div style={{ marginBottom: 15 }}>
                    <div className="status-label"><b>Server:</b> <span className="status-value" style={{ color: backendStatus === 'connected' ? 'green' : 'red' }}>{backendStatus}</span></div>
                    <div className="status-label"><b>Car:</b> <span className="status-value" style={{ color: carConnected ? 'green' : 'red' }}>{carConnected ? 'Connected' : 'Not Connected'}</span></div>
                    {carConnected && (
                      <>
                        <div className="status-label"><b>Trip Status:</b> <span className="status-value">{status}</span></div>
                        {batteryLevel !== null && <div className="status-label"><b>Battery:</b> <span className="status-value">{batteryLevel.toFixed(1)}%</span></div>}
                        {currentSpeed > 0 && <div className="status-label"><b>Speed:</b> <span className="status-value">{currentSpeed.toFixed(1)} m/s</span></div>}
                      </>
                    )}
                  </div>

                  {/* Trip Controls */}
                  <div className="button-group">
                    <button className="button button-secondary" onClick={handleReset}>Clear</button>
                    <button className="button button-set-car" onClick={() => setIsSettingCarLocation(true)}>Set Car Position</button>
                    <button className="button button-primary" onClick={handlePlanRoute} disabled={!destination || !carPosition}>Find Path</button>
                    <button className="button button-success" onClick={handleStartTrip} disabled={status !== 'route_ready' || !carPosition}>Let's Go</button>
                  </div>

                  {/* Emergency Controls */}
                  <div className="button-group">
                    <button className="button button-warning" onClick={handleCancelTrip} disabled={!tripId}>Stop Trip</button>
                    <button className="button button-danger" onClick={handleEmergencyStop}>Emergency Stop</button>
                  </div>
                </div>
              </div>
            </div>

            {/* Map Box on the right */}
            <div className="map-box">
              {isSettingCarLocation && <div className="map-prompt">Click on the map to place the car</div>}
              <MapContainer 
                ref={mapRef}
                attributionControl={false} /* Removes the Leaflet attribution */
                zoomControl={false} /* Removes the default zoom control */
                center={userPosition || DEFAULT_CENTER} 
                zoom={DEFAULT_ZOOM} 
                style={{ height: '100%', width: '100%' }}
              >
                <TileLayer url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png" />
                <LocationSelector 
                  onSelectDestination={handleSelectDestination} 
                  onSetCarLocation={handleSetCarLocation}
                  isSelecting={!tripId} 
                  isSettingCar={isSettingCarLocation}
                />
                <UserLocationMarker />
                
                {carPosition && <Marker position={carPosition} icon={L.divIcon({ className: 'car-marker', html: '🚗', iconSize: [30, 30], iconAnchor: [15, 15] })} />}
                {destination && <Marker position={destination} icon={L.divIcon({ className: 'destination-marker', html: '📍', iconSize: [30, 30], iconAnchor: [15, 30] })} />}
                {route.length > 1 && <Polyline positions={route} color="blue" weight={5} />}
              </MapContainer>
              {/* Controls stack overlay */}
              <div className="map-controls-stack">
                <button className="map-control-btn" onClick={handleZoomIn} title="Zoom In" aria-label="Zoom In">+</button>
                <button className="map-control-btn" onClick={handleZoomOut} title="Zoom Out" aria-label="Zoom Out">-</button>
                <button 
                  className="map-control-btn" 
                  onClick={handleRecenter} 
                  title="Recenter"
                  aria-label="Recenter"
                >
                  <svg width="22" height="22" viewBox="0 0 22 22" fill="none" xmlns="http://www.w3.org/2000/svg">
                    <path d="M11 2v2m0 14v2m9-9h-2M4 11H2m15.07-6.93l-1.41 1.41M6.34 17.66l-1.41 1.41m12.02 0l-1.41-1.41M6.34 4.34L4.93 2.93" stroke="#111" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
                    <circle cx="11" cy="11" r="5" stroke="#111" strokeWidth="2"/>
                  </svg>
                </button>
              </div>
            </div>
          </div>
        )}

        {view === 'control' && (
          <div className="map-view-container">
            <RCControls socket={socket} />
          </div>
        )}

        {view === 'voice' && (
          <div className="map-view-container">
            <h2>Talk to Robotaxi - Coming Soon</h2>
          </div>
        )}
      </div>

      {/* Bottom Navigation */}
      <Paper className="bottom-nav-glass" sx={{ position: 'fixed', bottom: 0, left: 0, right: 0, background: 'transparent', backdropFilter: 'none' }} elevation={3}>
        <BottomNavigation
          showLabels
          value={view}
          onChange={(event, newValue) => {
            setView(newValue);
          }}
          sx={{ background: 'transparent' }}
        >
          <BottomNavigationAction label="Navigate" value="map" icon={<MapIcon />} />
          <BottomNavigationAction label="Drive" value="control" icon={<DirectionsCarIcon />} />
          <BottomNavigationAction label="Talk" value="voice" icon={<MicIcon />} />
        </BottomNavigation>
      </Paper>
    </div>
  );
}

export default App;

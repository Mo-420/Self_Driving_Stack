# WaveShare Robotaxi Web App

A modern web interface for controlling and monitoring the WaveShare Robotaxi system.

## Features

- **Interactive Map**: Click to set pickup and dropoff points
- **Route Planning**: Automatic route calculation using OpenRouteService
- **Real-time Telemetry**: Live car position, battery level, and speed updates
- **Trip Management**: Start, cancel, and monitor trips
- **Emergency Stop**: Instant emergency stop functionality
- **Socket.IO Communication**: Real-time bidirectional communication with the robotaxi

## Architecture

```
Web App (React + Leaflet)
    ↓ Socket.IO
Flask Backend Server
    ↓ (Future integration)
WaveShare Robotaxi System (Pi)
```

## Quick Start

### Prerequisites

- Node.js (v14 or higher)
- npm
- Python 3.7+
- pip3

### Installation & Startup

1. **Navigate to the web app directory:**
   ```bash
   cd web_app
   ```

2. **Run the startup script:**
   ```bash
   ./start_web_app.sh
   ```

   This will:
   - Install React dependencies
   - Install Python dependencies
   - Start the Flask backend server (port 5000)
   - Start the React development server (port 3000)

3. **Open your browser:**
   - Frontend: http://localhost:3000
   - Backend API: http://localhost:5000

## Manual Setup (Alternative)

If you prefer to run components separately:

### Backend Server
```bash
cd web_app
pip3 install -r requirements.txt
python3 server.py
```

### Frontend App
```bash
cd web_app
npm install
npm start
```

## Usage

### 1. Setting Up a Trip
1. Open the web app in your browser
2. Click on the map to set your pickup point (first click)
3. Click on the map to set your dropoff point (second click)
4. Click "Plan Route" to calculate the optimal route
5. Click "Start Trip" to begin the journey

### 2. Monitoring the Trip
- **Car Position**: The car icon (🚗) shows the current position
- **Battery Level**: Real-time battery percentage
- **Speed**: Current speed in meters per second
- **Status**: Current trip status (idle, navigating, completed, etc.)

### 3. Trip Controls
- **Cancel Trip**: Stop the current trip safely
- **E-STOP**: Emergency stop - immediately halts all movement
- **Reset**: Clear the map and start over

## Integration with WaveShare Robotaxi

### Current Status
The web app is currently set up with a simulated backend that demonstrates the interface and communication protocol.

### Future Integration
To connect with the actual WaveShare robotaxi system:

1. **Modify `server.py`** to communicate with your robotaxi system:
   ```python
   # In handle_start_trip function:
   # TODO: Send route to robotaxi system
   # This would integrate with your existing robotaxi_enhanced.py or robotaxi_simple.py
   ```

2. **Update telemetry** to use real data from the Pi:
   ```python
   # In update_telemetry function:
   # Replace simulation with real data from robotaxi system
   ```

3. **Add hardware communication** to send commands to the WaveShare chassis

## API Endpoints

### Socket.IO Events

#### From Web App to Server
- `start_trip`: Start a new trip with route data
- `cancel_trip`: Cancel the current trip
- `emergency_stop`: Emergency stop command

#### From Server to Web App
- `telemetry`: Real-time car data (position, battery, speed, status)
- `trip_update`: Trip status updates
- `emergency_stop_ack`: Emergency stop acknowledgment

### Trip Data Format
```json
{
  "trip_id": "1234567890",
  "pickup": [51.505, -0.09],
  "dropoff": [51.51, -0.1],
  "route": [[51.505, -0.09], [51.51, -0.1]],
  "timestamp": "2024-01-01T12:00:00Z"
}
```

### Telemetry Data Format
```json
{
  "latitude": 51.505,
  "longitude": -0.09,
  "battery_level": 85.5,
  "speed": 2.0,
  "status": "navigating"
}
```

## Development

### Project Structure
```
web_app/
├── src/                    # React source code
│   ├── App.js             # Main React component
│   └── ...
├── public/                 # Static assets
├── templates/              # Flask templates
│   └── index.html
├── server.py              # Flask-SocketIO backend
├── requirements.txt       # Python dependencies
├── package.json           # Node.js dependencies
└── start_web_app.sh      # Startup script
```

### Customization

#### Changing the Map Center
Edit `src/App.js`:
```javascript
const DEFAULT_CENTER = [YOUR_LAT, YOUR_LNG];
const DEFAULT_ZOOM = 15;
```

#### Adding New Features
1. Add new Socket.IO events in `server.py`
2. Update the React component in `src/App.js`
3. Test the integration

## Troubleshooting

### Common Issues

1. **Port 3000 or 5000 already in use:**
   ```bash
   # Find and kill the process
   lsof -ti:3000 | xargs kill -9
   lsof -ti:5000 | xargs kill -9
   ```

2. **Node.js dependencies not installing:**
   ```bash
   rm -rf node_modules package-lock.json
   npm install
   ```

3. **Python dependencies not installing:**
   ```bash
   pip3 install --upgrade pip
   pip3 install -r requirements.txt
   ```

### Logs
- Backend logs are printed to the console
- Frontend logs are available in the browser's developer tools

## Contributing

1. Make changes to the code
2. Test the functionality
3. Update documentation as needed
4. Commit and push changes

## License

This project is part of the WaveShare Robotaxi system.

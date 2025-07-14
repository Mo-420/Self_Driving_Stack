# SD_Cambridge_web_app Frontend

This directory contains the minimal React + Leaflet web app for the Robotaxi global-route UI.

## Structure
```
frontend/
├── public/
│   └── index.html
├── src/
│   ├── App.js
│   ├── index.js
│   ├── components/
│   │   └── MapView.js
│   └── ...
├── package.json
└── README.md
```

## Features
- Leaflet map for pick-up/drop-off selection
- Calls `/api/route` to request a route
- Draws route polyline on map
- Connects to Socket.IO for real-time events
- Polls `/api/status/:id` for trip status

## Setup
1. `npm install`
2. `npm start`

## Next Steps
- Scaffold React app (create-react-app or Vite)
- Add Leaflet map component
- Implement API calls and Socket.IO client
- Add UI for trip status and route visualization 
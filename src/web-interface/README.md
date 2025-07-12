# Robotics Controller Web Interface

A modern, real-time web interface for monitoring and controlling the robotics system.

## Features

- **Real-time Sensor Monitoring**: Distance, IMU, line sensors, IR, GPS, and button status
- **Robot Control**: Movement commands, speed control, emergency stop
- **Actuator Control**: Motor and servo control with real-time feedback
- **System Monitoring**: Status indicators and performance metrics
- **Logging**: Command history and system events with filtering
- **Modern UI**: Responsive design with dark theme

## Usage

### Starting the Web Interface

```bash
# Build and run the robotics controller
cmake --build build --parallel $(nproc)
./build/bin/robotics-controller

# Access the web interface at http://localhost:8080
```

### Controls

- **Movement**: Use arrow buttons or keyboard (WASD + Space for stop)
- **Speed**: Adjust with slider (10-100%)
- **Modes**: Switch between Manual and Autonomous
- **Emergency**: Red stop button for immediate halt

### API Endpoints

- `GET /api/sensors` - Sensor data
- `GET /api/status` - System status
- `POST /api/command` - Movement commands
- `POST /api/actuator` - Motor/servo control

## Files

- `index.html` - Main interface
- `script.js` - Application logic
- `styles.css` - Styling
- `README.md` - Documentation

## Troubleshooting

1. **Interface not loading**: Check if controller is running on port 8080
2. **No sensor data**: Verify API endpoints are responding
3. **Commands not working**: Check browser console for errors

For technical support, refer to the main project documentation.

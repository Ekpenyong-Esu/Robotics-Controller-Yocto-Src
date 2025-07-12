/**
 * ROBOTICS CONTROLLER WEB INTERFACE JAVASCRIPT
 * ============================================
 *
 * This JavaScript file provides the complete interactive functionality
 * for a robotics controller web interface. It handles:
 *
 * - Real-time communication with robotics hardware
 * - User interface controls and feedback
 * - Sensor data visualization
 * - System monitoring and logging
 * - Error handling and user notifications
 *
 * Structure:
 * 1. Main RoboticsController class
 * 2. Initialization and setup methods
 * 3. Communication and API methods
 * 4. UI update and event handling
 * 5. Sensor data processing
 * 6. Utility functions
 */

/**
 * MAIN ROBOTICS CONTROLLER CLASS
 * =============================
 *
 * This class encapsulates all the functionality for the robotics controller
 * interface. It manages state, handles communication, and updates the UI.
 */
class RoboticsController {
    constructor() {
        // === CORE CONNECTION SETTINGS ===
        // Base URL for API communication (same origin as the web interface)
        this.baseUrl = window.location.origin;

        // === CONNECTION STATE ===
        this.isConnected = false; // Track connection to robotics hardware
        this.currentMode = "unknown"; // Current operational mode (manual, auto, etc.)
        this.lastKnownMode = null; // Previous mode for comparison

        // === CONTROL SETTINGS ===
        this.currentSpeed = 50; // Current movement speed (0-100%)

        // === UPDATE INTERVALS ===
        // These handle periodic updates for real-time data
        this.updateInterval = null; // Main UI update timer
        this.sensorUpdateInterval = null; // Sensor data update timer

        // === LOGGING SYSTEM ===
        this.logEntries = []; // Array to store log messages
        this.maxLogEntries = 1000; // Limit log entries to prevent memory issues

        // === SYSTEM MONITORING ===
        this.lowBatteryWarned = false; // Prevent repeated low battery warnings
        this.startTime = Date.now(); // Track session start time
        this.lastDataUpdate = 0; // Timestamp of last data update
        this.dataUpdateCount = 0; // Count of data updates for diagnostics
        this.lastSensorData = null; // Cache of last sensor reading

        // === SENSOR DATA STRUCTURE ===
        // Organized storage for all sensor readings with validation
        this.sensorData = {
            // Distance sensor (ultrasonic, lidar, etc.)
            distance: {
                value: 0, // Distance reading in appropriate units
                valid: false, // Whether the reading is valid/reliable
                timestamp: 0, // When the reading was taken
            },

            // Inertial Measurement Unit (IMU) - motion sensing
            imu: {
                // Accelerometer data (m/s²)
                accel: { x: 0, y: 0, z: 0 },
                // Gyroscope data (rad/s or deg/s)
                gyro: { x: 0, y: 0, z: 0 },
                // Magnetometer data (magnetic field strength)
                mag: { x: 0, y: 0, z: 0 },
                valid: false,
                timestamp: 0,
            },

            // Line following sensors (array of values)
            line: {
                values: [], // Array of sensor readings
                valid: false,
                timestamp: 0,
            },

            // Infrared sensors for obstacle detection
            ir: {
                left: false, // Left IR sensor state
                right: false, // Right IR sensor state
                valid: false,
                timestamp: 0,
            },

            // GPS positioning system
            gps: {
                latitude: 0, // GPS latitude coordinate
                longitude: 0, // GPS longitude coordinate
                altitude: 0, // GPS altitude reading
                valid: false,
                timestamp: 0,
            },

            // Physical button/switch inputs
            button: {
                pressed: false, // Button state
                valid: false,
                timestamp: 0,
            },
        };

        // === SYSTEM STATE MONITORING ===
        // Overall system health and status information
        this.systemState = {
            battery: {
                voltage: 0, // Battery voltage reading
                percentage: 0, // Calculated battery percentage
            },
            temperature: 0, // System temperature
            uptime: 0, // System uptime in seconds
            errors: [], // Array of current error messages
            warnings: [], // Array of current warning messages
        };

        // === INITIALIZATION LOGGING ===
        // Log the startup of the interface for debugging
        this.addLog(
            "🚀 Robotics Controller Web Interface starting up...",
            "info"
        );

        // Start the initialization process
        this.init();
    }

    /**
     * INITIALIZATION METHOD
     * ====================
     *
     * Sets up the entire interface including:
     * - Event listeners for user interactions
     * - Periodic update timers
     * - Connection status monitoring
     * - Logging and notification systems
     */
    init() {
        // Setup user interface event handlers
        this.setupEventListeners();

        // Start real-time data updates
        this.startPeriodicUpdates();

        // Initialize connection status
        this.updateConnectionStatus();

        // Setup logging and notification systems
        this.setupLogSystem();
        this.setupNotificationSystem();

        // Update system information display
        this.updateSystemInfo();

        this.addLog("Web interface initialized", "success");

        // === PAGE VISIBILITY HANDLER ===
        // Update status when user returns to the tab
        document.addEventListener("visibilitychange", () => {
            if (!document.hidden) {
                this.updateStatus();
            }
        });

        // Force an initial status update
        this.updateStatus();
    }

    /**
     * EVENT LISTENERS SETUP
     * =====================
     *
     * Sets up all user interface event handlers including:
     * - Movement control buttons
     * - Mode switching buttons
     * - Emergency stop functionality
     * - Settings and configuration controls
     */
    setupEventListeners() {
        // === MOVEMENT CONTROL BUTTONS ===
        // Handle directional movement buttons (forward, backward, left, right, stop)
        document.querySelectorAll(".movement-btn").forEach((button) => {
            // Extract command from button ID (e.g., "btn-forward" -> "forward")
            const command = button.id.replace("btn-", "");

            // === MOUSE DOWN EVENT ===
            // Visual feedback when button is pressed
            button.addEventListener("mousedown", () => {
                button.classList.add("pressed");
                // Add directional class to parent container for additional styling
                if (command !== "stop") {
                    document
                        .querySelector(".movement-controls")
                        .classList.add(`active-${command}`);
                }
            });

            // === MOUSE UP EVENT ===
            // Remove visual feedback when button is released
            button.addEventListener("mouseup", () => {
                button.classList.remove("pressed");
                document
                    .querySelector(".movement-controls")
                    .classList.remove(`active-${command}`);
            });

            // === MOUSE LEAVE EVENT ===
            // Clean up if mouse leaves button while pressed
            button.addEventListener("mouseleave", () => {
                button.classList.remove("pressed");
                document
                    .querySelector(".movement-controls")
                    .classList.remove(`active-${command}`);
            });
        });

        // === SPEED CONTROL SLIDER ===
        // Slider that controls the movement speed (0-100%)
        const speedSlider = document.getElementById("speed-slider");
        const speedValue = document.getElementById("speed-value");
        const speedControl = document.querySelector(".speed-control");

        if (speedSlider && speedValue) {
            // === SPEED SLIDER INPUT EVENT ===
            // Update speed value in real-time as user moves slider
            speedSlider.addEventListener("input", (e) => {
                const value = e.target.value;

                // Update the displayed speed value
                speedValue.textContent = value;

                // Set CSS data attribute for styling purposes
                if (speedControl)
                    speedControl.setAttribute("data-speed", value);

                // Store the speed value in our controller state
                this.currentSpeed = parseInt(value);
            });

            // === INITIALIZE SPEED SETTING ===
            // Set initial speed from slider's default value
            this.currentSpeed = parseInt(speedSlider.value);
            if (speedControl)
                speedControl.setAttribute("data-speed", this.currentSpeed);
        }

        // === MODE CONTROL BUTTONS ===
        // Buttons for switching between operational modes
        const autonomousBtn = document.getElementById("btn-autonomous");
        const manualBtn = document.getElementById("btn-manual");
        const emergencyBtn = document.getElementById("btn-emergency");

        // === AUTONOMOUS MODE BUTTON ===
        // Switch to autonomous operation where robot follows programmed behavior
        if (autonomousBtn) {
            autonomousBtn.addEventListener("click", () => {
                this.sendCommand("auto");
            });
        }

        // === MANUAL MODE BUTTON ===
        // Switch to manual control where user directly controls robot movement
        if (manualBtn) {
            manualBtn.addEventListener("click", () => {
                this.sendCommand("manual");
            });
        }

        // === EMERGENCY STOP BUTTON ===
        // Immediately stop all robot operations for safety
        if (emergencyBtn) {
            emergencyBtn.addEventListener("click", () => {
                this.sendCommand("emergency");
            });
        }

        // === SETUP ADDITIONAL CONTROLS ===
        // Initialize actuator controls (motors, servos, etc.)
        this.setupActuatorControls();

        // === KEYBOARD CONTROL SUPPORT ===
        // Enable keyboard shortcuts for accessibility and faster control
        document.addEventListener("keydown", (e) => this.handleKeyboard(e));
        document.addEventListener("keyup", (e) => this.handleKeyboardUp(e));

        // === PREVENT ACCIDENTAL PAGE REFRESH ===
        // Warn user if they try to leave the page while connected to robot
        window.addEventListener("beforeunload", (e) => {
            if (this.isConnected) {
                e.preventDefault();
                e.returnValue = "";
            }
        });

        // === CAMERA CONTROL SETUP ===
        // Initialize camera-related controls and functionality
        this.setupCameraControls();
    }

    /**
     * ACTUATOR CONTROLS SETUP
     * =======================
     *
     * Sets up controls for direct hardware actuator manipulation including:
     * - Individual motor speed controls
     * - Servo position controls
     * - Auto-return to neutral positions
     */
    setupActuatorControls() {
        // === LOCATE ACTUATOR CONTROL ELEMENTS ===
        // Find motor and servo control sliders in the DOM
        const leftMotorSlider = document.getElementById("left-motor-speed");
        const rightMotorSlider = document.getElementById("right-motor-speed");
        const servoPanSlider = document.getElementById("servo-pan");
        const servoTiltSlider = document.getElementById("servo-tilt");

        // === LEFT MOTOR CONTROL ===
        // Independent control of left wheel/motor
        if (leftMotorSlider) {
            // === REAL-TIME MOTOR SPEED ADJUSTMENT ===
            leftMotorSlider.addEventListener("input", (e) => {
                const value = e.target.value;

                // Update the displayed value for user feedback
                const valueElement =
                    document.getElementById("left-motor-value");
                if (valueElement) valueElement.textContent = value;

                // Send command to hardware immediately
                this.sendActuatorCommand("motor", "left", value);
            });

            // === AUTO-STOP ON RELEASE ===
            // Return motor to neutral when user releases slider
            leftMotorSlider.addEventListener("mouseup", () => {
                setTimeout(() => {
                    // Only auto-stop in manual mode to prevent interference with autonomous operation
                    if (this.currentMode === "manual") {
                        leftMotorSlider.value = 0;
                        const valueElement =
                            document.getElementById("left-motor-value");
                        if (valueElement) valueElement.textContent = "0";

                        // Send stop command to hardware
                        this.sendActuatorCommand("motor", "left", 0);
                    }
                }, 100); // Small delay to ensure smooth operation
            });
        }

        // === RIGHT MOTOR CONTROL ===
        // Independent control of right wheel/motor (mirror of left motor setup)
        if (rightMotorSlider) {
            // === REAL-TIME MOTOR SPEED ADJUSTMENT ===
            rightMotorSlider.addEventListener("input", (e) => {
                const value = e.target.value;

                // Update displayed value
                const valueElement =
                    document.getElementById("right-motor-value");
                if (valueElement) valueElement.textContent = value;

                // Send command to hardware
                this.sendActuatorCommand("motor", "right", value);
            });

            // === AUTO-STOP ON RELEASE ===
            rightMotorSlider.addEventListener("mouseup", () => {
                setTimeout(() => {
                    if (this.currentMode === "manual") {
                        rightMotorSlider.value = 0;
                        const valueElement =
                            document.getElementById("right-motor-value");
                        if (valueElement) valueElement.textContent = "0";

                        // Send stop command to hardware
                        this.sendActuatorCommand("motor", "right", 0);
                    }
                }, 100); // Small delay to ensure smooth operation
            });
        }

        // === SERVO CONTROL SETUP ===
        // Servo motors for camera positioning or robotic arm control

        // === PAN SERVO CONTROL ===
        // Controls horizontal camera/sensor movement
        if (servoPanSlider) {
            servoPanSlider.addEventListener("input", (e) => {
                const value = e.target.value;

                // Update displayed angle value
                const valueElement = document.getElementById("servo-pan-value");
                if (valueElement) valueElement.textContent = value;

                // Send position command to servo
                this.sendActuatorCommand("servo", "pan", value);
            });
        }

        // === TILT SERVO CONTROL ===
        // Controls vertical camera/sensor movement
        if (servoTiltSlider) {
            servoTiltSlider.addEventListener("input", (e) => {
                const value = e.target.value;

                // Update displayed angle value
                const valueElement =
                    document.getElementById("servo-tilt-value");
                if (valueElement) valueElement.textContent = value;

                // Send position command to servo
                this.sendActuatorCommand("servo", "tilt", value);
            });
        }
    }

    /**
     * CAMERA CONTROLS SETUP
     * =====================
     *
     * Initializes camera-related functionality including:
     * - Fullscreen view toggle
     * - Recording start/stop
     * - Snapshot capture
     */
    setupCameraControls() {
        // === LOCATE CAMERA CONTROL BUTTONS ===
        const cameraFullscreen = document.getElementById("camera-fullscreen");
        const cameraRecord = document.getElementById("camera-record");
        const cameraSnapshot = document.getElementById("camera-snapshot");

        // === FULLSCREEN TOGGLE ===
        // Allow user to view camera feed in fullscreen mode
        if (cameraFullscreen) {
            cameraFullscreen.addEventListener("click", () => {
                this.toggleCameraFullscreen();
            });
        }

        // === VIDEO RECORDING TOGGLE ===
        // Start/stop video recording functionality
        if (cameraRecord) {
            cameraRecord.addEventListener("click", () => {
                this.toggleCameraRecording();
            });
        }

        // === SNAPSHOT CAPTURE ===
        // Take still images from camera feed
        if (cameraSnapshot) {
            cameraSnapshot.addEventListener("click", () => {
                this.takeCameraSnapshot();
            });
        }
    }

    /**
     * LOGGING SYSTEM SETUP
     * ====================
     *
     * Configures the logging interface including:
     * - Log level filtering
     * - Log clearing functionality
     * - Log export capabilities
     */
    setupLogSystem() {
        // === LOG LEVEL FILTER ===
        // Dropdown to filter log entries by severity level
        const logLevelFilter = document.getElementById("log-level-filter");
        if (logLevelFilter) {
            logLevelFilter.addEventListener("change", () => {
                // Re-filter log display when selection changes
                this.filterLogEntries();
            });
        }

        // === CLEAR LOG BUTTON ===
        // Button to remove all log entries
        const clearLogBtn = document.getElementById("clear-log");
        if (clearLogBtn) {
            clearLogBtn.addEventListener("click", () => {
                this.clearLog();
            });
        }

        // === EXPORT LOG BUTTON ===
        // Button to download log entries as a text file
        const exportLogBtn = document.getElementById("export-log");
        if (exportLogBtn) {
            exportLogBtn.addEventListener("click", () => {
                this.exportLog();
            });
        }
    }

    /**
     * NOTIFICATION SYSTEM SETUP
     * =========================
     *
     * Initializes the user notification system including:
     * - Toast notifications for quick feedback
     * - Modal alerts for important messages
     * - Proper event handling for dismissal
     */
    setupNotificationSystem() {
        // === CREATE TOAST CONTAINER ===
        // Container for temporary notification messages
        if (!document.getElementById("toast-container")) {
            const toastContainer = document.createElement("div");
            toastContainer.id = "toast-container";
            toastContainer.className = "toast-container";
            document.body.appendChild(toastContainer);
        }

        // === ALERT MODAL SETUP ===
        // Setup close functionality for important alert dialogs
        const alertClose = document.getElementById("alert-close");
        const alertOk = document.getElementById("alert-ok");
        const alertModal = document.getElementById("alert-modal");

        // === CLOSE BUTTON HANDLER ===
        if (alertClose && alertModal) {
            alertClose.addEventListener("click", () => {
                alertModal.classList.add("hidden");
            });
        }

        // === OK BUTTON HANDLER ===
        if (alertOk && alertModal) {
            alertOk.addEventListener("click", () => {
                alertModal.classList.add("hidden");
            });
        }

        // === CLICK OUTSIDE TO CLOSE ===
        // Allow user to close modal by clicking background
        if (alertModal) {
            alertModal.addEventListener("click", (e) => {
                // Only close if clicking the modal background, not the content
                if (e.target === alertModal) {
                    alertModal.classList.add("hidden");
                }
            });
        }
    }

    /**
     * PERIODIC UPDATES SYSTEM
     * =======================
     *
     * Starts all background timers for real-time data updates.
     * Different update frequencies are used for different types of data
     * to balance responsiveness with performance.
     */
    startPeriodicUpdates() {
        // === STATUS UPDATES ===
        // Check robot connection and operational status every 3 seconds
        // Less frequent than sensor data to reduce server load
        this.updateInterval = setInterval(() => this.updateStatus(), 3000);

        // === SENSOR DATA UPDATES ===
        // Fetch sensor readings every 1 second for real-time feedback
        // More frequent for responsive robot operation
        this.sensorUpdateInterval = setInterval(
            () => this.updateSensorData(),
            1000
        );

        // === SYSTEM INFO UPDATES ===
        // Update system statistics every 2 seconds
        // Moderate frequency for uptime, temperature, etc.
        setInterval(() => this.updateSystemInfo(), 2000);
    }

    /**
     * UPDATE ROBOT STATUS
     * ==================
     *
     * Fetches current operational status from the robot including:
     * - Connection state
     * - Operating mode (manual/auto/emergency)
     * - Battery status
     * - System errors and warnings
     */
    async updateStatus() {
        try {
            // === FETCH STATUS FROM ROBOT ===
            const response = await fetch(`${this.baseUrl}/api/status`);

            if (response.ok) {
                const data = await response.json();

                // === UPDATE UI DISPLAYS ===
                this.updateStatusDisplay(data);

                // === CONNECTION IS HEALTHY ===
                this.updateConnectionStatus(true);

                // === LOG SIGNIFICANT CHANGES ===
                // Only log mode changes to avoid spam
                if (this.lastKnownMode !== data.mode) {
                    this.lastKnownMode = data.mode;
                }

                this.currentMode = data.mode || "unknown";
            } else {
                throw new Error(
                    `HTTP ${response.status}: ${response.statusText}`
                );
            }
        } catch (error) {
            console.error("Failed to update status:", error);
            this.updateConnectionStatus(false);
        }
    }

    /**
     * UPDATE SENSOR DATA FROM HARDWARE
     * ===============================
     *
     * Fetches latest sensor readings from the robotics controller
     * and updates the UI displays. Calculates data refresh rate
     * for performance monitoring.
     */
    async updateSensorData() {
        try {
            // === FETCH SENSOR DATA ===
            const response = await fetch(`${this.baseUrl}/api/sensors`);

            if (response.ok) {
                const data = await response.json();

                // Process and display the sensor data
                this.processSensorData(data);

                // Update connection status to show healthy communication
                this.updateConnectionStatus(true);

                // === CALCULATE DATA RATE ===
                // Monitor how often we're receiving sensor updates
                const now = Date.now();
                this.dataUpdateCount++;

                // Update rate display every second
                if (now - this.lastDataUpdate >= 1000) {
                    const rate = this.dataUpdateCount;
                    const dataRateElement =
                        document.getElementById("data-rate");
                    if (dataRateElement)
                        dataRateElement.textContent = `${rate} Hz`;

                    // Reset counters for next measurement period
                    this.dataUpdateCount = 0;
                    this.lastDataUpdate = now;
                }
            } else if (response.status !== 404) {
                // 404 errors are expected during startup - sensor endpoint may not be ready
                throw new Error(
                    `HTTP ${response.status}: ${response.statusText}`
                );
            }
        } catch (error) {
            // === ERROR HANDLING ===
            // Ignore 404 errors during startup - sensor endpoint may not be ready
            if (!error.message.includes("404")) {
                console.error("Failed to update sensor data:", error);
            }
        }
    }

    /**
     * PROCESS SENSOR DATA
     * ==================
     *
     * Takes raw sensor data from the robot and processes it for display.
     * Updates internal sensor state and triggers UI updates for each sensor type.
     *
     * @param {Object} data - Raw sensor data object from robot API
     */
    processSensorData(data) {
        // === TIMESTAMP ALL SENSOR READINGS ===
        const timestamp = Date.now();

        // === DISTANCE SENSOR PROCESSING ===
        // Ultrasonic, LiDAR, or other distance measuring sensors
        if (data.distance !== undefined) {
            this.sensorData.distance = {
                value: data.distance, // Distance value in cm or appropriate units
                valid: true, // Mark as valid reading
                timestamp: timestamp, // When this reading was taken
            };
            // Update the UI display with color coding based on distance
            this.updateDistanceDisplay(data.distance);
        }

        // === INERTIAL MEASUREMENT UNIT (IMU) PROCESSING ===
        // Accelerometer, gyroscope, and magnetometer data
        if (data.imu) {
            this.sensorData.imu = {
                // Accelerometer: measures linear acceleration (m/s² or g-force)
                accel: data.imu.accel || { x: 0, y: 0, z: 0 },
                // Gyroscope: measures angular velocity (rad/s or deg/s)
                gyro: data.imu.gyro || { x: 0, y: 0, z: 0 },
                // Magnetometer: measures magnetic field strength (for compass heading)
                mag: data.imu.mag || { x: 0, y: 0, z: 0 },
                valid: true,
                timestamp: timestamp,
            };
            this.updateIMUDisplay(this.sensorData.imu);
        }

        // === LINE FOLLOWING SENSORS ===
        // Array of light sensors for line detection and following
        if (data.line_sensors) {
            this.sensorData.line = {
                values: data.line_sensors, // Array of sensor values (0-1023 typically)
                valid: true,
                timestamp: timestamp,
            };
            this.updateLineSensorDisplay(data.line_sensors);
        }

        // === INFRARED OBSTACLE SENSORS ===
        // Binary sensors for detecting obstacles or objects
        if (data.ir_sensors) {
            this.sensorData.ir = {
                left: data.ir_sensors.left || false, // Left side obstacle detection
                right: data.ir_sensors.right || false, // Right side obstacle detection
                valid: true,
                timestamp: timestamp,
            };
            this.updateIRSensorDisplay(this.sensorData.ir);
        }

        // === GPS POSITIONING SYSTEM ===
        // Global positioning for outdoor navigation
        if (data.gps) {
            this.sensorData.gps = {
                latitude: data.gps.latitude || 0, // GPS latitude coordinate
                longitude: data.gps.longitude || 0, // GPS longitude coordinate
                altitude: data.gps.altitude || 0, // GPS altitude reading
                valid: true,
                timestamp: timestamp,
            };
            this.updateGPSDisplay(this.sensorData.gps);
        }

        // === PHYSICAL BUTTON/SWITCH INPUTS ===
        // Hardware buttons or switches on the robot
        if (data.button !== undefined) {
            this.sensorData.button = {
                pressed: data.button, // Boolean: is button currently pressed?
                valid: true,
                timestamp: timestamp,
            };
            this.updateButtonDisplay(data.button);
        }

        // === UPDATE SENSOR TIMESTAMP DISPLAY ===
        // Show when the last sensor update was received
        const timestampElement = document.getElementById("sensor-timestamp");
        if (timestampElement) {
            timestampElement.textContent = new Date(
                timestamp
            ).toLocaleTimeString();
        }

        // === LOG SIGNIFICANT SENSOR CHANGES ===
        // Only log important changes to avoid flooding the log
        this.logSensorChanges(data);
    }

    /**
     * UPDATE DISTANCE SENSOR DISPLAY
     * ==============================
     *
     * Updates the UI display for distance sensor readings with
     * color-coded feedback based on proximity thresholds.
     *
     * @param {number} distance - Distance reading in centimeters
     */
    updateDistanceDisplay(distance) {
        const element = document.getElementById("distance-value");
        if (element) {
            // === DISPLAY FORMATTED DISTANCE ===
            element.textContent = `${distance.toFixed(1)} cm`;

            // Color-code based on distance
            element.className = "sensor-value";
            if (distance < 10) {
                element.classList.add("danger");
            } else if (distance < 30) {
                element.classList.add("warning");
            } else {
                element.classList.add("success");
            }
        }
    }

    updateIMUDisplay(imu) {
        const accelElement = document.getElementById("accel-values");
        const gyroElement = document.getElementById("gyro-values");
        const magElement = document.getElementById("mag-values");

        if (accelElement) {
            accelElement.textContent = `${imu.accel.x.toFixed(
                2
            )}, ${imu.accel.y.toFixed(2)}, ${imu.accel.z.toFixed(2)}`;
        }
        if (gyroElement) {
            gyroElement.textContent = `${imu.gyro.x.toFixed(
                2
            )}, ${imu.gyro.y.toFixed(2)}, ${imu.gyro.z.toFixed(2)}`;
        }
        if (magElement) {
            magElement.textContent = `${imu.mag.x.toFixed(
                2
            )}, ${imu.mag.y.toFixed(2)}, ${imu.mag.z.toFixed(2)}`;
        }
    }

    updateLineSensorDisplay(values) {
        const sensors = document.querySelectorAll(".line-sensor");
        values.forEach((value, index) => {
            if (sensors[index]) {
                const sensor = sensors[index];
                sensor.style.opacity = value / 1023; // Assuming 10-bit ADC
                sensor.setAttribute("data-value", value);

                // Color-code based on intensity
                if (value > 800) {
                    sensor.className = "line-sensor high-intensity";
                } else if (value > 400) {
                    sensor.className = "line-sensor medium-intensity";
                } else {
                    sensor.className = "line-sensor low-intensity";
                }
            }
        });
    }

    updateIRSensorDisplay(ir) {
        const leftElement = document.getElementById("ir-left");
        const rightElement = document.getElementById("ir-right");

        if (leftElement) {
            leftElement.textContent = ir.left ? "DETECTED" : "CLEAR";
            leftElement.className = `ir-indicator ${
                ir.left ? "detected" : "clear"
            }`;
        }

        if (rightElement) {
            rightElement.textContent = ir.right ? "DETECTED" : "CLEAR";
            rightElement.className = `ir-indicator ${
                ir.right ? "detected" : "clear"
            }`;
        }
    }

    updateGPSDisplay(gps) {
        const latElement = document.getElementById("gps-lat");
        const lonElement = document.getElementById("gps-lon");
        const altElement = document.getElementById("gps-alt");

        if (latElement) latElement.textContent = gps.latitude.toFixed(6);
        if (lonElement) lonElement.textContent = gps.longitude.toFixed(6);
        if (altElement) altElement.textContent = gps.altitude.toFixed(1);
    }

    updateButtonDisplay(pressed) {
        const element = document.getElementById("button-status");
        if (element) {
            element.textContent = pressed ? "PRESSED" : "NOT PRESSED";
            element.className = `sensor-value button-indicator ${
                pressed ? "pressed" : "not-pressed"
            }`;
        }
    }

    logSensorChanges(newData) {
        // Log only critical sensor changes
        if (this.lastSensorData) {
            // Button state changes
            if (
                newData.button !== undefined &&
                this.lastSensorData.button !== newData.button
            ) {
                this.addLog(
                    `Button ${newData.button ? "pressed" : "released"}`,
                    "info"
                );
            }
        }

        this.lastSensorData = JSON.parse(JSON.stringify(newData));
    }

    updateStatusDisplay(data) {
        // Update mode display
        const modeElement = document.getElementById("mode-display");
        if (modeElement) {
            modeElement.textContent = (data.mode || "UNKNOWN").toUpperCase();
            modeElement.className = `mode-display mode-${
                data.mode || "unknown"
            }`;
        }

        // Update system status
        const systemStatusElement = document.getElementById("system-status");
        if (systemStatusElement) {
            systemStatusElement.textContent = data.status || "Unknown";
        }

        // Update battery info if available
        if (data.battery) {
            this.systemState.battery = data.battery;
            this.checkBatteryLevel(data.battery.voltage);
        }

        // Update errors and warnings
        if (data.errors) {
            this.systemState.errors = data.errors;
            this.displaySystemAlerts();
        }
    }

    updateConnectionStatus(connected = null) {
        if (connected !== null) {
            this.isConnected = connected;
        }

        const indicator = document.getElementById("connection-indicator");
        const statusText = document.getElementById("status-text");

        if (indicator && statusText) {
            if (this.isConnected) {
                indicator.className = "status-indicator connected";
                statusText.textContent = "Connected";
                indicator.style.color = "var(--success-color)";
            } else {
                indicator.className = "status-indicator disconnected";
                statusText.textContent = "Disconnected";
                indicator.style.color = "var(--danger-color)";
            }
        }
    }

    updateSystemInfo() {
        // Update uptime
        const uptimeElement = document.getElementById("system-uptime");
        if (uptimeElement) {
            const uptime = Math.floor((Date.now() - this.startTime) / 1000);
            const hours = Math.floor(uptime / 3600);
            const minutes = Math.floor((uptime % 3600) / 60);
            const seconds = uptime % 60;
            uptimeElement.textContent = `${hours
                .toString()
                .padStart(2, "0")}:${minutes
                .toString()
                .padStart(2, "0")}:${seconds.toString().padStart(2, "0")}`;
        }
    }

    /**
     * SEND COMMAND TO ROBOTICS HARDWARE
     * ================================
     *
     * Sends movement and control commands to the robotics controller
     * via HTTP API. Handles error conditions and provides user feedback.
     *
     * @param {string} command - The command to send (forward, backward, left, right, stop, etc.)
     */
    async sendCommand(command) {
        try {
            // === PREPARE REQUEST PAYLOAD ===
            let requestBody = { command: command };

            // Add current speed setting for movement commands
            if (["forward", "backward", "left", "right"].includes(command)) {
                requestBody.speed = this.currentSpeed;
            }

            // === SEND HTTP REQUEST ===
            const response = await fetch(`${this.baseUrl}/api/command`, {
                method: "POST",
                headers: {
                    "Content-Type": "application/json",
                },
                body: JSON.stringify(requestBody),
            });

            // === HANDLE SUCCESSFUL RESPONSE ===
            if (response.ok) {
                // Create user-friendly feedback message
                const speedInfo = requestBody.speed
                    ? ` (${requestBody.speed}%)`
                    : "";

                // Show success notification to user
                this.showNotification(
                    `Command executed: ${command}${speedInfo}`,
                    "success"
                );

                // Update UI to reflect current status
                this.updateStatus();
            } else {
                // HTTP error - throw for catch block to handle
                throw new Error(
                    `HTTP ${response.status}: ${response.statusText}`
                );
            }
        } catch (error) {
            // === ERROR HANDLING ===
            console.error("Failed to send command:", error);

            // Show error notification to user
            this.showNotification(
                `Failed to send command: ${command}`,
                "error"
            );

            // Update connection status to show disconnected state
            this.updateConnectionStatus(false);

            // Auto-retry connection check after a brief delay
            setTimeout(() => {
                this.updateStatus();
            }, 2000);
        }
    }

    async sendActuatorCommand(type, actuator, value) {
        try {
            const command = {
                type: "actuator",
                actuator_type: type,
                actuator_id: actuator,
                value: parseFloat(value),
            };

            const response = await fetch(`${this.baseUrl}/api/actuator`, {
                method: "POST",
                headers: {
                    "Content-Type": "application/json",
                },
                body: JSON.stringify(command),
            });

            if (!response.ok) {
                throw new Error(
                    `HTTP ${response.status}: ${response.statusText}`
                );
            }
        } catch (error) {
            console.error("Failed to send actuator command:", error);
            this.updateConnectionStatus(false);
        }
    }

    /**
     * KEYBOARD INPUT HANDLING
     * =======================
     *
     * Handles keyboard shortcuts for robot control, providing accessibility
     * and faster operation for experienced users. Supports both arrow keys
     * and WASD movement schemes.
     *
     * @param {KeyboardEvent} event - The keyboard event object
     */
    handleKeyboard(event) {
        // === PREVENT DEFAULT BROWSER ACTIONS ===
        // Stop browser from scrolling page or other default key behaviors
        const controlKeys = [
            "ArrowUp", // Prevent page scroll
            "ArrowDown", // Prevent page scroll
            "ArrowLeft", // Prevent page navigation
            "ArrowRight", // Prevent page navigation
            "Space", // Prevent page scroll
        ];
        if (controlKeys.includes(event.code)) {
            event.preventDefault();
        }

        // === IGNORE KEYS WHEN TYPING ===
        // Don't process control keys if user is typing in input fields
        if (
            event.target.tagName === "INPUT" ||
            event.target.tagName === "TEXTAREA"
        ) {
            return;
        }

        // === COMMAND MAPPING VARIABLES ===
        let command = null; // Robot command to send
        let buttonId = null; // UI button to highlight

        // === KEY TO COMMAND MAPPING ===
        switch (event.code) {
            // === FORWARD MOVEMENT ===
            case "ArrowUp": // Arrow key scheme
            case "KeyW": // WASD scheme
                command = "forward";
                buttonId = "btn-forward";
                break;

            // === BACKWARD MOVEMENT ===
            case "ArrowDown": // Arrow key scheme
            case "KeyS": // WASD scheme
                command = "backward";
                buttonId = "btn-backward";
                break;

            // === LEFT TURN/MOVEMENT ===
            case "ArrowLeft": // Arrow key scheme
            case "KeyA": // WASD scheme
                command = "left";
                buttonId = "btn-left";
                break;

            // === RIGHT TURN/MOVEMENT ===
            case "ArrowRight": // Arrow key scheme
            case "KeyD": // WASD scheme
                command = "right";
                buttonId = "btn-right";
                break;

            // === EMERGENCY STOP ===
            case "Space": // Spacebar for immediate stop
                command = "stop";
                buttonId = "btn-stop";
                break;

            // === MODE CONTROLS ===
            case "KeyE": // E for Emergency mode
                command = "emergency";
                buttonId = "btn-emergency";
                break;
            case "KeyM": // M for Manual mode
                command = "manual";
                buttonId = "btn-manual";
                break;
            case "KeyB": // B for auto (Because A is used for left movement)
                command = "auto";
                buttonId = "btn-autonomous";
                break;
        }

        // === EXECUTE COMMAND AND UPDATE UI ===
        if (command && buttonId) {
            // Send the command to the robot
            this.sendCommand(command);
            // Add visual feedback to show key is pressed
            this.addPressedState(buttonId, command);
        }
    }

    /**
     * KEYBOARD RELEASE HANDLING
     * =========================
     *
     * Handles key release events to remove visual feedback when
     * user stops pressing control keys.
     *
     * @param {KeyboardEvent} event - The keyboard event object
     */
    handleKeyboardUp(event) {
        // === KEY TO BUTTON MAPPING FOR RELEASES ===
        // Map keyboard keys to their corresponding UI button IDs
        const keyButtonMap = {
            ArrowUp: "btn-forward", // Arrow up -> forward button
            KeyW: "btn-forward", // W key -> forward button
            ArrowDown: "btn-backward", // Arrow down -> backward button
            KeyS: "btn-backward", // S key -> backward button
            ArrowLeft: "btn-left", // Arrow left -> left button
            KeyA: "btn-left", // A key -> left button
            ArrowRight: "btn-right", // Arrow right -> right button
            KeyD: "btn-right", // D key -> right button
            Space: "btn-stop", // Spacebar -> stop button
        };

        // === REMOVE VISUAL FEEDBACK ===
        const buttonId = keyButtonMap[event.code];
        if (buttonId) {
            this.removePressedState(buttonId);
        }
    }

    /**
     * ADD PRESSED STATE TO UI BUTTON
     * ==============================
     *
     * Adds visual feedback to show that a button is currently pressed,
     * either by mouse click or keyboard shortcut.
     *
     * @param {string} buttonId - The ID of the button element
     * @param {string} command - The command associated with the button
     */
    addPressedState(buttonId, command) {
        const button = document.getElementById(buttonId);
        if (button) {
            // === ADD PRESSED CLASS FOR STYLING ===
            button.classList.add("pressed");

            // === ADD DIRECTIONAL STYLING ===
            // Add active class to movement controls container for additional visual feedback
            if (command && command !== "stop") {
                const controls = document.querySelector(".movement-controls");
                if (controls) controls.classList.add(`active-${command}`);
            }
        }
    }

    /**
     * REMOVE PRESSED STATE FROM UI BUTTON
     * ===================================
     *
     * Removes visual feedback when a button is no longer pressed.
     * Cleans up all movement-related styling classes.
     *
     * @param {string} buttonId - The ID of the button element
     */
    removePressedState(buttonId) {
        const button = document.getElementById(buttonId);
        if (button) {
            // === REMOVE PRESSED STYLING ===
            button.classList.remove("pressed");

            // === CLEAN UP MOVEMENT CLASSES ===
            // Remove all directional styling from movement controls
            const movementControls =
                document.querySelector(".movement-controls");
            if (movementControls) {
                movementControls.classList.remove(
                    "active-forward",
                    "active-backward",
                    "active-left",
                    "active-right"
                );
            }
        }
    }

    /**
     * CAMERA CONTROL METHODS
     * ======================
     *
     * These methods handle camera-related functionality. Currently
     * they are placeholder implementations that can be extended
     * based on the specific camera system being used.
     */

    /**
     * TOGGLE CAMERA FULLSCREEN
     * ========================
     *
     * Switches the camera view between normal and fullscreen modes.
     * Implementation depends on the camera streaming technology used.
     */
    toggleCameraFullscreen() {
        // === PLACEHOLDER IMPLEMENTATION ===
        // TODO: Implement fullscreen camera functionality
        // This could involve:
        // - Expanding camera element to fullscreen
        // - Using Fullscreen API
        // - Sending fullscreen command to camera server
        console.log("Camera fullscreen toggle requested");
    }

    /**
     * TOGGLE CAMERA RECORDING
     * ======================
     *
     * Starts or stops video recording from the camera feed.
     * Implementation depends on the camera system capabilities.
     */
    toggleCameraRecording() {
        // === PLACEHOLDER IMPLEMENTATION ===
        // TODO: Implement camera recording functionality
        // This could involve:
        // - Sending start/stop recording commands to camera server
        // - Managing local video recording
        // - Updating UI to show recording status
        console.log("Camera recording toggle requested");
    }

    /**
     * TAKE CAMERA SNAPSHOT
     * ===================
     *
     * Captures a still image from the current camera feed.
     * Implementation depends on the camera streaming setup.
     */
    takeCameraSnapshot() {
        // Implementation for camera snapshot
    }

    /**
     * LOGGING SYSTEM METHODS
     * =====================
     *
     * Comprehensive logging system for debugging, monitoring, and user feedback.
     * Provides different log levels and both console and UI display.
     */

    /**
     * ADD LOG ENTRY
     * =============
     *
     * Creates a new log entry with timestamp and level classification.
     * Manages log storage limit and displays in both console and UI.
     *
     * @param {string} message - The log message to record
     * @param {string} level - Log level: "info", "success", "warning", "error", "debug"
     */
    addLog(message, level = "info") {
        // === CREATE LOG ENTRY ===
        const timestamp = new Date().toLocaleTimeString();
        const logEntry = {
            timestamp: timestamp,
            message: message,
            level: level,
            id: Date.now() + Math.random(), // Unique ID for each entry
        };

        // === STORE LOG ENTRY ===
        // Add to beginning of array for reverse chronological order
        this.logEntries.unshift(logEntry);

        // === MEMORY MANAGEMENT ===
        // Prevent unlimited log growth that could cause performance issues
        if (this.logEntries.length > this.maxLogEntries) {
            this.logEntries = this.logEntries.slice(0, this.maxLogEntries);
        }

        // === DISPLAY LOG ENTRY ===
        this.displayLogEntry(logEntry);

        // === CONSOLE LOGGING ===
        // Also log to browser console with appropriate level
        const consoleMethod =
            level === "error" ? "error" : level === "warning" ? "warn" : "log";
        console[consoleMethod](`[${timestamp}] ${message}`);
    }

    /**
     * DISPLAY LOG ENTRY IN UI
     * ======================
     *
     * Creates DOM elements to display log entries in the UI with
     * proper styling based on log level.
     *
     * @param {Object} entry - Log entry object with timestamp, message, level
     */
    displayLogEntry(entry) {
        const logContainer = document.getElementById("log-entries");
        if (!logContainer) return;

        // === CREATE LOG ELEMENT ===
        const logElement = document.createElement("div");
        logElement.className = `log-entry log-${entry.level}`;

        // Build HTML structure with timestamp, level, and message
        logElement.innerHTML = `
            <span class="log-timestamp">${entry.timestamp}</span>
            <span class="log-level">${entry.level.toUpperCase()}</span>
            <span class="log-message">${entry.message}</span>
        `;

        // === ADD TO DOM ===
        // Insert at beginning for reverse chronological order (newest first)
        logContainer.insertBefore(logElement, logContainer.firstChild);

        // === DOM CLEANUP ===
        // Prevent excessive DOM elements that could slow the interface
        while (logContainer.children.length > 50) {
            logContainer.removeChild(logContainer.lastChild);
        }

        // Apply current filter
        this.filterLogEntries();
    }

    filterLogEntries() {
        const filter =
            document.getElementById("log-level-filter")?.value || "all";
        const logEntries = document.querySelectorAll(".log-entry");

        logEntries.forEach((entry) => {
            const level = entry.classList.contains("log-error")
                ? "error"
                : entry.classList.contains("log-warning")
                ? "warning"
                : entry.classList.contains("log-success")
                ? "success"
                : entry.classList.contains("log-debug")
                ? "debug"
                : "info";

            let show = false;
            switch (filter) {
                case "all":
                    show = true;
                    break;
                case "error":
                    show = level === "error";
                    break;
                case "warning":
                    show = level === "error" || level === "warning";
                    break;
                case "info":
                    show = level !== "debug";
                    break;
                case "debug":
                    show = level === "debug";
                    break;
            }

            entry.style.display = show ? "flex" : "none";
        });
    }

    clearLog() {
        this.logEntries = [];
        const logContainer = document.getElementById("log-entries");
        if (logContainer) {
            logContainer.innerHTML = "";
        }
    }

    exportLog() {
        const logData = this.logEntries
            .map(
                (entry) =>
                    `[${entry.timestamp}] ${entry.level.toUpperCase()}: ${
                        entry.message
                    }`
            )
            .join("\n");

        const blob = new Blob([logData], { type: "text/plain" });
        const url = URL.createObjectURL(blob);
        const a = document.createElement("a");
        a.href = url;
        a.download = `robotics-controller-log-${new Date()
            .toISOString()
            .slice(0, 19)
            .replace(/:/g, "-")}.txt`;
        document.body.appendChild(a);
        a.click();
        document.body.removeChild(a);
        URL.revokeObjectURL(url);
    }

    /**
     * NOTIFICATION SYSTEM
     * ==================
     *
     * Displays temporary toast notifications to provide user feedback
     * for actions, errors, and system status changes.
     *
     * @param {string} message - The message to display
     * @param {string} type - Notification type: "info", "success", "warning", "error"
     * @param {number} duration - How long to show notification (ms), 0 = permanent
     */
    showNotification(message, type = "info", duration = 3000) {
        // === FIND NOTIFICATION CONTAINER ===
        const container = document.getElementById("toast-container");
        if (!container) return;

        // === CREATE TOAST ELEMENT ===
        const toast = document.createElement("div");
        toast.className = `toast toast-${type}`;

        // Build toast HTML with message and close button
        toast.innerHTML = `
            <span class="toast-message">${message}</span>
            <button class="toast-close">&times;</button>
        `;

        // === CLOSE BUTTON FUNCTIONALITY ===
        toast.querySelector(".toast-close").addEventListener("click", () => {
            container.removeChild(toast);
        });

        // === ADD TOAST TO CONTAINER ===
        container.appendChild(toast);

        // === AUTO-DISMISS TIMER ===
        // Remove toast after specified duration (unless duration is 0)
        if (duration > 0) {
            setTimeout(() => {
                if (container.contains(toast)) {
                    container.removeChild(toast);
                }
            }, duration);
        }
    }

    /**
     * BATTERY LEVEL MONITORING
     * ========================
     *
     * Monitors battery voltage and displays warning when level is low.
     * Prevents repeated warnings for the same low battery condition.
     *
     * @param {number} voltage - Current battery voltage reading
     */
    checkBatteryLevel(voltage) {
        // === LOW BATTERY THRESHOLD CHECK ===
        // 3.3V is typically considered low for lithium-ion batteries
        if (voltage < 3.3 && !this.lowBatteryWarned) {
            // === SHOW PERSISTENT WARNING ===
            // Duration 0 means the notification stays until manually closed
            this.showNotification("Low battery warning!", "warning", 0);

            // === PREVENT REPEATED WARNINGS ===
            // Set flag to avoid showing the same warning repeatedly
            this.lowBatteryWarned = true;
        }
    }

    /**
     * SYSTEM ALERTS DISPLAY
     * =====================
     *
     * Displays system errors and warnings in the log system.
     * Called when status updates contain error or warning information.
     */
    displaySystemAlerts() {
        // === DISPLAY ERROR MESSAGES ===
        if (this.systemState.errors.length > 0) {
            // Log each error with high priority level
            this.systemState.errors.forEach((error) => {
                this.addLog(`System Error: ${error}`, "error");
            });
        }

        // === DISPLAY WARNING MESSAGES ===
        if (this.systemState.warnings.length > 0) {
            // Log each warning with medium priority level
            this.systemState.warnings.forEach((warning) => {
                this.addLog(`System Warning: ${warning}`, "warning");
            });
        }
    }
}

/**
 * AUTOMATIC INITIALIZATION
 * ========================
 *
 * Set up the robotics controller when the page is fully loaded.
 * This ensures all DOM elements are available before we try to
 * attach event listeners and update the interface.
 */
document.addEventListener("DOMContentLoaded", function () {
    // Create global instance accessible from anywhere
    window.robotController = new RoboticsController();
});

/**
 * GLOBAL UTILITY FUNCTIONS
 * ========================
 *
 * These functions are exposed globally to allow HTML onclick handlers
 * and other external scripts to interact with the robotics controller.
 * They provide a safe interface that checks for initialization.
 */

/**
 * SEND COMMAND (GLOBAL FUNCTION)
 * ==============================
 *
 * Global wrapper for sending commands. Used by HTML onclick handlers.
 * Provides safety checks to ensure the controller is initialized.
 *
 * @param {string} cmd - Command to send to the robotics hardware
 */
function sendCommand(cmd) {
    if (window.robotController) {
        window.robotController.sendCommand(cmd);
    } else {
        console.error("Robotics controller not initialized");
    }
}

/**
 * SWITCH OPERATING MODE (GLOBAL FUNCTION)
 * =======================================
 *
 * Changes between manual and autonomous operation modes.
 *
 * @param {string} mode - Either "manual" or "auto"
 */
function switchMode(mode) {
    if (window.robotController) {
        window.robotController.sendCommand(
            mode === "manual" ? "manual" : "auto"
        );
    }
}

/**
 * UPDATE STATUS (GLOBAL FUNCTION)
 * ===============================
 *
 * Forces an immediate status update from the hardware.
 * Useful for manual refresh operations.
 */
function updateStatus() {
    if (window.robotController) {
        window.robotController.updateStatus();
    }
}

/**
 * END OF ROBOTICS CONTROLLER JAVASCRIPT
 * ====================================
 *
 * SUMMARY OF IMPLEMENTATION:
 *
 * ARCHITECTURE OVERVIEW:
 * This JavaScript file implements a comprehensive web-based control interface
 * for robotics systems using modern ES6+ features and best practices.
 *
 * KEY DESIGN PATTERNS:
 *
 * 1. **Class-Based Architecture**: The main RoboticsController class encapsulates
 *    all functionality, providing clean separation of concerns and easy testing.
 *
 * 2. **Event-Driven Programming**: Extensive use of event listeners for user
 *    interactions, ensuring responsive and interactive user experience.
 *
 * 3. **Async/Await Pattern**: Modern asynchronous programming for HTTP requests,
 *    providing clean error handling and non-blocking operations.
 *
 * 4. **State Management**: Centralized state tracking for connection status,
 *    sensor data, system health, and user interface state.
 *
 * 5. **Error Handling**: Comprehensive try-catch blocks and user feedback
 *    for robust operation in real-world robotics environments.
 *
 * CORE FEATURES:
 *
 * - **Real-time Communication**: HTTP-based API communication with robotics hardware
 * - **Live Data Updates**: Periodic sensor data fetching and display updates
 * - **User Interface Management**: Dynamic DOM manipulation for status displays
 * - **Command Execution**: Safe command transmission with speed parameters
 * - **Logging System**: Multi-level logging for debugging and monitoring
 * - **Error Recovery**: Automatic retry mechanisms for connection issues
 * - **User Feedback**: Toast notifications and visual indicators
 * - **Keyboard Controls**: Full keyboard support for accessibility
 *
 * PERFORMANCE CONSIDERATIONS:
 *
 * - **Throttled Updates**: Configurable update intervals to balance responsiveness and performance
 * - **Memory Management**: Limited log storage and DOM cleanup to prevent memory leaks
 * - **Efficient DOM Updates**: Minimal DOM manipulation for smooth operation
 * - **Background Processing**: Non-blocking operations to maintain UI responsiveness
 *
 * EXTENSIBILITY:
 *
 * The modular design makes it easy to:
 * - Add new sensor types and displays
 * - Implement additional control modes
 * - Customize update frequencies
 * - Extend logging and notification systems
 * - Add new communication protocols
 *
 * BROWSER COMPATIBILITY:
 * Requires modern browsers supporting ES6+, Fetch API, and async/await.
 * Compatible with Chrome 55+, Firefox 52+, Safari 11+, Edge 79+.
 */

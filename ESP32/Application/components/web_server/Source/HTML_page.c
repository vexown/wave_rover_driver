/******************************************************************************
 * @file HTML_page.c
 * @brief Defines the HTML page content for the web server component.
 * 
 ******************************************************************************/

/*******************************************************************************/
/*                                 INCLUDES                                    */
/*******************************************************************************/
/*    Include headers required for the definitions/implementation in *this*    */
/* source file. This typically includes this module's own header("template.h") */
/*      and any headers needed for function bodies, static variables, etc.     */
/*******************************************************************************/

/*******************************************************************************/
/*                                  MACROS                                     */
/*******************************************************************************/

/*******************************************************************************/
/*                                DATA TYPES                                   */
/*******************************************************************************/

/*******************************************************************************/
/*                     GLOBAL FUNCTION DECLARATIONS                            */
/*******************************************************************************/
/*  for function defined in some other .c files, to be used here. Use extern.  */
/*******************************************************************************/

/*******************************************************************************/
/*                     GLOBAL VARIABLES DECLARATIONS                           */
/*******************************************************************************/
/*  for variables defined in some other .c files, to be used here. Use extern. */
/*******************************************************************************/

/*******************************************************************************/
/*                     GLOBAL VARIABLES DEFINITIONS                            */
/*******************************************************************************/
/*    for variables defined in this .c file, to be used in other .c files.     */
/*******************************************************************************/
// Basic HTML Page with Motor Control Buttons and OTA Button
const char *HTML_PAGE = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <title>Wave Rover Control</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
    :root {
        --primary: #00bcd4;
        --primary-dark: #008ba3;
        --danger: #f44336;
        --danger-dark: #d32f2f;
        --warning: #ff9800;
        --warning-dark: #f57c00;
        --bg-color: #121212;
        --panel-bg: #1e1e1e;
        --text-color: #e0e0e0;
        --glow: 0 0 10px rgba(0, 188, 212, 0.7);
    }
    
    * {
        box-sizing: border-box;
        margin: 0;
        padding: 0;
    }
    
    body {
        font-family: 'Segoe UI', 'Roboto', sans-serif;
        text-align: center;
        background-color: var(--bg-color);
        color: var(--text-color);
        padding: 20px;
        min-height: 100vh;
        display: flex;
        flex-direction: column;
    }
    
    .header {
        margin-bottom: 30px;
        position: relative;
    }
    
    h1 {
        font-weight: 700;
        letter-spacing: 2px;
        margin-bottom: 15px;
        font-size: 2.2rem;
        text-transform: uppercase;
        color: var(--primary);
        text-shadow: var(--glow);
    }
    
    .control-panel {
        background-color: var(--panel-bg);
        border-radius: 15px;
        padding: 20px;
        margin: 0 auto 30px;
        max-width: 500px;
        box-shadow: 0 10px 20px rgba(0, 0, 0, 0.5);
        border: 1px solid rgba(0, 188, 212, 0.3);
    }
    
    .grid-container {
        display: grid;
        grid-template-columns: repeat(3, 1fr);
        grid-gap: 15px;
        margin-bottom: 20px;
    }
    
    .btn {
        display: inline-block;
        padding: 15px 10px;
        font-size: 16px;
        font-weight: bold;
        cursor: pointer;
        text-align: center;
        text-decoration: none;
        outline: none;
        color: white;
        background-color: var(--primary);
        border: none;
        border-radius: 10px;
        box-shadow: 0 5px var(--primary-dark);
        transition: all 0.2s ease;
        text-transform: uppercase;
        letter-spacing: 1px;
        /* Prevent text selection on rapid clicks */
        user-select: none;
        -webkit-user-select: none;
        -moz-user-select: none;
        -ms-user-select: none;
    }
    
    .btn:hover {
        box-shadow: 0 5px var(--primary-dark), 0 0 15px rgba(0, 188, 212, 0.5);
    }
    
    .btn:active {
        background-color: var(--primary-dark);
        box-shadow: 0 2px var(--primary-dark);
        transform: translateY(3px);
    }
    
    .btn.stop-btn { /* Renamed from .stop for clarity */
        background-color: var(--danger);
        box-shadow: 0 5px var(--danger-dark);
    }
    
    .btn.stop-btn:hover {
        box-shadow: 0 5px var(--danger-dark), 0 0 15px rgba(244, 67, 54, 0.5);
    }
    
    .btn.stop-btn:active {
        background-color: var(--danger-dark);
        box-shadow: 0 2px var(--danger-dark);
    }
    
    .btn.ota {
        background-color: var(--warning);
        box-shadow: 0 5px var(--warning-dark);
        margin-top: 10px;
        padding: 12px 20px;
    }
    
    .btn.ota:hover {
        box-shadow: 0 5px var(--warning-dark), 0 0 15px rgba(255, 152, 0, 0.5);
    }
    
    .btn.ota:active {
        background-color: var(--warning-dark);
        box-shadow: 0 2px var(--warning-dark);
    }
    
    .icon {
        font-size: 24px;
        margin-bottom: 5px;
        pointer-events: none; /* Prevent icon interfering with button events */
    }
    
    #status {
        margin-top: 20px;
        padding: 15px;
        font-style: italic;
        color: #aaa;
        background-color: rgba(0, 0, 0, 0.3);
        border-radius: 8px;
        max-width: 500px;
        margin: 0 auto;
        word-wrap: break-word;
        position: relative;
        border-left: 3px solid var(--primary);
    }
    
    #status::before {
        content: "STATUS";
        position: absolute;
        top: -10px;
        left: 10px;
        background-color: var(--panel-bg);
        padding: 0 8px;
        font-size: 12px;
        color: var(--primary);
        letter-spacing: 1px;
    }
    
    .footer {
        margin-top: 30px;
        font-size: 12px;
        color: #666;
        margin-top: auto;
    }
    
    .battery-container {
        position: absolute;
        top: 0;
        right: 15px;
        display: flex;
        align-items: center;
        font-size: 14px;
    }
    
    .battery-level {
        width: 30px;
        height: 15px;
        border: 2px solid #555;
        border-radius: 3px;
        margin-left: 5px;
        position: relative;
    }
    
    .battery-level::after {
        content: "";
        position: absolute;
        top: 2px;
        bottom: 2px;
        left: 2px;
        width: 66%; /* Example level */
        background-color: var(--primary);
        border-radius: 1px;
    }
    
    /* Responsive design */
    @media (max-width: 500px) {
        .grid-container {
        grid-gap: 10px;
        }
        
        .btn {
        padding: 12px 8px;
        font-size: 14px;
        }
        
        h1 {
        font-size: 1.8rem;
        }
    }
    </style>
</head>

<body>
    <div class="header">
        <h1>Wave Rover Control</h1>
        <div class="battery-container">
        <span>Battery</span>
        <div class="battery-level"></div>
        </div>
    </div>
    
    <div class="control-panel">
        <div class="grid-container">
            <div class="grid-item"></div>
            <div class="grid-item">
                <button id="btn-forward" class="btn motor-btn" data-direction="forward" style="width: 70px; height: 70px;">
                    <div class="icon">↑</div>
                </button>
            </div>
            <div class="grid-item"></div>

            <div class="grid-item">
                <button id="btn-left" class="btn motor-btn" data-direction="left" style="width: 70px; height: 70px;">
                    <div class="icon">←</div>
                </button>
            </div>
            <div class="grid-item">
                    <button id="btn-stop" class="btn stop-btn" style="width: 70px; height: 70px;">
                    <div class="icon">■</div>
                </button>
            </div>
            <div class="grid-item">
                <button id="btn-right" class="btn motor-btn" data-direction="right" style="width: 70px; height: 70px;">
                    <div class="icon">→</div>
                </button>
            </div>

            <div class="grid-item"></div>
            <div class="grid-item">
                <button id="btn-backward" class="btn motor-btn" data-direction="backward" style="width: 70px; height: 70px;">
                    <div class="icon">↓</div>
                </button>
            </div>
            <div class="grid-item"></div>
        </div>
    
        <button id="btn-ota" class="btn ota">Update Firmware</button>
        <br>
        <button id="btn-reset" class="btn ota" style="margin-top: 10px;" onclick="resetDevice()">Soft Reset</button>

    </div>
    
    <div id="status">Initializing systems...</div>
    
    <div class="footer">Wave Rover Control Panel v2.0</div>

    <script>
        // --- Debounce Configuration ---
        const DEBOUNCE_PRESS_DELAY = 50;   // ms: Ignore presses faster than this
        const DEBOUNCE_RELEASE_DELAY = 150; // ms: Refractory period after release before new press allowed

        // --- State Variables ---
        let commandSequenceNumber = 0; // For tracking command sequence to prevent out-of-sequence command processing on receiving side
        let canSendCommand = true;
        let pressTimeout = null;
        let releaseTimeout = null;
        let isTouching = false; // Prevent firing mouse events after touch events

        // --- Core Command Function ---
        function sendCmd(dir) {
            commandSequenceNumber++; // Increment command sequence number for each command sent
            const currentSeq = commandSequenceNumber; // Capture current sequence number for logging

            // Send command to server including the sequence number
            console.log(`Sending Command: ${dir} (Seq: ${currentSeq})`); // Log with sequence
            fetch(`/control?dir=${dir}&seq=${currentSeq}`) 
                .then(response => response.text())
                .then(data => {
                    console.log(`Command sent: ${dir} (Seq: ${currentSeq}), Response: ${data}`);
                    // Optional: Update status immediately based on command sent
                    // document.getElementById('status').innerText = `Command: ${dir}`; 
                    // updateStatus(); // Update status after sending command (might be delayed)
                })
                .catch(error => {
                    console.error(`Error sending command ${dir} (Seq: ${currentSeq}):`, error);
                    document.getElementById('status').innerText = 'Communication error: Failed to send command';
                });
        }

        // --- Debounced Event Handlers ---
        function handlePress(direction) {
            clearTimeout(pressTimeout);   // Clear any pending press timeout
            clearTimeout(releaseTimeout); // Clear refractory period timeout

            if (canSendCommand) {
                sendCmd(direction);
                canSendCommand = false; // Prevent immediate re-trigger

                // Set a short timeout to ignore rapid bounces/re-presses
                pressTimeout = setTimeout(() => {
                    // This timeout doesn't make canSendCommand true, 
                    // it just ensures the flag isn't stuck false if release doesn't fire.
                    // We primarily rely on handleRelease to re-enable commands.
                }, DEBOUNCE_PRESS_DELAY); 
            } else {
                    console.log('Press Debounced'); // Log for debugging
            }
        }

        function handleRelease() {
            clearTimeout(pressTimeout);   // Clear any pending press timeout
            clearTimeout(releaseTimeout); // Clear previous refractory timeout

            sendCmd('stop'); // Always send stop immediately on release

            // Start refractory period: Set timeout to re-enable commands
            releaseTimeout = setTimeout(() => {
                canSendCommand = true;
                console.log('Ready for next command'); // Log for debugging
            }, DEBOUNCE_RELEASE_DELAY);
        }

        // --- OTA Function ---
        function startOta() {
            if (confirm('Start Firmware Update? The device will reboot.')) {
                document.getElementById('status').innerText = 'Initiating firmware update...';
                fetch('/ota')
                .then(response => response.text())
                .then(data => {
                    console.log('OTA response:', data);
                    document.getElementById('status').innerText = 'OTA update process started. Device will reboot shortly.';
                })
                .catch(error => {
                    console.error('Error starting OTA:', error);
                    document.getElementById('status').innerText = 'Failed to start OTA update.';
                });
            }
        }

        // --- Reset Function ---
        function resetDevice() {
            console.log("Sending reset command...");
            if (confirm("Are you sure you want to reset the device?")) {
                fetch('/reset')
                    .then(response => response.text())
                    .then(data => {
                        console.log(data);
                        document.body.innerHTML = '<h1>Device is resetting... Please wait. You may need to refresh the page manually.</h1>';
                    })
                    .catch(error => console.error('Error:', error));
            }
        }
            
        // --- Status Update Functions ---
        function updateStatus() {
            fetch('/print')
                .then(response => response.text())
                .then(data => {
                    const statusDiv = document.getElementById('status');
                    // Only update if the content has changed to avoid flickering
                    if (statusDiv.innerText !== data) {
                        statusDiv.innerText = data;
                    }
                })
                .catch(error => {
                    // Avoid logging errors too frequently if connection is lost
                    // console.error('Error fetching status:', error); 
                    const statusDiv = document.getElementById('status');
                        if (!statusDiv.innerText.startsWith('Error loading status') && !statusDiv.innerText.startsWith('Communication error')) {
                            statusDiv.innerText = 'Error loading status information';
                        }
                });
        }
        
        function startStatusUpdates() {
            updateStatus(); // Initial update
            setInterval(updateStatus, 5000); // Then update every 5 seconds
        }

        // --- Event Listener Setup ---
        window.addEventListener('DOMContentLoaded', (event) => {
            console.log('DOM fully loaded and parsed');
            
            // Add listeners to motor control buttons
            document.querySelectorAll('.motor-btn').forEach(button => {
                const direction = button.getAttribute('data-direction');
                
                // Mouse Events
                button.addEventListener('mousedown', (e) => {
                    if (isTouching) return; // Don't process mouse event if touch is active
                    console.log('mousedown:', direction);
                    handlePress(direction);
                });
                
                // Use mouseleave as a fallback if mouseup occurs outside button
                button.addEventListener('mouseup', (e) => {
                    if (isTouching) return; 
                    console.log('mouseup');
                    handleRelease();
                });
                button.addEventListener('mouseleave', (e) => {
                    if (isTouching) return; 
                    // Check if mouse button is still pressed when leaving
                    if (e.buttons === 1 || e.which === 1) { 
                            console.log('mouseleave while pressed');
                            handleRelease();
                    }
                });

                // Touch Events
                button.addEventListener('touchstart', (e) => {
                    e.preventDefault(); // Important to prevent mouse events
                    isTouching = true;
                    console.log('touchstart:', direction);
                    handlePress(direction);
                });

                button.addEventListener('touchend', (e) => {
                    e.preventDefault(); 
                    console.log('touchend');
                    handleRelease();
                    // Small delay before resetting isTouching flag
                    setTimeout(() => { isTouching = false; }, 100); 
                });
                    button.addEventListener('touchcancel', (e) => {
                    e.preventDefault(); 
                    console.log('touchcancel');
                    handleRelease();
                        setTimeout(() => { isTouching = false; }, 100); 
                });
            });

                // Add listener for the dedicated Stop button
            document.getElementById('btn-stop').addEventListener('click', (e) => {
                    console.log('Stop button clicked');
                    handleRelease(); // Treat click as a release event
            });


            // Add listener for OTA button
                document.getElementById('btn-ota').addEventListener('click', startOta);

            // Start status updates
            startStatusUpdates();
        });

    </script>
</body>
</html>
)rawliteral";
    
/*******************************************************************************/
/*                     STATIC FUNCTION DECLARATIONS                            */
/*******************************************************************************/

/*******************************************************************************/
/*                             STATIC VARIABLES                                */
/*******************************************************************************/

/*******************************************************************************/
/*                     GLOBAL FUNCTION DEFINITIONS                             */
/*******************************************************************************/

/*******************************************************************************/
/*                     STATIC FUNCTION DEFINITIONS                             */
/*******************************************************************************/

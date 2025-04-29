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
      justify-content: space-between;
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
    }
    
    .btn:hover {
      box-shadow: 0 5px var(--primary-dark), 0 0 15px rgba(0, 188, 212, 0.5);
    }
    
    .btn:active {
      background-color: var(--primary-dark);
      box-shadow: 0 2px var(--primary-dark);
      transform: translateY(3px);
    }
    
    .btn.stop {
      background-color: var(--danger);
      box-shadow: 0 5px var(--danger-dark);
    }
    
    .btn.stop:hover {
      box-shadow: 0 5px var(--danger-dark), 0 0 15px rgba(244, 67, 54, 0.5);
    }
    
    .btn.stop:active {
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
    }
    
    /* Battery indicator (for show - would need actual data) */
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
      width: 66%;
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
                <button class="btn" style="width: 70px; height: 70px;"
                                onmousedown="sendCmd('forward')"
                                onmouseup="sendCmd('stop')"
                                ontouchstart="sendCmd('forward')"
                                ontouchend="sendCmd('stop')">
                    <div class="icon">↑</div>
                </button>
            </div>
            <div class="grid-item"></div>

            <div class="grid-item">
                <button class="btn" style="width: 70px; height: 70px;"
                                onmousedown="sendCmd('left')"
                                onmouseup="sendCmd('stop')"
                                ontouchstart="sendCmd('left')"
                                ontouchend="sendCmd('stop')">
                    <div class="icon">←</div>
                </button>
            </div>
            <div class="grid-item">
                <button class="btn stop" style="width: 70px; height: 70px;" onclick="sendCmd('stop')">
                    <div class="icon">■</div>
                </button>
            </div>
            <div class="grid-item">
                <button class="btn" style="width: 70px; height: 70px;"
                                onmousedown="sendCmd('right')"
                                onmouseup="sendCmd('stop')"
                                ontouchstart="sendCmd('right')"
                                ontouchend="sendCmd('stop')">
                    <div class="icon">→</div>
                </button>
            </div>

            <div class="grid-item"></div>
            <div class="grid-item">
                <button class="btn" style="width: 70px; height: 70px;"
                                onmousedown="sendCmd('backward')"
                                onmouseup="sendCmd('stop')"
                                ontouchstart="sendCmd('backward')"
                                ontouchend="sendCmd('stop')">
                    <div class="icon">↓</div>
                </button>
            </div>
            <div class="grid-item"></div>
        </div>
    
        <button class="btn ota" onclick="startOta()">Update Firmware</button>
    </div>
  
    <div id="status">Initializing systems...</div>
  
    <div class="footer">Wave Rover Control Panel v2.0</div>

    <script>
        function sendCmd(dir) {
        // Send command to server
        fetch('/control?dir=' + dir)
            .then(response => response.text())
            .then(data => {
            console.log('Command sent:', dir, 'Response:', data);
            updateStatus(); // Update status after sending command
            })
            .catch(error => {
            console.error('Error sending command:', error);
            document.getElementById('status').innerText = 'Communication error: Failed to send command';
            });
        }
        
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
        
        // Function to fetch and update status
        function updateStatus() {
        fetch('/print')
            .then(response => response.text())
            .then(data => {
            document.getElementById('status').innerText = data;
            })
            .catch(error => {
            console.error('Error fetching status:', error);
            document.getElementById('status').innerText = 'Error loading status information';
            });
        }
        
        // Fetch status periodically
        function startStatusUpdates() {
        updateStatus(); // Initial update
        setInterval(updateStatus, 5000); // Then update every 5 seconds
        }
        
        // Start status updates when page loads
        window.onload = startStatusUpdates;

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

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
    <html>
    <head>
    <title>Wave Rover Control</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
    body { font-family: Arial, sans-serif; text-align: center; }
    .btn { display: inline-block; padding: 15px 25px; font-size: 24px; cursor: pointer;
           text-align: center; text-decoration: none; outline: none; color: #fff;
           background-color: #4CAF50; border: none; border-radius: 15px; box-shadow: 0 9px #999; margin: 10px; }
    .btn:active { background-color: #3e8e41; box-shadow: 0 5px #666; transform: translateY(4px); }
    .stop { background-color: #f44336; }
    .ota { background-color: #ff9800; } /* Style for OTA button */
    .grid-container { display: grid; grid-template-columns: auto auto auto; padding: 10px; justify-content: center; }
    .grid-item { padding: 20px; font-size: 30px; text-align: center; }
    </style>
    <script>
    function sendCmd(dir) {
      fetch('/control?dir=' + dir)
        .then(response => response.text())
        .then(data => console.log(data))
        .catch(error => console.error('Error:', error));
    }
    function startOta() {
      // Optionally display a confirmation message
      if (confirm('Start Firmware Update? The device will reboot.')) {
        fetch('/ota')
          .then(response => response.text())
          .then(data => {
             console.log(data);
             alert('OTA process started. Check device logs.'); // Inform user
          })
          .catch(error => {
             console.error('Error:', error);
             alert('Failed to start OTA.'); // Inform user of failure
          });
      }
    }
    </script>
    </head>
    <body>
    <h1>Wave Rover Control</h1>
    <div class="grid-container">
      <div class="grid-item"></div>
      <div class="grid-item"><button class="btn" onclick="sendCmd('forward')">Forward</button></div>
      <div class="grid-item"></div>
      <div class="grid-item"><button class="btn" onclick="sendCmd('left')">Left</button></div>
      <div class="grid-item"><button class="btn stop" onclick="sendCmd('stop')">Stop</button></div>
      <div class="grid-item"><button class="btn" onclick="sendCmd('right')">Right</button></div>
      <div class="grid-item"></div>
      <div class="grid-item"><button class="btn" onclick="sendCmd('backward')">Backward</button></div>
      <div class="grid-item"></div>
    </div>
    <hr>
    <div>
      <button class="btn ota" onclick="startOta()">Update Firmware</button>
    </div>
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

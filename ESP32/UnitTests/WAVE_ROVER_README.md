## Important note

This project includes a `unit-test-app` directory, which is a customized version of the unit test application provided with the Espressif ESP-IDF in their official repository https://github.com/espressif/esp-idf - all credit for development of this unit-test app goes to espressif's team. 

Here I'm simply using it for testing components in my own application. For License terms of this application see the ESP-IDF repo via the link above.

This directory is included in the repository to ensure that unit tests can be easily built and run for the WAVE ROVER Driver project.

### Building and Running the Unit Tests

Use the provided UnitTest_Build.sh and UnitTest_Execute.sh scripts.

### Updating the unit-test-app

If you have updated the esp-idf version in the project and the unit-test-app has a new version, you will need to update esp-idf version
in the Build script, remove all the unit-test-app files (all files except UnitTests_Build.sh, UnitTests_Execute.sh, WAVE_ROVER_README.md, .gitignore) and run the Build script. It will automatically pull the latests unit-test-app from the updated esp-idf version and set up the CMakeLists.txt to include the components from the Wave Rover Driver application.
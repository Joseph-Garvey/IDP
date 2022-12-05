# IDP Group M205
## Software Guide
### Key Files:
1. Robot_Main_V2 contains the code that was intended for the final competition.
2. 1st Competition contans code used during the 1st competition.
3. "Last Stand" contains the actual code used to manually drive the robot through the tunnel during the final competition.
### Other Files of Note:
1. PID Testing was a brief adventure into PID line following that was abandoned when it became apparent that our line sensors had failed for the 2nd time. It was intended to use PID with autotuning to increase the speed at which we could traverse the course. Our less sophisticated "time away from the line" alogrithm was limited by a tradeoff between speed and accuracy, as increasing the speed delta or decreasing the time constant would increase the robot's ability to turn corners but at the cost of an increased chance of losing the line and/or reduced speed due to oscillations along the line.
2. Tests Folder:
   1. Bluetooth LE Serial Emulator - it was planned to use a serial emulator over Bluetooth LE to monitor the robot sensors remotely, and send the "return home" command to the robot. During testing, it was determined that the processing and memory needed (as the library consumed a lot) would leave none left over for our program.
   2. IMU_ArduinoLib - during the line following testing with the cardboard robot, it was found that the robot could not get up the ramp. Whilst the production chassis was being produced, a quick script that would read the Arduino's inbuilt IMU was researched so that if needed, the robot could ramp both motors to 100% in an attempt to get over the ramp. The second benefit this would have had is that the tilt of the ramp could have been used as an error check for our line sensors. At the time, we did not know whether the line sensor/comparator circuit would reliably count junctions whilst the robot was moving. The IMU tilt could be used as a check, where if the count was not what would be expected at the time the ramp was crossed, the count could be corrected.
   3. Line Following was a script used to test the efficacy of different methods of line following, and tuning the line following parameters.
   4. Line Sensors was used to debug Line sensor output.
   5. Line counter was much the same but for junction detection.
   6. DC Motor Test was used to debug motors when they stopped working the week of competition.
   7. Optic Block Type was used to test optical block detection.
   8. Tunnel was used to test the robot tracking the walls of the tunnel.
3. Junction_Main was a last ditch attempt to use the junction sensors as our line followers, in an attempt to get points for looping around the circuit.
4. OpenCV - Initial tests using openCV.
   

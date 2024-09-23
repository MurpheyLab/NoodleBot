# How to Build a Swimmer
<img src="https://github.com/MurpheyLab/noodlebot/blob/main/swimmer/swimmer.png" height="500">

## Hardware
### Materials
1. Pool Noodle
2. Motors 
    - we used TowerPro SG-5010
3. Teensy
4. Adafruit HUZZAH ESP8266 Breakout Board
5. 3D printed motor brackets 
    - provided in the solidworks folder
    - may need to adjust dimensions depending on the desired size of your robot
6. hardware stack (order from outside to inside): 
    - TOP (brackets): M3 x 12mm screw / servo bracket (outer link) / servo bracket 2 (middle link) / M2 nut
    - BOTTOM (brackets to motor center): M2.5 x 16mm screw / servo bracket 2 (middle link) /  servo bracket (outer link) / motor horn / motor
    - BOTTOM (outer bracket to motor horn): M2 x 10mm screw / M2 washer / M3 washer (optional) / servo bracket (outer link) / wheel / M2 washer / M2 lock washer / M2 nut (repeat 2x to prevent motor slip)

### Assembly Steps
1. cut pool noodle into shorter segments to make links of the desired lengths
    - Don't forget to account for the length of the 3D printed hinges if you want the links to be the same length (middle link has 2 hinges, while front and back joints only have 1 hinge)
2. cut each link in half (through the diameter of each noodle)
3. with an additional pool noodle or leftover material, cut a small strip that is the same length as each link
4. sandwich the strip between the outer rings of the two halves of a link, and use an adhesive (like hot glue)
    - this will create an opening on one side for the electronics to be placed inside the robot
    - see reference the image for what is should look like

<img src="https://github.com/MurpheyLab/noodlebot/blob/main/swimmer/swimmer_link.JPG" width="500">

5. attach the 3D printed motor brackets as follows:
    - first and last link: cap at one end and "servo bracket" at the other end
    - middle link: "servo bracket 2" at both ends
6. connect motor horn onto each outer servo bracket (recommend 2 places to prevent slip)
7. attach servo to motor horn 
    - recommend using `roslaunch noodlebot joint_test_gui.launch` to center the motors before attaching to the brackets
8. insert the motor into the bracket and screw the brackets together
9. connect electronics (see below for more details)


<img src="https://github.com/MurpheyLab/noodlebot/blob/main/swimmer/swimmer_wiring_diagram.png" height="500">

## ESP8266 Firmware
The WiFi module needs to be flashed with **AT firmware**. The files for flashing can be found [here](https://github.com/espressif/ESP8266_NONOS_SDK). Follow [this tutorial](https://www.instructables.com/The-Simple-Guide-to-Flashing-Your-ESP8266-Firmware/), but there are some configuration settings that are different, which are noted in the steps below.

1. directly connect the ESP8266 breakout board to the computer with a USB to TTL Serial cable
2. settings for the nodeMCU flasher
    - flash size: 4MB
    - baud rate: 115200
    - flash speed: 40 MHz
    - QIO
3. mapping of bin files 
    - boot_v1.7.bin  → 0x00000
    - user1.2048.new.5.bin → 0x01000
    - esp_init_data_default_v08.bin → 0x1FC000
    - blank.bin → 0xFE000 and 0x1FE000
4. if the mapping in 3 doesn't work, try this one
    - boot_v1.7.bin  → 0x00000
    - user1.2048.new.5.bin → 0x01000
    - esp_init_data_default_v05.bin → 0x3FC000
    - blank.bin → 0x7E000 and 0x3FE000
5. After flashing is complete, open a serial monitor (like Arduino or Termite) to test that the chip is now programmed for AT commands
    - press reset
        - you should see some gibberish and the "ready"
    - if you type "AT", it should return "OK"
    - if you type "AT+GMR", it shoudl return firmware version and some other specs

Debugging tips
1. make sure you’re using the right baud rate
2. Check the wiring, especially TX and RX are correct 
3. make sure EN is pulled high and GPIO 0 is pulled low 
4. make sure the mapping is correct
5. check ESP github to see if there are any changes
6. may have to follow the similar pattern used in previous versions for 32Mbit if it isn’t explicitly written
7. press reset on the board before flashing
8. make sure there is common ground between all electronics components
9. if the blue light is flashing and/or AT commands are not working, use baud rate of 74880 to look at messages with a serial monitor
    - should provide more readable messages that can provide more insight 

## Arduino

#### One Time Setup 
1. Download Arduino IDE
2. Install the WiFiEspAT libray using IDE
3. Compile ROS package using instructions in main readme (`catkin_make`)
4. Navigate to library installation location (likely `$HOME/Arduino/libraries`) and build Arduino library with `rosrun rosserial_arduino make_libraries.py .`
5. Compile Arduino code using IDE
6. Upload code to Arduino using IDE

#### Arduino Code Upload
Once the setup steps are completed, you should only have to compile and upload the Arduino code.

Note: If you change any message in the ROS package or install any new message libraries, you'll have to delete the `ros_lib` Arduino package and re-run the make_libraries command

#### Possible Arduino Code Modifications 
- NoodleBot communicates with the learning laptop via wifi. To set up your wifi, modify the `ssid`, `pass`, and `server` parameters in `wifi.h`
    - `ssid` is the name of your wireless router
    - `pass` is the wifi password
    - `server()` includes the ip address of the learning laptop. In ubuntu, you can get the ip address with the command `ifconfig`
- If you changed the PWM pins of the servos on the Teensy, modify the Arduino code to indicate the servo pins on the teensy.
    - the Arduino code uses Serial1 pins on Teensy to connect it with the WiFi breakout board
- If you want to use April Tags instead of the Marvelmind IPS system, see details below.

## State Information 

We use the Marvelmind Indoor Positioning System (IPS) for robot state information. Four stationary beacons are placed around the test environment with line-of-sight to the robot. The two mobile beacons (or "hedgehogs") on the robot provide 2D state and rotation information via radio communication with a centralized modem. For these tests, we use firmware v7.900 for Super-Beacon-2 and Modem HW v5.1-2. The test area is roughly 3.4m by 1.8m in x and y respectively. With this configuration, the modem reports an update rate of 13.7Hz. We have included our modem configuration file (NoodleBot.ini) for reference. Alternate methods of collecting state information are possible such as April Tags, but the timing may vary and require adjustments to the robot firmware.

## Marvelmind Setup
Setup indoor positioning system (IPS) according to the Marvelmind instructions [https://marvelmind.com/download/](https://marvelmind.com/download/). We used a separate laptop to run the Marvelmind modem. 


## April Tags Setup

Setup USB Camera (use [http://wiki.ros.org/camera_calibration](http://wiki.ros.org/camera_calibration) to generate calibration file)

`$ cp $HOME/swimmer_ws/src/noodlebot/config/usb_cam1.yaml $HOME/.ros/camera_info/.`

To use april tags, you need to:
1. set `use_hedgehog=false` in `noodlebot/Arduino/MultiServoWithIntegrator/MultiServoWithIntegrator.ino`
2. set `april_tags=True` in `noodlebot/launch/train.launch`
3. set `april_tags=True` in `noodlebot/launch/enjoy.launch`

### April tag selection (for each test)
- If you want to change your april tag library, update `noodlebot/config/tags.yaml`
- Specify which tags you're using in tags.yaml (whatver names you put in that file will show up in rviz)
- You can generate april tags using the following site [https://novanix.github.io/apriltag-pdf-gen/](https://novanix.github.io/apriltag-pdf-gen/)


### To run camera interface 
`$ roslaunch noodlebot init_cam_n_track.launch`

Note: If you get an error, you may need to check to modify the param "video device" in this launch file. You can check which devices are plugged into your computer with `$ ls /dev/video*`
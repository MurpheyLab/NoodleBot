# How to Build a NoodleBot Swimmer

<img src="https://github.com/MurpheyLab/noodlebot/blob/main/swimmer/swimmer.png" height="500">

The left side of the above image shows a top-down view of the NoodleBot hardware. The total cost of the 3-link swimmer is approximately $100 excluding the cost of the Indoor Positioning System (IPS). The inset in the middle of the image shows the electronic components that are housed inside the middle joint. NoodleBot was designed to mimic the [MuJoCo swimmer](https://mgoulao.github.io/gym-docs/environments/mujoco/swimmer/). The left side of the image shows the differences in joint orientations and body frames between the original simulated swimmer (top) and our hardware swimmer (bottom). The original body frame is offset $0.5L$ from the front link, where L is the link length. Our body frame is centered on the middle link. All joints shown in original and updated diagrams are $45^\circ$.

Other differences between the original simulation and the hardware swimmer are summarized in the following table. 
| Property | Original Simulation | Hardware|  
| -- | -- | -- |  
| State Dimension | 10 | 11 [^1] | 
| Action Dimension |  2 | 2 | 
| Time Step |  $0.01s$ | $0.05s$ | 
| Joint Limits | $\pm 100^\circ$ [^3] | $\pm 80^\circ$ [^2] | 
| Joint Velocity Limits | None |  $\pm0.3 \text{deg}/\text{ms}$ [^2] |  
| Robot Angle Range | $\pm \infty$  | $\pm \pi$ |  % (angle wrapping) | 
| Reset Joint Angles | $\mathcal{U}(-0.1,0.1)$ | $\mathcal{U}(-80^\circ,80^\circ)$ | 
| Reset Velocities (all) | $\mathcal{U}(-0.1,0.1)$ | ${0}$ | 
| Link Properties | Equal length and mass | See [Details Table](#details-by-link) |
| Body Frame (see figure above) |  Front link $+0.5L$ [^3] | Center of middle link |  
| Workspace Constraints | None | Barriers in $x,y$  | 
| Contact Friction | None | Unknown  | 
| Medium density | 4000 | ${\sim}1.2$ (air)  | 
| Medium viscosity | 0.1 | ${\sim}0.00002$ (air)  | 

[^1]: The robot angle state representation $\theta$ is replaced with $[sin(\theta), cos(\theta)]$ to remove angle wrapping discontinuities. 
[^2]: Joint angles and joint velocities are constrained to protect the motors.  
[^3]: Simulated swimmer body frames and joint limits were updated to match our hardware frames.  

# Hardware
### Materials
1. Pool Noodle
2. Motors (2 X [TowerPro SG-5010](https://www.adafruit.com/product/155))
3. [Teensy 4.1](https://www.adafruit.com/product/4622)
4. [Adafruit HUZZAH ESP8266 Breakout Board](https://www.adafruit.com/product/2471)
5. 3D printed motor brackets 
    - See solidworks folder
    - Note: May need to adjust dimensions depending on the desired size of your robot
6. Hardware stack (order from outside to inside): 
    - TOP (brackets): M3 x 12mm screw / servo bracket (outer link) / servo bracket 2 (middle link) / M2 nut
    - BOTTOM (brackets to motor center): M2.5 x 16mm screw / servo bracket 2 (middle link) /  servo bracket (outer link) / motor horn / motor
    - BOTTOM (outer bracket to motor horn): M2 x 10mm screw / M2 washer / M3 washer (optional) / servo bracket (outer link) / wheel / M2 washer / M2 lock washer / M2 nut (repeat 2x to prevent motor slip)
7. Portable USB charger battery packs / power bank (2x, we use Charmast 10400mAh 3-Port Ultra-Slim LED Screen Power Bank)
8. Wiring (3x [USB-A to TTL Serial Cable](https://www.adafruit.com/product/954?gQT=1), 1x [USB-A to Micro-B Cable](htthttps://www.adafruit.com/product/592))
9. Optional: shielding to protect boards from contact with noodle housing (which can accumulate charge during operation). An ESD bag can provide shielding

### Assembly Steps
1. Cut pool noodle into shorter segments to make links of the desired lengths
    - Don't forget to account for the length of the 3D printed hinges if you want the links to be the same length (middle link has 2 hinges, while front and back joints only have 1 hinge)
2. Cut each link in half (through the diameter of each noodle)
3. With an additional pool noodle or leftover material, cut a small strip that is the same length as each link
4. Sandwich the strip between the outer rings of the two halves of a link, and use an adhesive (like hot glue)
    - This will create an opening on one side for the electronics to be placed inside the robot
    - See reference the image for what is should look like

<img src="https://github.com/MurpheyLab/noodlebot/blob/main/swimmer/swimmer_link.JPG" width="500">

5. Attach the 3D printed motor brackets as follows:
    - First and last link: cap at one end and "servo bracket" at the other end
    - Middle link: "servo bracket 2" at both ends
6. Connect motor horn onto each outer servo bracket (recommend 2 places to prevent slip)
7. Attach servo to motor horn 
    - We recommend using `roslaunch noodlebot joint_test_gui.launch` to center the motors before attaching to the brackets
8. Insert the motor into the bracket and screw the brackets together
9. Connect electronics (see below for more details)


<img src="https://github.com/MurpheyLab/noodlebot/blob/main/swimmer/swimmer_wiring_diagram.png" height="500">

### Details by Link
Our robot is designed to be customizable. Length and mass values in the following table are provided as baselines but could easily be adjusted as desired (up to the torque capacity of the motors).

| Link | Length | Mass | Components | 
| -- | -- | -- | -- | 
| Front | 38cm | 345g | Battery, FDM  End Cap, FDM Bracket, Pool-Noodle Housing
| Middle | 43cm | 495g | Servo Motors (2x), FDM Brackets (2x),  WiFi Module, Teensy Microcotroller, Wiring, IPS Beacons (2x), Pool-Noodle Housing |
| Back |  38cm | 360g | Battery, FDM End Cap, FDM Bracket, Cables, Pool-Noodle Housing |

# ESP8266 Firmware
The WiFi module needs to be flashed with **AT firmware**. The files for flashing can be found [here](https://github.com/espressif/ESP8266_NONOS_SDK). Follow [this tutorial](https://www.instructables.com/The-Simple-Guide-to-Flashing-Your-ESP8266-Firmware/), but there are some configuration settings that are different, which are noted in the steps below.

1. Directly connect the ESP8266 breakout board to the computer with a USB to TTL Serial cable
2. Set the following settings for the nodeMCU flasher
    ```
    flash size: 4MB
    baud rate: 115200
    flash speed: 40 MHz
    QIO
    ```
3. Use the following mapping of bin files to flash the board
    ```
    boot_v1.7.bin  → 0x00000
    user1.2048.new.5.bin → 0x01000
    esp_init_data_default_v08.bin → 0x1FC000
    blank.bin → 0xFE000 and 0x1FE000
    ```
4. If the previous mapping doesn't work, try the following
    ```
    boot_v1.7.bin  → 0x00000
    user1.2048.new.5.bin → 0x01000
    esp_init_data_default_v05.bin → 0x3FC000
    blank.bin → 0x7E000 and 0x3FE000
    ```
5. After flashing is complete, open a serial monitor (like Arduino or Termite) to test that the chip is now programmed for AT commands
    - Press reset, you should see some gibberish and then "ready"
    - If you type "AT", it should return "OK"
    - If you type "AT+GMR", it should return firmware version and some other specs

Debugging tips
1. Make sure you’re using the right baud rate
2. Check the wiring, especially TX and RX are correct 
3. Make sure EN is pulled high and GPIO 0 is pulled low 
4. Make sure the mapping is correct
5. Check ESP github to see if there are any changes
6. You may have to follow the similar pattern used in previous versions for 32Mbit if it isn’t explicitly written
7. Press reset on the board before flashing
8. Make sure there is common ground between all electronics components
9. If the blue light is flashing and/or AT commands are not working, use baud rate of 74880 to look at messages with a serial monitor
    - This rate should provide more readable messages that can provide more insight 

# Arduino

## One Time Setup 
1. Download Arduino IDE
2. Install the WiFiEspAT libray using IDE
3. Compile ROS package using instructions in main readme (`catkin_make`)
4. Navigate to library installation location (likely `$HOME/Arduino/libraries`) and build Arduino library with `rosrun rosserial_arduino make_libraries.py .`
5. Compile Arduino code using IDE
6. Upload code to Arduino using IDE

## Arduino Code Upload
Once the setup steps are completed, you should only have to compile and upload the Arduino code.

Note: If you change any message in the ROS package or install any new message libraries, you'll have to delete the `ros_lib` Arduino package and re-run the make_libraries command

## Possible Arduino Code Modifications 
- NoodleBot communicates with the learning laptop via wifi. To set up your wifi, modify the `ssid`, `pass`, and `server` parameters in `wifi.h`
    - `ssid` is the name of your wireless router
    - `pass` is the wifi password
    - `server()` includes the ip address of the learning laptop. In ubuntu, you can get the ip address with the command `ifconfig`
- If you changed the PWM pins of the servos on the Teensy, modify the Arduino code to indicate the servo pins on the teensy.
    - the Arduino code uses Serial1 pins on Teensy to connect it with the WiFi breakout board
- If you want to use April Tags instead of the Marvelmind IPS system, see details below.

# State Information 

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
DDSM Driver HAT (A)
DDSM Driver HAT (A)
DDSM Driver HAT (A).jpg

UART, RS485, USB
Introduction
This is a driver board for the DDSM series direct drive servo hub motors, featuring 4x DDSM115 motor interfaces and 4x DDSM210 interfaces, integrated with an ESP32 and hub motor control circuit. We have open-sourced the hub motor control demos and the JSON communication interface. Users can connect via USB and use a host computer to send JSON-formatted commands to control the hub motors and receive JSON-formatted feedback (such as motor current, speed, temperature, etc.).
Users can also perform secondary development, using it as a slave controller in robotic projects.

Features
Suitable for DDSM series hub motors such as DDSM115, DDSM210, and DDSM315.
Onboard 4-ch DDSM115 motor interfaces and 4-ch DDSM210 motor interfaces.
Provides DDSM hub motor control demo and SDK, easy to use.
Supports multiple wired and wireless communication modes and JSON command interaction for easier control.
Supports connecting to Raspberry Pi, powering the Pi via integrated 5V buck regulator circuit and communicating through GPIO UART interface, neat cable management.
Integrated WEB application for controlling and setting the DDSM hub motors, more convenient for debugging.
Supports controlling the DDSM hub motors directly via onboard Type-C port.
Onboard Interface
Ddsm.png

Usage Precaution
Due to the unique structure of the hub motor, it cannot be placed directly on a table to operate. It is best to fix the axle of the hub motor. If you do not have the appropriate components to install the hub motor, you can set a heartbeat function to automatically stop the hub motor from running. The specific method will be introduced later. Also, you can use UGV Suspension (A) or UGV Suspension (B) to install the hub motor, please select one according to your hub motor model.
UGV-Suspennsion-Diff.png

We currently have only three hub motor models — DDSM115, DDSM210, and DDSM315 — that can be used with the DDSM Driver HAT (A). Different models of hub motors cannot be used together simultaneously, as their control commands and feedback information are different. Before setting the ID for the motor, do not connect the motor and the driver board.
Basic Usage
Power Supply
The DDSM Driver HAT (A) board can be powered through either the DC5525 or XT60 power interfaces.
The DDSM Driver HAT (A) board supports a 9~28V DC power supply, but the specific power specifications need to be determined based on the model of the DDSM hub motor you are using:

When using the DDSM115, the power supply range is 12~24V.
When using the DDSM210, the power supply range is 11~22V.
The current of the power supply should be chosen according to the operational requirements of the hub motor. The higher the supported current, the better the effect. In our usage tests, a 24V 6A power supply was suitable for most scenarios.

ESP32 Control
DDSM Driver HAT (A) comes with the control demo for the hub motor by default. Please ensure "Serial Port Control Switch" is switched to "ESP32" model (as shown below). Then, the hub motor can receive the control commands from ESP32. With the power supply from DC5525 or XT60 port, the ESP32 of the diver board will be automatically on and establish hotspot for users. DDSM-Driver-HAT-A-details-7.jpg

The hotspot established by ESP32 is "ESP32-AP" by default, and the default password is "12345678". You can connect this hotspot through your smartphone, tablet or PC. Open the browser, input "192.168.4.1" on the address bar to enter the Web application for the hub motor control.
In the web control application, there is a JSON command input box. Below is an example of a JSON command. You can click "INPUT" next to the JSON command example to automatically fill the corresponding JSON command into the input box. After that, you can change the values of the command in the input box. Once modified, click "SEND" to send the command to the ESP32.
Based on the model of the hub motor you need to control, send instructions to the drive board to set the communication protocol of the drive board. By default, the drive board will interpret the control and feedback instructions of the hub motor according to the DDSM115 communication protocol.
If you need to control the DDSM210 hub motor, click on "INPUT" next to the "CMD_TYPE_DDSM210 JSON" instruction in the DDSM CTRL. You will see this corresponding JSON instruction appear in the input box above. Then, click "SEND" to send it, setting the drive board to interpret the control and feedback instructions of the hub motor according to the DDSM210 communication protocol.
If the communication protocol is set to DDSM210 and you need to control the DDSM115 hub motor, use the CMD_TYPE_DDSM115 JSON instruction in the DDSM CTRL to switch to the DDSM115 communication protocol.
This process does not require connecting the hub motor, and the setting only needs to be performed once each time the drive board is powered on.

Set Motor ID
The ID of the hub motor is "1" by default. Before controlling the hub motor, you need to set the motor ID.

Connect the hub motor to the driver board. Please note that there is only a hub motor can be connected to the driver board. If you connect it with multiple hub motors, then they will be set to the same ID.
Power the driver board through DC5525 or XT60 port. Connect ESP32-AP's hotspot to your smartphone, tablet or PC (password: 12345678). Open the browser to visit "192.168.4.1" for the Web application.
Click on "INPUT" next to the "CMD_DDSM_CHANGE_ID" instruction in DDSM CTRL, and you can see the above input box displaying the JSON command "{"T":10011,"id":1}" to set the motor ID:
T: The command type for setting the motor ID is 10011. Different command operations have their respective command types. This is a fixed value and cannot be modified.
id: Change this to the ID number you need to set for the hub motor, which should be between 1 and 253. The newly set ID supports power-off retention.
If you forget the ID previously set for the hub motor, or need to check whether the ID was set successfully, you can send the "CMD_DDSM_ID_CHECK" command in the DDSM CTRL to query the hub motor ID. The feedback information will be:

{"T":20010,"id":6,"typ":210,"spd":-1,"crt":-2,"act":1,"tep":40,"err":0}
Speed Close-loop Control
Set Heartbeat Function
Before controlling the rotation of the hub motor, you need to secure the hub motor shaft. If you currently do not have the corresponding structural parts to secure the motor shaft, you can set a heartbeat function to automatically stop the hub motor from rotating (in the default speed loop mode).

Click on "INPUT" next to the CMD_HEARTBEAT_TIME command. You will see the JSON command for setting the heartbeat function displayed in the input box at the top of the web interface: {"T":11001,"time":2000}, where:
T: The command type for setting the heartbeat function is 11001. This is a fixed value and cannot be modified.
time: The time, in milliseconds. When the time value is -1, the heartbeat function is disabled. When the time value is 2000, if no new command is received within 2 seconds after sending a motion control command, the drive board will automatically stop the hub motor from rotating. This can prevent the hub motor from continuously rotating.
There are two points to note:

In the examples we provide, the heartbeat function only applies to the four hub motors with IDs 1, 2, 3, and 4.
When the hub motor is in position loop mode, it is best to disable the heartbeat function (i.e., set the time value to -1), otherwise, the automatic execution of the heartbeat function in position loop mode will cause the hub motor to return to the 0 position.
For example:
You can set a heartbeat function command with a time value of 1000. Enter the JSON command in the input box: {"T":11001,"time":1000} and click SEND. This way, when you control the hub motors with IDs 1, 2, 3, and 4 to rotate, they will automatically stop after rotating for 1 second. This prevents potential danger caused by the motor suddenly rotating without structural parts securing the motor shaft.
Speed Control
Click on "INPUT" next to the CMD_DDSM_CTRL JSON command. You will see the JSON command for speed loop control of the motor displayed in the input box at the top of the web interface: {"T":10010,"id":1,"cmd":50,"act":3}.
T: set the command type of the heartbeat function is 11001, which is the fixed value and cannot be modified.
id: should be modified to the controlled motor ID.
cmd: the value of the action command needs to correspond to the specific range of values for the particular loop mode.
act: the acceleration time per revolution is measured in 0.1 milliseconds. The larger the act value, the smoother the speed change.
In the default speed loop mode, for DDSM115 and DDSM210 hub motors, the cmd value is the target speed, but the units are different: for DDSM115, the unit is revolutions/minute (rpm); for DDSM210, the unit is 0.1 revolutions/minute (0.1rpm). For example, if cmd=100, for DDSM115, the target speed is 100rp,. while for DDSM210, the target speed is 10rpm.
Click on "SEND" to send this command, and then the hub motor will rotate. If you do not set the heartbeat function (for the heartbeat function, the time value=-1), the hub motor will keep rotating. Then you can use the above speed control command to modify the cmd value of the target speed as "0", the hub motor will stop rotating. Also, you can use "CMD_DDSM_STOP" command to stop the motor from rotating.

Motor Mode Switch
The hub motor supports switching to different motor modes, while they are all in speed-loop mode after powering on again (although with the speed-loop control part). You can use the "CMD_CHANGE_MODE" command to switch the modes, and you can see the JSON commands "{"T":10012,"id":1,"mode":2}":

T: the command type of the motor mode switch is "10012", which is a fixed value and cannot be modified.
id: the ID in motor mode to be switched.
mode: the corresponding value of the motor mode:
0: Open loop, only DDSM210 has an open loop mode, similar to the PWM control of a DC motor (similarly, when this value is too low, the motor will not rotate; it needs to reach a certain threshold before it starts rotating). The larger the absolute value of the cmd, the greater the acceleration. The range of cmd values is -32767 to 32767.
1: Current loop, only DDSM115 has a current loop mode, which is closed-loop control of the current value. The larger the absolute value of the cmd (hereafter referred to as cmd value, as the CMD_DDSM_CTRL JSON command controls the mode in all cases), the greater the acceleration. The range of cmd values is -32767 to 32767, corresponding to -8A to 8A, but DDSM115 only supports up to 2.7A.
2: Speed loop, a mode available in both DDSM115 and DDSM210, and the default mode when the hub motor is powered on. It can control the speed value in a closed loop. The range of cmd values is determined by the motor's maximum speed, which depends on the power supply capability and the motor's load. The larger the load, the lower the achievable maximum speed. DDSM115 can reach 200 rpm without load; DDSM210 can reach 210 rpm without load.
3: Position loop, a mode available in both DDSM115 and DDSM210. When switching the motor to the position loop, the speed should not be too high; it is best to switch to the position loop when the motor is stationary. When switching to the position loop, the cmd value is 0. However, if a motor already in the position loop receives the command to switch to the position loop again, the position 0 will not be updated. In the position loop mode, the range of cmd values is 0 to 32767 (0 to 360°). The value increases gradually for clockwise rotation, and the motor will automatically plan the shortest path to move to the target position. For example, if the motor is currently at position 0 but needs to rotate counterclockwise to a certain position, the cmd value should be greater than 32767/2 because the path to the target position is shorter when rotating counterclockwise.
Get Motor Feedback
When controlling the motor with the "CMD_DDSM_CTRL" command, the motor feeds back current status information.
You can use the "CMD_DDSM_INFO" command to obtain other information feedback.
JSON Command Wired Control
The "Basic Usage" section introduces how to send JSON commands on the Web application and the JSON command, this section will introduce how to send JSON commands to the control the driver board and the hub motor through the wired connection.
Also, you can use Raspberry Pi, Jetson, PC or other devices with USB interface (for serial port communication) as the host device. Connect the driver board to the host device with an USB cable or 40PIN serial ports, then you can run the Python demo on the host device, or control the hub motor with JSON commands through serial port assistant.

Serial Port Assistant
This section introduces how to use serial port assistant to control the hub motor:

Download serial port assistant.
Connect the hub motor to the driver board, power on the driver board, and connect the host device to the "ESP32-USB" Type-c interface through a USB cable. Please switch on ESP32 gear.
Double-click the downloaded serial port assistant. Select USB com port, and open the serial port. Set the baud rate as 115200, and edit the JSON command. You need to manually add the line break \n after the JSON command, or you can leave "\n" out and check the "AddCrLf" box.
The functions of JSON command must be the same with the above content in Web application section.
DDSM-Driver-HAT-A-SSCOM.png

Python UART Communication
This chapter mainly introduces how to use a Python UART port to establish communication between the robotic arm and devices such as PC/Raspberry Pi/Jetson Orin Nano. Here, we take the UART communication between the PC and the robotic arm as an example:

Install Python on Windows
First, download the latest Python package from Python official website according to your OS. Here, the system downloaded is Windows 3.12.0.
RoArm-M2-S Python.jpg
After downloading, double-click python-3.12.0-amd64. Please ensure to check "Add python.exe to PATH", then click "Customize installation" to enter "Optional Features".
Note that if you are installing it on a Windows system, be sure to check the "Add Python.exe to PATH" box.
RoArm-M2-S Python02.png
In the Optional Features interface, keep the check mark and click "Next" to enter the "Advanced Options" interface.
In the "Advanced Options" interface, you can click "Browse" to change the installation address to the address you want to install, here is the default installation address, click " Install" to install after setting, and wait for the installation to complete.
RoArm-M2-S Python03.png
Once installed, we can then compile the Python project and control the robotic arm with JSON commands using the Python demos.

Configure Python Virtual Environment
Click here to download the Python demo, unzip it, input "cmd" on the startup menu, open the Windows command prompt interface, and then input "cd file path" to enter the " ddsm_python" project:

cd C:\Users\liuwei\Downloads\Ddsm_python\ddsm_python (liuwei is the username, you need to modify it as yours.)
Create a virtual environment for this project in this project folder by typing the command: python -m venv [the name of the virtual environment, generally named project "env"].

python -m venv ddsm-env
To activate this virtual environment, you can input: project name-env\Scripts\activate.bat.

ddsm-env\Scripts\activate.bat
After unpacking, you can see that there is a requirements.txt file in the demo, which is a list of packages needed for the DDSM_Python project, you can install all the necessary packages directly into the virtual environment by typing the following command:

python -m pip install -r requirement.txt
Ddsm pythonverify.png
The virtual environment has been established and the installation packages used in this project have been installed, so let's run the serial communication demos next.

Python UART Communication
The Python demo for serial communication is serial_simple_ctrl.py as shown below:

import serial
import argparse
import threading

def read_serial():
    while True:
        data = ser.readline().decode('utf-8')
        if data:
            print(f"Received: {data}", end='')

def main():
    global ser
    parser = argparse.ArgumentParser(description='Serial JSON Communication')
    parser.add_argument('port', type=str, help='Serial port name (e.g., COM1 or /dev/ttyUSB0)')

    args = parser.parse_args()

    ser = serial.Serial(args.port, baudrate=115200, dsrdtr=None)
    ser.setRTS(False)
    ser.setDTR(False)

    serial_recv_thread = threading.Thread(target=read_serial)
    serial_recv_thread.daemon = True
    serial_recv_thread.start()

    try:
        while True:
            command = input("")
            ser.write(command.encode() + b'\n')
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()


if __name__ == "__main__":
    main()
Connect the hub motor to the driver board, and power on the driver board. Connect the host and the ESP-USB Type-C interface of the driver board with a USB cable. Switch on the ESP32 gear.
After connection, search for Device Manager in the search bar of "Start" to check the newly inserted port number, here the newly inserted port number is COM67, different computers insert different port numbers, please remember the new port number.
DDSM-Python91.png
Use the following commands to run the UART communication demo, please remember to add the port number that the robotic arm is connected to. Replace COM67 with the new port number in the PC (here only one motor is connected). If you use devices such as Raspberry Pi and Jetson Orin Nano, you need to change them to the corresponding port name.

python serial_simple_ctrl.py COM67
After the run is complete, the terminal will not feedback any messages.
You can input JSON command on the terminal and press Enter, then DDSM Driver HAT (A) supports communication. For example, You need to get the ID of the currently connected motor (only one motor can be connected at this time), you can send {"T":10031}, and a message will be sent back after successful sending.
DDSM-Python92.png

Product Initialization
Users who need to quickly restore their product to the factory program can use the ESP32 download tool for RoArm-M2-S that we provide.

1. Click here to download, unzip it and double-click "flash_download_tool_3.9.5.exe" to open. Then, two windows pop up. The UI interface of the download tool is for operation, and the other window is the terminal to display the working status of the download tool.
2. In the "DOWNLOAD TOOL MODE" interface, select "Chip Type" as ESP32, and "WorkMode" as Factory, and the relative path will be used when calling the binary file, so you don't need to manually enter the binary file path, click OK.
WAVEROVER Demo01.png

3. Enter ESP32 FLASH DOWNLOAD TOOL interface. As the picture shown below, you can upload the demo to 8 driver board on the right. Switch on ESP32-USB gear on the DDSM Driver HAT (A) driver board, and then connect the ESP32-USB interface of the driver board to the PC with a USB cable. Click on "COM" port, select the new COM port (here is COM67); BAUD is for setting the downloading speed. The higher the value, the faster the speed. The baud rate for ESP32 is up to 921600.
DDSM-Factory1.png

4. After the selection, click on "START" to start uploading the demo, after the upload is completed. "IDLE" will change to "FINISH". Then, the driver board can be disconnected from the USB. Connect to the hub motor, power on it and then you can operate it by following #Basic_Usage.
DDSM-Factory2.png DDSM-Factory3.png

Resource
STEP Model
STEP Model
Schematic
Schematic
Example Demo
Example Demo
Downloader
Downloader
Python Demo
Python Demo
SSCOM
Serial port assistant
Support
# RM2024 RDC - W1NT3R

## Introduction
We are W1NT3R, Group 13 of the HKUST Robot Design Competition 2023. This repository contains the codes and configurations for our Task Robot, used to transport seedlings from a loading zone to specified planting zones.

------------
[TOC]

------------

## Images
![](https://files.catbox.moe/cjregv.png)

![](https://files.catbox.moe/9az16h.jpg)

## Contributors
- @DuckDuckCho
- @SunnyFarmDay
- @Qlinaq
- @wilsonooo09
- @z1y4n
- @ZoellickHan

## Structure

### Instructions for Contributing
- The directory `RM2024-RDC-Core` contains the drivers and controller
- Within `./Core/Inc/AppConfig.h`, you could enable/disable certain drivers and configure them.
- Tasks are created in `./Core/Src/UserTask.cpp`.

### Code Structure
Using FreeRTOS, different tasks are initialized when the robot is powered.

- DR16 Decoding and Processing with UART
	- Utilizing `rxEventCallback()` to receive data
	- Decoding and converting data from the DR7 controller
	- Determining the mode of operation and setting target RPM for the corresponding motors
		- Auto shortcut mode: Automatic mode for going through the auto shortcut
		- Driving mode: Manual control for picking up and transporting seedlings
			- Forward/Backward mode: Used for transporting the car
			- Robotic Arm mode: Used to operate the robotic clamp
	- Limiting the target RPM to prevent overly high output
	- Receiving the next round of data with `HAL_UARTEx_ReceiveToIdle_IT()`

- Receiving Data from M3508 Motors with CAN
	- Configure 2 list mode CAN filters (`CAN_FILTERMODE_IDLIST`), corresponding to the chassis motors and the clamp motors
	- Check for available messages with `HAL_CAN_GetRxFifoFillLevel()`
	- Get CAN ID and verify the information with `rxHeader`
	- Decode the message for each motor

- Transmitting Data to M3508 Motors with CAN
	- Check for the connection status of the controller to verify the messages sent
	- Get the target motor RPM specified by the DR16 processing
	- Get the current motor RPM returned from CAN by the motor
	- Run PID control to obtain a required output current
	- Transmit the current to the motor with `HAL_CAN_AddTxMessage()`
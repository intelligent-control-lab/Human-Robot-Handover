# Human-Robot-Handover
This repo implements the safe robot handover.

Hardwares: FANUC LR-mate 200id 7L robot, Speedgoat.

Question contact: 
* ruixuanl@andrew.cmu.edu
* ruic3@andrew.cmu.edu

## Setup
1. Follow the instructions in `./Calibration/` to calibrate the camera.
2. Build the controller.
* `cd Real-time-Jerk-bounded-Position-Controller/`.
* `make all`.
* `cd ..`.
3. Adjust the parameters in `demo.m`.
* `ROBOT`: Currently using `LRMate200iD7L`.
* `MODE`: 
  1) `HumanINTR`: Virtual interaction. The user drags the mouse to move a virtual human in the environment. The virtual human only has motion in the xy plane.
  2) `CamerINTR`: Real interaction. The camera captures the real human motion.
* `USE_ROBOT`: 
  1) 0: use robot simulation. 
  2) 1: use the real robot.
* `enbSSA`:
  1) 0: Turn off the safe control. 
  2) 1: Turn on the safe control.
* `USE_GRIPPER`:
  1) 0: No gripper needed.
  2) 1: Gripper is on.
* `USE_FTS`:
  1) 0: No force-torque sensor needed.
  2) 1: Force-torque sensor is connected.

Note: The gripper and the FTS will be used only when `USE_GRIPPER` and `USE_FTS` are both turned on.

## Operation Guide
The handover pipeline is tested on FANUC LR-mate 200id 7L robot with the Speedgoat basline.
1. Turn on the Speedgoat by pressing the power button.
2. Turn on the FANUC robot by turning the switch 90 degree clockwise.
3. On the teaching pendant, press `Shift`+`Reset`.
4. On the teaching pendant, press `FCTN` -> `Abort All`.
5. On the teaching pendant, select the Stream Motion program.
6. On the robot controll box, press the green button.
7. Run `demo.m`.


## Acknowledgement
This project is in part supported by Siemens.

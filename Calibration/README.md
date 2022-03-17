# Calibration
## Dependencies
1. python3 -m pip install opencv-python
2. python3 -m pip install opencv-contrib-python
3. python3 -m pip install numpy
4. python3 -m pip install scipy


## Camera Calibration Procedure
1. Print an Aruco tag and place it in the scene. Make sure it is visible to the camera.
2. Run `test_kinect.m`.
3. Adjust parameters in `calib.py`.
* `cam_mtx`: Camera intrinsic matrix.
* `dist_coeff`: Camera distortion coefficients.
* `aruco_marker_side_length`: Side length of the printed marker. Need to be manually measured. Unit: m.
* `robot_to_tag`: Transformation matrix from the world frame to the printed tag frame. Need to be mannually measured/estimated.
4. `python3 calib.py`.

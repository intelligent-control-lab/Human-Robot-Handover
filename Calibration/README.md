# Calibration
## Dependencies
1. python3 -m pip install opencv-python
2. python3 -m pip install opencv-contrib-python
3. python3 -m pip install numpy
4. python3 -m pip install scipy


## Execute Camera Calibration
1. Run `test_kinect.m`.
2. Adjust parameters in `calib.py`.
* `cam_mtx`: Camera intrinsic matrix.
* `dist_coeff`: Camera distortion coefficients.
* `aruco_marker_side_length`: Side length of the printed marker. Need to be manually measured. Unit: m.
* `robot_to_tag`: Transformation matrix from the world frame to the printed tag frame. Need to be mannually measured/estimated.
3. `python3 calib.py`.

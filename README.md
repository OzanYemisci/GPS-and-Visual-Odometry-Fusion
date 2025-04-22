#Camera-GPS Sensor Fusion
This project demonstrates the integration of Visual Odometry (VO) with GPS/RTK data, using the KITTI dataset. The goal is to improve positioning accuracy by introducing artificial noise to GPS and refining trajectories with an Unscented Kalman Filter (UKF).

## Folder Structure
EE492 Project/
│
├── devkit/                  # KITTI devkit for loading GPS and pose
├── 2011_09_26_drive_0022/   # KITTI data (images, oxts, calib)
├── kalmanFilterRT.m         # Kalman smoothing of GPS
├── kalmanFiltering.m        # UKF for fusing VO + GPS
├── addGpsRtkNoiseInterval.m # Adds Gaussian noise to GPS intervals
├── Main.m                   # Main script that runs the full pipeline
└── helper*.m                # Helper functions for VO and 3D pose estimation


## How to Run
- Download the KITTI dataset (drive 2011_09_26_drive_0022_sync).
- Place the dataset folders (image_02, oxts, and calib_cam_to_cam.txt) in the correct paths.
- Run the Main.m script in MATLAB.
- Output figures will show:
  - Camera trajectory vs ground truth
  - Noisy GPS vs smoothed vs filtered trajectory
  - Error plots and improvement percentage

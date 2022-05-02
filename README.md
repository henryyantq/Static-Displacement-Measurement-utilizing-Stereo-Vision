# Static Displacement Measurement Utilizing Stereo Vision

We proposed a scheme for static displacement measurement utilizing stereo vision. The experiment of this scheme was conducted using a single smartphone camera, in order to eliminate the differences between the 'two cameras'. 

A marker with white background and a single red-filled circle in the middle is designed for ROI positioning. A focal calibration program is established for calculating the focal value (in pixels) of the camera, and a binocular displacement measurement algorithm is designed for retrieving the metric/world coordinate (in mms) of the ROI center so as to calculate the final metric displacement.   

[Marker: To be downloaded and printed](https://github.com/henryyantq/Static-Displacement-Measurement-utilizing-Stereo-Vision/blob/main/marker.png)

[Focal calibration](https://github.com/henryyantq/Static-Displacement-Measurement-utilizing-Stereo-Vision/blob/main/calib_for_focal_in_stereo.cpp)

[Binocular displacement measurement](https://github.com/henryyantq/Static-Displacement-Measurement-utilizing-Stereo-Vision/blob/main/Binocular.cpp)

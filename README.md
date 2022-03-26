# HAVCV
Hardware Accelerated Vibrometer based on Computer Vision. Normally, a vibrometer is an instrument that reflects the surface stress distribution by detecting the surface vibration. It is usually utilized for evaluating the mechanical defection of a surface. The first version

HAVCV is written in C, utilizing the OpenCV 4 libraries. **Noting that the cpp file should be compiled after one deploys OpenCV 4 correctly**. _An Nvidia Jetson platform is recommended for built-in hardware acceleration, and the program was originally tested using Qt Creator 5.12.0 on Nvidia Jetson Nano 2GB with an 8MP camera that is able to retrieve video frames in 720@60fps and 480p@120fps_.

[Reference Paper](http://sstl.cee.illinois.edu/papers/aeseancrisst15/248_Lee_Monocular.pdf)

HAVCV use feature recognition which automatically detects ROIs.

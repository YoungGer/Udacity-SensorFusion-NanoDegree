# SFND 2D Feature Tracking

<img src="images/keypoints.png" width="820" height="248" />

The idea of the camera course is to build a collision detection system - that's the overall goal for the Final Project. As a preparation for this, you will now build the feature tracking part and test various detector / descriptor combinations to see which ones perform best. This mid-term project consists of four parts:

* First, you will focus on loading images, setting up data structures and putting everything into a ring buffer to optimize memory load. 
* Then, you will integrate several keypoint detectors such as HARRIS, FAST, BRISK and SIFT and compare them with regard to number of keypoints and speed. 
* In the next part, you will then focus on descriptor extraction and matching using brute force and also the FLANN approach we discussed in the previous lesson. 
* In the last part, once the code framework is complete, you will test the various algorithms in different combinations and compare them with regard to some performance measures. 

See the classroom instruction and code comments for more details on each of these parts. Once you are finished with this project, the keypoint matching part will be set up and you can proceed to the next lesson, where the focus is on integrating Lidar points and on object detection using deep-learning. 

## Dependencies for Running Locally
* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* OpenCV >= 4.1
  * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
  * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./2D_feature_tracking`.

# Benchmark
The example matching result is:
<img src="images/example.png" width="820" height="180" />

I have created `experiments.py` which loops through all the possible combination of detector and descriptor pairs.

Run it: `python experiments.py`.

This creates a `output/result.txt` file.

I used  KNN match selection (k=2) and performed descriptor distance ratio filtering with t=0.8 in file `matching2D.cpp`.

|Sr. No. | Detector + Descriptor |Total Keypoints |Total Matches |Total Time (ms) |
|:---:|:---:|:----:|:-----:|:-----:|
|1 | SHITOMASI + BRISK |1179 |767 |2291.16 |
|2 | SHITOMASI + BRIEF |1179 |944 |125.191 |
|3 | SHITOMASI + ORB |1179 |908 |126.824 |
|4 | SHITOMASI + FREAK |1179 |768 |365.967 |
|5 | SHITOMASI + AKAZE |N/A |N/A |N/A |
|6 | SHITOMASI + SIFT |1179 |788 |188.365 |
|7 | HARRIS + BRISK |248 |142 |2276.44 |
|8 | HARRIS + BRIEF |248 |173 |135.613 |
|9 | HARRIS + ORB |248 |162 |139.757 |
|10 | HARRIS + FREAK |248 |144 |376.209 |
|11 | HARRIS + AKAZE |N/A |N/A |N/A |
|12 | HARRIS + SIFT |248 |166 |212.742 |
|13 | FAST + BRISK |1491 |899 |2190.39 |
|14 | FAST + BRIEF |1491 |1099 |48.353 |
|15 | FAST + ORB |1491 |1071 |47.8879 |
|16 | FAST + FREAK |1491 |878 |310.866 |
|17 | FAST + AKAZE |N/A |N/A |N/A |
|18 | FAST + SIFT |1491 |924 |115.162 |
|19 | BRISK + BRISK |2762 |1471 |4581.71 |
|20 | BRISK + BRIEF |2762 |1704 |2416.18 |
|21 | BRISK + ORB |2762 |1675 |2421.21 |
|22 | BRISK + FREAK |2762 |1460 |2691.6 |
|23 | BRISK + AKAZE |N/A |N/A |N/A |
|24 | BRISK + SIFT |2762 |1361 |2495.69 |
|25 | ORB + BRISK |1161 |553 |2327.52 |
|26 | ORB + BRIEF |1161 |545 |176.963 |
|27 | ORB + ORB |1161 |562 |184.929 |
|28 | ORB + FREAK |1161 |512 |446.59 |
|29 | ORB + AKAZE |N/A |N/A |N/A |
|30 | ORB + SIFT |1161 |510 |245.217 |
|31 | AKAZE + BRISK |1670 |1193 |2556.14 |
|32 | AKAZE + BRIEF |1670 |1266 |436.603 |
|33 | AKAZE + ORB |1670 |1260 |425.875 |
|34 | AKAZE + FREAK |1670 |1178 |702.261 |
|35 | AKAZE + AKAZE |N/A |N/A |N/A |
|36 | AKAZE + SIFT |1670 |1082 |468.205 |
|37 | SIFT + BRISK |1387 |565 |2560.5 |
|38 | SIFT + BRIEF |1387 |704 |617.944 |
|39 | SIFT + ORB |1387 |683 |602.68 |
|40 | SIFT + FREAK |1387 |591 |858.346 |
|41 | SIFT + AKAZE |N/A |N/A |N/A |
|42 | SIFT + SIFT |1387 |625 |654.526 |

## Top 3 detector/ descriptor pairs

I choose best 3 detector/ descriptor pairs according to the total time.

|Sr. No. | Detector + Descriptor |Total Keypoints |Total Matches |Total Time (ms) |
|:---:|:---:|:----:|:-----:|:-----:|
|1 | FAST + ORB |1491 |1071 |47.8879 |
|2 | FAST + BRIEF |1491 |1099 |48.353 |
|3 | SHITOMASI + BRIEF |1179 |944 |125.191 |

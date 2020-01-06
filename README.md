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

## Results

Some notes:
1- AKAZE detectors only work with AKAZE descriptors
2- Sift does not work with orb

Number of keypoints by each detector is given in table below

|      | akaze | sift | harris | shitomasi | fast  | brisk | orb |
|------|-------|------|--------|-----------|-------|-------|-----|
| img0 | 166   | 138  | 17     | 125       | 419   | 419   | 264 |
| img1 | 157   | 132  | 14     | 118       | 427   | 427   | 282 |
| img2 | 161   | 124  | 18     | 123       | 404   | 404   | 282 |
| img3 | 155   | 137  | 21     | 120       | 423   | 423   | 277 |
| img4 | 163   | 134  | 26     | 120       | 386   | 386   | 297 |
| img5 | 164   | 140  | 43     | 113       | 414   | 414   | 279 |
| img6 | 173   | 137  | 18     | 114       | 418   | 418   | 289 |
| img7 | 175   | 148  | 31     | 123       | 406   | 406   | 272 |
| img8 | 177   | 159  | 26     | 111       | 396   | 396   | 266 |
| img9 | 179   | 137  | 34     | 112       | 401   | 401   | 254 |

with mean values shown here

|      | akaze | sift | harris | shitomasi | fast  | brisk | orb |
|------|-------|------|--------|-----------|-------|-------|-----|
| mean | 167   | 138  | 19     | 125       | 410   | 310   | 271 |

Mean of number of matches between different descriptors/detectors is shown here

|       | akaze | sift | harris | shitomasi | fast  | brisk | orb |
|-------|-------|------|--------|-----------|-------|-------|-----|
| akaze | 120   | -    | -      | -         | -     | -     | -   |
| sift  | -     | 92   | 19     | 104       | 310   | 190   | 85  |
| brisk | -     | 68   | 18     | 79        | 210   | 142   | 76  |
| brief | -     | 74   | 18     | 92        | 248   | 149   | 51  |
| orb   | -     | -    | 18     | 86        | 231   | 98    | 65  |
| freak | -     | 60   | 15     | 62        | 168   | 125   | 40  |

Time taken for all above-mentioned combinations to calculate for all 10 images are shown here

|       | akaze | sift | harris | shitomasi | fast  | brisk | orb  |
|-------|-------|------|--------|-----------|-------|-------|------|
| akaze | 943   | -    | -      | -         | -     | -     | -    |
| sift  | -     | 1096 | 213    | 198       | 192   | 2227  | 331  |
| brisk | -     | 2322 | 1883   | 1862      | 1843  | 3801  | 2011 |
| brief | -     | 743  | 128    | 117       | 72    | 2138  | 126  |
| orb   | -     | -    | 148    | 136       | 231   | 2147  | 340  |
| freak | -     | 983  | 381    | 372       | 328   | 2451  | 402  |

From the data shown in the tables above; and since we are aiming at collision avoidance system in a moving vehicle. I believe
fast results are with highest importance. Here are my top 3 recommendations:
1- fast has largest number of keypoints detected, when combined with brief, it also gives high matches with considerable low time
2- also, fast with sift can give considerably better results that other combinations
3- shitomasi with brief can be a good combination according to number of matches given time




# SFND 2D Feature Tracking

<img src="media/keypoints.png" width="820" height="248" />

## The project consists of four parts:
1. The Data Buffer: You will start with loading the images, setting up the data structure, and put everything into the data buffer.
2. Keypoint Detection: You will integrate several keypoint detectors, such as HARRIS, FAST, BRISK, ORB, AKAZE, and SIFT, and compare them to each other based on the number of key points and speed.
3. Descriptor Extraction & Matching: Extract the descriptors and match them using the brute-force and FLANN approach.
4. Performance Evaluation: You will compare and evaluate which combination of algorithms perform the best concerning performance measurement parameters.

The idea of the camera course is to build a collision detection system - that's the overall goal for the Final Project. As a preparation for this, a the feature tracking part is built to test various detector / descriptor combinations to see which ones perform best. 

code in the following repo was used to follow along and conplete the project
```
git clone https://github.com/udacity/SFND_2D_Feature_Tracking.git
```
## Project rubrics
1. Implement a vector for dataBuffer objects whose size does not exceed a limit (e.g. 2 elements). This can be achieved by pushing in new elements on one end and removing elements on the other end.

Your first task is to set up the loading procedure for the images, which is currently not optimal. In the student version of the code, we push all images into a vector inside a for-loop and with every new image, the data structure grows. Now imagine you want to process a large image sequence with several thousand images and Lidar point clouds over night - in the current implementation this would push the memory of your computer to its limit and eventually slow down the entire program. So in order to prevent this, we only want to hold a certain number of images in memory so that when a new one arrives, the oldest one is deleted from one end of the vector and the new one is added to the other end. The following figure illustrates the principle.

2. Implement detectors HARRIS, FAST, BRISK, ORB, AKAZE, and SIFT and make them selectable by setting a string accordingly.
3. Remove all keypoints outside of a pre-defined rectangle and only use the keypoints within the rectangle for further processing.
4. Implement descriptors BRIEF, ORB, FREAK, AKAZE and SIFT and make them selectable by setting a string accordingly.
5. Implement FLANN matching as well as k-nearest neighbor selection. Both methods must be selectable using the respective strings in the main function.
6. Use the K-Nearest-Neighbor matching to implement the descriptor distance ratio test, which looks at the ratio of best vs. second-best match to decide whether to keep an associated pair of keypoints.
7. Count the number of keypoints on the preceding vehicle for all 10 images and take note of the distribution of their neighborhood size. Do this for all the detectors you have implemented.
8. Count the number of matched keypoints for all 10 images using all possible combinations of detectors and descriptors. In the matching step, the BF approach is used with the descriptor distance ratio set to 0.8.
9. Log the time it takes for keypoint detection and descriptor extraction. The results must be entered into a spreadsheet and based on this data, the TOP3 detector / descriptor combinations must be recommended as the best choice for our purpose of detecting keypoints on vehicles.

## Project description
### Initially
1. input = 10 images
2. output = 9 matched keypoints
3. shitomasi detector 
4. brisk descriptor
5. brute force matching

### To complete the project, following steps were followed
1. The main file is named - MidTermProject_Camera_Student.cpp
- We will take in command-line args that will help us change between
- the detectors and descriptors to prevent constant recompilation
- argv[0] --> Program
- argv[1] --> detectorType: SHITOMASI, HARRIS, FAST, BRISK, ORB, AKAZE, SIFT
- argv[2] --> descriptorType: BRISK, BRIEF, ORB, FREAK, AKAZE, SIFT
- argv[3] --> matcherType (matching algorithms - brute-force, FLANN): MAT_BF, MAT_FLANN
- argv[4] --> selectorType (choosing the best match vs. k best matches): SEL_NN, SEL_KNN
- argv[5] --> bVis (visualising results): 0/1
- argv[6] --> bLimitKpts (for limiting keypoint results): 0/1
- argv[7] --> bFocusOnVehicle (to concentrate on vehicle from KITTI dataset): 0/1
- Defaults
- SHITOMASI, BRISK, MAT_BF, DES_BINARY, SEL_NN, bVis=1, bLimitKpts=0, bFocusOnVehicle=1
```
# example run commands
./2D_feature_tracking HARRIS BRISK MAT_BF SEL_NN 1 0 1
```

2. The code is divided in the following sections
- Initializing sensor and data location path
- Storing commandline agrs in respective string variables
- we are using ring buffers - first in first out queue - where the recent data is added on head side of the queue and the older data from the tail side is accessed first.
- we use a vector of DataFrame - which are defined in the dataStructures.h - it represent all the sensor data available at an instance of time
- later we loop over all images 
- load the image in the buffer using vector manipulation
- extract 2D keypoints from current image
- in the matching2D_Student.cpp, we add a function which uses openCV and allows the use of different detectors which are not SHITOMASI.
- in this project we detect keypoints only in a selected region, e.g. focusing on the car directly ahead. 
- So, we define a rect openCV datatype which generates a rectangle. I have kept only the keypoints inside this rectangular region as a part on the task 3.
- an optional task is to limit the number of keypoints if the option is provided on the commandline args
- the next task, is to add descriptors in file matching2D.cpp and enable string-based selection based on descriptorType for BRIEF, ORB, FREAK, AKAZE, SIFT
- the next task, make a list of keypoints detected for different approaches.
- later find task is to count the number of matched keypoints for all 10 images using all possible combinations of detectors and descriptors. In the matching step, use the BF approach with the descriptor distance ratio set to 0.8.
- ninth task is to log the time it takes for keypoint detection and descriptor extraction. The results are entered into a spreadsheet and based on this information you will then suggest the TOP3 detector / descriptor combinations as the best choice for our purpose of detecting keypoints on vehicles. Finally, in a short text, justified the recommendation based on observations and on the data you collected.


## Dependencies for Running Locally
* cmake >= 2.8
* OpenCV
* gcc/g++ 

## Basic Build Instructions
1. Clone this repo.
2. Make a build directory in the top level directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./2D_feature_tracking`.

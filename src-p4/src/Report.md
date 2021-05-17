# Camera-Based 2D Feature Matching Project Writeup

This project concerns evaluating the different combinations of keypoint detectors and descriptors to decide what combination works best for calculating the Time-to-Collision (TTC) between the ego vehicle and the vehicle directly in front of it.

The main function can accept commandline arguments for quicker experimentation. The project can be run using following commands:

```
mkdir build
cd build
cmake ..
make
 ./2D_feature_tracking detectorType descriptorType matcherType selectorType bVis bLimitKpts bFocusOnVehicle
 
# or 
./2D_feature_tracking
# this runs SHITOMASI, BRIEF, BF_NN, SEL_NN, 1, 0, 1 IN DEFAULT
```

The command-line arguments can in varied configurations which include

- `detectorType`: The type of detector to use - one of `HARRIS`, `FAST`, `BRISK`, `AKAZE`, `SIFT`
- `descriptorType`: The type of descriptor to use - one of `BRIEF`, `ORB`, `FREAK`, `AKAZE`, `SIFT`
- `matcherType`: Matching algorithm - one of `MAT_BF`, `MAT_FLANN`.
- `selectorType`: Keypoint selection method - one of `SEL_NN`, `SEL_KNN`
- `bVis`: Visualising results - one of `0` (disable), `1` (enable)
- `bLimitKpts`: For limiting keypoint display results - one of `0` (disable), `1` (enable)
- `bFocusOnVehicle`: Concentrate on the preceding vehicle and remove all keypoints not within its area - one of `0` (disable), `1` (enable)

NOTES:
1. AKAZE detector works only with AKAZE descriptor
2. SIFT detector cannot be paired with ORB descriptor, it gives out `out of memory` error
--------------------------------------------------------------------------------
# PROJECT AIM
## MP.1 - Data Buffer Optimisation
1. creating a ring buffer
2. the most recent image is sent in on top 
3. and the oldest image is poped out keeping the buffer size constant (in my project 3)

## MP.2 - Keypoint Detection
1. The Shi-Tomasi detector method has already been provided to us as a start. 
2. HARRIS and other modern detectors namely, FAST, BRISK, ORB, AKAZE & SIFT are implemented with the help of OpenCV 3.2.0
3. Detected keypoints are stored in a dataframe (data-structure) vector

## MP.3 - Keypoint Removal
1. Using given region of interest which includes on the car directly in front of the camera, only the keypoints in that region are kept in the keypoint vector.
2. using the input for `bFocusOnVehicle`, a function is setup to acommplish the task
3. it then uses `std::vector::erase()` using this new iterator to finally remove the keypoints in the vector that were outside of the rectangle. This is a very well-known operation in C++ called the [Erase-Remove Idiom](https://en.wikipedia.org/wiki/Erase%E2%80%93remove_idiom) and efficiently removes elements from a STL container that do not meet certain criteria and this was ultimately used to remove the keypoints that are not within the bounding area of the preceding vehicle.

## MP.4 - Keypoint Descriptors
1. The Brief method is used as the default descriptor. 
2. In the original template, the `BRISK` option was already implemented which constructs the right extractor to help extract descriptors from the given keypoints.
3. other descriptors methods namely, FREAK, ORB, AKAZE & SIFT are used with the help of OpenCV 3.2.0
4. descriptors  extracted are stored in a matrix

## MP.5 - Descriptor Matching
1. The brute-force approach has already been provided. 
2. the distance type is now set for the brute-force approach, L2 norm when using `SIFT` and the Hamming distance for the other binary descriptors
3. FLANN is taken from previous exercises

## MP.6 - Descriptor Distance Ratio
1. taken from previous exercises
2. the distance ratio logic was implemented such that if the ratio between the distance of the first match to the second match is less than a threshold (0.8) 

## MP.7 - Performance Evaluation 1
1. to count the number of keypoints for each method over all 10 images provided in this midterm project with no focus on the front vehicle
2. the number of keypoints is solely restricted to the preceding vehicle.

| Image / Detector | HARRIS | FAST | BRISK | ORB | SIFT | AKAZE | SHITOMASI |
|:----------------:|:------:|:----:|:-----:|:---:|:----:|:-----:|:---------:|
|        1         | 102 | 1824 | 2757 | 500 | 1438 | 1351 | 1370 |
|        2         | 90 | 1832 | 2777 | 500 | 1371 | 1327 | 1301 |
|        3         | 99 | 1810 | 2741 | 500 | 1380 | 1311 | 1361 |
|        4         | 103 | 1817 | 2735 | 500 | 1335 | 1351 | 1358 |
|        5         | 138 | 1793 | 2757 | 500 | 1305 | 1360 | 1333 |
|        6         | 257 | 1796 | 2695 | 500 | 1369 | 1347 | 1284 |
|        7         | 74 | 1788 | 2715 | 500 | 1396 | 1363 | 1322 |
|        8         | 171 | 1695 | 2628 | 500 | 1382 | 1331 | 1366 |
|        9         | 140 | 1749 | 2639 | 500 | 1463 | 1358 | 1389 |
|        10        | 207 | 1770 | 2672 | 500 | 1422 | 1331 | 1339 |

- `HARRIS` it was quite sparse where there were very few keypoints detected and it looked like that they were only visible in very strong corners in the image. Sparse detection
- `FAST` attracted to the bridge and trees more rather than the preceding vehicle, but keypoints on the other vehicles were captured
- `BRISK` noisy, quite a large number of keypoints detected
- `ORB` keypoints that were spread out mostly on the road areas and vehicles and less around the bridges and trees
- `SIFT` good spread, well distributed neighbourhood sizes
- `AKAZE` good spread, smaller neighbourhood sizes
- `SHITOMASI` has a good amount of keypoints spread around the image. Good spread around the image

## MP.8 - Performance Evaluation 2
1. the number of matches between all possible pairs of detectors and descriptors. 
2. The data can be found in the  `DATA` subfolder
3. only focusing on the vehicle in the front unlike the previous task

**HARISS DETECTOR**
| Images / Descriptor | BRIEF / ORB / FREAK / SIFT |
|:----------------:|:-----------------------------:|
|        1-2       | 15 |
|        2-3       | 13 |
|        3-4       | 16 |
|        4-5       | 14 |
|        5-6       | 21 |
|        6-7       | 32 |
|        7-8       | 14 |
|        8-9       | 27 |
|        9-10      | 21 |

**FAST DETECTOR**
| Images / Descriptor | BRIEF / ORB / FREAK / SIFT |
|:----------------:|:-----------------------------:|
|        1-2       | 149 |
|        2-3       | 152 |
|        3-4       | 150 |
|        4-5       | 155 |
|        5-6       | 149 |
|        6-7       | 149 |
|        7-8       | 156 |
|        8-9       | 150 |
|        9-10      | 138 |

**BRISK DETECTOR**
| Images / Descriptor | BRIEF / ORB / SIFT | FREAK |
|:----------------:|:---------------------:|:-----:|
|        1-2       | 264 | 242 |
|        2-3       | 282 | 260 |
|        3-4       | 282 | 263 |
|        4-5       | 277 | 264 |
|        5-6       | 297 | 274 |
|        6-7       | 279 | 256 |
|        7-8       | 289 | 269 |
|        8-9       | 272 | 255 |
|        9-10      | 266 | 243 |

**ORB DETECTOR**
| Images / Descriptor | BRIEF / ORB / SIFT | FREAK |
|:----------------:|:---------------------:|:-----:|
|        1-2       | 92 | 46 |
|        2-3       | 102 | 53 |
|        3-4       | 106 | 56 |
|        4-5       | 113 | 65 |
|        5-6       | 109 | 55 |
|        6-7       | 125 | 64 |
|        7-8       | 130 | 66 |
|        8-9       | 129 | 71 |
|        9-10      | 127 | 73 |

**SIFT DETECTOR**
| Images / Descriptor | BRIEF / SIFT | FREAK |
|:----------------:|:---------------:|:-----:|
|        1-2       | 138 | 137 |
|        2-3       | 132 | 131 |
|        3-4       | 124 | 123 |
|        4-5       | 137 | 136 |
|        5-6       | 134 | 133 |
|        6-7       | 140 | 139 |
|        7-8       | 137 | 135 |
|        8-9       | 148 | 147 |
|        9-10      | 159 | 158 |

**AKAZE DETECTOR**
| Images / Descriptor | AKAZE |
|:----------------:|:------:|
|        1-2       | 166 |
|        2-3       | 157 |
|        3-4       | 161 |
|        4-5       | 155 |
|        5-6       | 163 |
|        6-7       | 164 |
|        7-8       | 173 |
|        8-9       | 175 |
|        9-10      | 177 |


# MP.9 - Performance Evaluation 3
1. the amount of time taken (ms) to calculate the detected keypoints and the resulting descriptors over all the 10 images.
2. focus is on the vehicle ahead
3. the data can be found in the `DATA` subfolder

| Image / Detector | HARRIS | FAST | BRISK | ORB | SIFT | AKAZE |
|:----------------:|:------:|:----:|:-----:|:---:|:----:|:-----:|
|        1         | 77.68 | 1.07 | 52.33 | 9.19 | 588.418 | 191.55 |
|        2         | 87.17 | 0.92 | 46.68 | 10.11 | 131.285 | 162.82 |
|        3         | 69.34 | 1.25 | 68.01 | 9.02 | 175.373 | 129.52 |
|        4         | 69.24 | 0.76 | 46.43 | 9.29 | 205.82 | 169.52 |
|        5         | 68.83 | 2.54 | 44.27 | 11.29 | 191.108 | 173.03 |
|        6         | 69.56 | 1.79 | 61.62 | 12.87 | 231.246 | 160.11 |
|        7         | 68.51 | 4.88 | 60.46 | 9.55 | 128.959 | 99.11 |
|        8         | 68.64 | 0.84 | 70.64 | 10.00 | 122.671 | 102.98 |
|        9         | 68.14 | 1.14 | 58.70 | 9.00 | 136.399 | 113.82 |
|        10        | 68.05 | 0.90 | 43.67 | 8.12 | 126.642 | 96.78 |

| Image / Descriptor | BRIEF | ORB | FREAK | AKAZE | SIFT |
|:----------------:|:------:|:----:|:-----:|:---:|:----:|
|        1         | 1.79 | 4.18 | 59.18 | 112.34 | 88.15 |
|        2         | 1.79 | 6.65 | 56.23 | 135.60 | 84.27 |
|        3         | 1.10 | 13.51 | 46.25 | 089.54 | 75.03 |
|        4         | 1.99 | 6.53 | 45.48 | 162.13 | 79.97 |
|        5         | 1.04 | 6.77 | 55.30 | 171.93 | 79.26 |
|        6         | 3.95 | 8.74 | 50.72 | 92.55 | 79.69 |
|        7         | 1.01 | 5.45 | 61.08 | 93.97 | 79.00|
|        8         | 1.16 | 3.95 | 73.16 | 95.49 | 79.86 |
|        9         | 1.09 | 3.86 | 59.15 | 106.19 | 80.71 |
|        10        | 0.94 | 5.23 | 67.93 | 83.49 | 80.46 |

## Choosing preferable combinations

- FAST - detects keypoints quickly, good spread
- BRIEF - fast descriptor
- based on the same aspects, ORB was also choosen

1. Detector: FAST, Descriptor: BRIEF
2. Detector: FAST, Descriptor: ORB
3. Detector: ORB, Descriptor: BRIEF
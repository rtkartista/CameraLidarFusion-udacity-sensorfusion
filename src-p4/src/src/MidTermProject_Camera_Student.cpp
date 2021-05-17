/* INCLUDES FOR THIS PROJECT */
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "dataStructures.h"
#include "matching2D.hpp"

using namespace std;

/* MAIN PROGRAM */
int main(int argc, const char *argv[])
{

    /* INIT VARIABLES AND DATA STRUCTURES */

    // data location
    string dataPath = "../";

    // camera
    string imgBasePath = dataPath + "images/";
    string imgPrefix = "KITTI/2011_09_26/image_00/data/000000"; // left camera, color
    string imgFileType = ".png";
    int imgStartIndex = 0; // first file index to load (assumes Lidar and camera names have identical naming convention)
    int imgEndIndex = 9;   // last file index to load
    int imgFillWidth = 4;  // no. of digits which make up the file index (e.g. img-0001.png)

    /* read data from command line args */

    // detector
    // SHITOMASI, HARRIS, FAST, BRISK, ORB, AKAZE, SIFT
    string detectorType = "SHITOMASI";
    if(argc >= 2)
    {
       detectorType = string(argv[1]);
    }

    // descriptor
    // BRIEF, ORB, FREAK, AKAZE, SIFT
    string descriptorType = "BRIEF";
    if(argc >= 3)
    { 
        descriptorType = string(argv[2]);
        if (descriptorType == "AKAZE" && detectorType != descriptorType) 
        {
            cout << "Using AKAZE as the detector is only compatible with AKAZE as the descriptor\nSwitching both detector and descriptor to AKAZE"<< endl;
            detectorType = "AKAZE";
            descriptorType = "AKAZE";
        }
    }

    // matcher
    // MAT_BF, MAT_FLANN
    string matcherType = "MAT_BF";
    if(argc >= 4) 
    { 
        matcherType = string(argv[3]); 
    }

    // DES_BINARY, DES_HOG
    // SIFT is a 128-dimensional 
    // continuous vector which means you need to specify the L2 norm as the distance metric in your brute-force matcher
    string descriptorContentType;
    if (argc >= 3)
    {
        if (descriptorType == "SIFT")
        {
            descriptorContentType = "DES_HOG";
        }
        else
        {
            descriptorContentType = "DES_BINARY";
        } 
    }

    // selector
    // SEL_NN, SEL_KNN
    string selectorType = "SEL_NN";
    if (argc >= 5) 
    { 
        selectorType = string(argv[4]);
    }

    // bVis
    bool bVis = 1;               
    if(argc >= 6 && string(argv[5]) == "0") 
    { 
        bVis = 0; 
    }

    // bLimitKpts
    bool bLimitKpts = 0;               
    if(argc >= 7 && string(argv[6]) == "1") 
    { 
        bLimitKpts = 1; 
    }

    // bFocusOnVehicle
    bool bFocusOnVehicle = 1;               
    if(argc >= 8 && string(argv[7]) == "0") 
    { 
        bFocusOnVehicle = 0; 
    }

    // no. of images which are held in memory (ring buffer) at the same time
    int dataBufferSize = 3;

    // list of data frames which are held in memory at the same time       
    vector<DataFrame> dataBuffer; 
    
    /* MAIN LOOP OVER ALL IMAGES */
    // change the file name before each combination
     std::ofstream myfile;
     myfile.open ("../match_HARRIS_FREAK.csv");
     myfile << "matches-count-of50, image-number\n";

    // std::ofstream myfile2;
    // myfile2.open ("../time_SIFT_detector.csv");
    // myfile2 << "Time, KEYPOINTS, image-number\n";
    double time;

    // std::ofstream myfile3;
    // myfile3.open ("../time_SIFT_descriptor.csv");
    // myfile3 << "Time, DESCRIPTOR, image-number\n";
    double time2;
    for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex++)
    {
        /* LOAD IMAGE INTO BUFFER */

        // assemble filenames for current index
        ostringstream imgNumber;
        imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex;
        string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

        // load image from file and convert to grayscale
        cv::Mat img, imgGray;
        img = cv::imread(imgFullFilename);
        cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

        //// STUDENT ASSIGNMENT
        //// TASK MP.1 -> replace the following code with ring buffer of size dataBufferSize
        // int dataBufferSize
        DataFrame frame;
        frame.cameraImg = imgGray;

        if(dataBuffer.size() < dataBufferSize)
        {
            dataBuffer.push_back(frame);
        }
        else
        {
            // poping the oldest element from the front
            if (dataBuffer.size() > 0) 
            {
                dataBuffer.erase(dataBuffer.begin());
            }

            // pushing the new element to the back
            dataBuffer.push_back(frame);
        }
        //// EOF STUDENT ASSIGNMENT
        cout << "#1 : LOAD IMAGE INTO BUFFER done" << endl;

        /* DETECT IMAGE KEYPOINTS */

        // extract 2D keypoints from current image
        vector<cv::KeyPoint> keypoints; // create empty feature list for current image

        //// STUDENT ASSIGNMENT
        //// TASK MP.2 -> add the following keypoint detectors in file matching2D.cpp and enable string-based selection based on detectorType
        //// -> HARRIS, FAST, BRISK, ORB, AKAZE, SIFT
        
        if (detectorType.compare("SHITOMASI") == 0)
        {
            detKeypointsShiTomasi(keypoints, imgGray, bVis);
        }
        else if (detectorType.compare("HARRIS") == 0)
        {
            detKeypointsHarris(keypoints, imgGray, bVis, time);
        }
        else
        {
            detKeypointsModern(keypoints, imgGray, detectorType, bVis, time);
        }
        //myfile2<<time<<","<<keypoints.size()<<","<<imgIndex<<endl;
        //// EOF STUDENT ASSIGNMENT

        //// STUDENT ASSIGNMENT
        //// TASK MP.3 -> only keep keypoints on the preceding vehicle

        // only keep keypoints on the preceding vehicle
        // Rect_ (_Tp _x, _Tp _y, _Tp _width, _Tp _height)
        cv::Rect vehicleRect(535, 180, 180, 150);
        if (bFocusOnVehicle)
        {
            keypoints.erase(remove_if(keypoints.begin(), keypoints.end(), [&vehicleRect](const cv::KeyPoint& point) 
                        { return !vehicleRect.contains(point.pt); }), keypoints.end());
    
            cout << "keypoints on the preceding vehicle are kept and rest of the data is erased" << endl;
        }
        else
        {
            cout << "bFocusOnVehicle is 'on'. Hence, we keep all the keypoints" << endl;
        }

        //// EOF STUDENT ASSIGNMENT

        // optional : limit number of keypoints (helpful for debugging and learning)
        if (bLimitKpts)
        {
            int maxKeypoints = 50;

            keypoints.erase(keypoints.begin() + maxKeypoints, keypoints.end());
            cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);

            cout << " NOTE: Keypoints have been limited!" << endl;
        }

        // push keypoints and descriptor for current frame to end of data buffer
        (dataBuffer.end() - 1)->keypoints = keypoints;
        cout << "#2 : DETECT KEYPOINTS done" << endl;

        /* EXTRACT KEYPOINT DESCRIPTORS */

        //// STUDENT ASSIGNMENT
        //// TASK MP.4 -> add the following descriptors in file matching2D.cpp and enable string-based selection based on descriptorType
        //// -> BRIEF, ORB, FREAK, AKAZE, SIFT

        cv::Mat descriptors;
        // calling the function from matching2D_Student.cpp
        descKeypoints((dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->cameraImg, descriptors, descriptorType, time2);
        //// EOF STUDENT ASSIGNMENT
        // myfile3<<time2<<","<<descriptors.size()<<","<<imgIndex<<endl;

        // push descriptors for current frame to end of data buffer
        (dataBuffer.end() - 1)->descriptors = descriptors;

        cout << "#3 : EXTRACT DESCRIPTORS done" << endl;

        if (dataBuffer.size() > 1) // wait until at least two images have been processed
        {
            /* MATCH KEYPOINT DESCRIPTORS */

            vector<cv::DMatch> matches;

            //// STUDENT ASSIGNMENT
            //// TASK MP.5 -> add FLANN matching in file matching2D.cpp
            //// TASK MP.6 -> add KNN match selection and perform descriptor distance ratio filtering with t=0.8 in file matching2D.cpp

            matchDescriptors((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints,
                             (dataBuffer.end() - 2)->descriptors, (dataBuffer.end() - 1)->descriptors,
                             matches, descriptorType, descriptorContentType, matcherType, selectorType);

            //// EOF STUDENT ASSIGNMENT

            // store matches in current data frame
            (dataBuffer.end() - 1)->kptMatches = matches;

            cout << "#4 : MATCH KEYPOINT DESCRIPTORS done" << endl;
            myfile << matches.size()<< ","<<imgIndex<<endl;

            // visualize matches between current and previous image
            if (bVis)
            {
                cv::Mat matchImg = ((dataBuffer.end() - 1)->cameraImg).clone();
                cv::drawMatches((dataBuffer.end() - 2)->cameraImg, (dataBuffer.end() - 2)->keypoints,
                                (dataBuffer.end() - 1)->cameraImg, (dataBuffer.end() - 1)->keypoints,
                                matches, matchImg,
                                cv::Scalar::all(-1), cv::Scalar::all(-1),
                                vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

                string windowName = "Matching keypoints between two camera images";
                cv::namedWindow(windowName, 7);
                cv::imshow(windowName, matchImg);
                cout << "Press key to continue to next image" << endl;
                cv::waitKey(0); // wait for key to be pressed
            }
        }

    } // eof loop over all images
    myfile.close();
     //myfile2.close();
     //myfile3.close();
    // TASK 7
    // std::ofstream myfile;
    // myfile.open ("../data_analysis.csv");
    // myfile << "Image, Method, Keypoints\n";

    // for (size_t imgIndex = 0; imgIndex <= imgEndIndex-imgStartIndex; imgIndex++)
    // {
    //     /* LOAD IMAGE INTO BUFFER */
    //     // SHITOMASI, HARRIS, FAST, BRISK, ORB, AKAZE, SIFT

    //     // assemble filenames for current index
    //     ostringstream imgNumber;
    //     imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex;
    //     string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

    //     // load image from file and convert to grayscale
    //     cv::Mat img, imgGray;
    //     img = cv::imread(imgFullFilename);
    //     cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

    //     vector<cv::KeyPoint> keypoints2,keypoints3,keypoints4,keypoints5,keypoints6,keypoints7,keypoints8; // create empty feature list for current image
    //     detKeypointsShiTomasi(keypoints2, imgGray, 0);
    //     myfile << imgFullFilename<<","<<"SHITOMASI"<<","<<keypoints2.size()<<","<<endl;
    //     detKeypointsHarris(keypoints3, imgGray, 0);
    //     myfile << imgFullFilename<<","<<"HARRIS"<<","<<keypoints3.size()<<","<<endl;
    //     detKeypointsModern(keypoints4, imgGray, "FAST", 0);
    //     myfile << imgFullFilename<<","<<"FAST"<<","<<keypoints4.size()<<","<<endl;
    //     detKeypointsModern(keypoints5, imgGray, "ORB", 0);
    //     myfile << imgFullFilename<<","<<"ORB"<<","<<keypoints5.size()<<","<<endl;
    //     detKeypointsModern(keypoints6, imgGray, "AKAZE", 0);
    //     myfile << imgFullFilename<<","<<"AKAZE"<<","<<keypoints6.size()<<","<<endl;
    //     detKeypointsModern(keypoints7, imgGray, "BRISK", 0);
    //     myfile << imgFullFilename<<","<<"BRISK"<<","<<keypoints7.size()<<","<<endl;
    //     detKeypointsModern(keypoints8, imgGray, "SIFT", 0);
    //     myfile << imgFullFilename<<","<<"SIFT"<<","<<keypoints8.size()<<","<<endl;
    //     myfile <<endl<<"Next image"<<endl;

    // }
    // myfile.close();
    return 0;
}

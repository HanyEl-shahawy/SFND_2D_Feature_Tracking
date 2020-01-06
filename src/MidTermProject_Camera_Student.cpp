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

#include <deque>

using namespace std;

namespace algs
{
    namespace detectors
    {
        string brisk = "BRISK";
        string shi_tomas = "SHITOMASI";
        string fast = "FAST";
        string orb = "ORB";
        string akaze = "AKAZE";
        string sift = "SIFT";
        string harris = "HARRIS";
    }
    namespace descriptors
    {
        namespace types
        {
            string binary = "DES_BINARY";
            string hog = "DES_HOG";
        }
        string brisk = "BRISK";
        string brief = "BRIEF";
        string orb = "ORB";
        string freak = "FREAK";
        string akaze = "AKAZE";
        string sift = "SIFT";
    }
    namespace matchers
    {
        string mat_bf = "MAT_BF";
        string mat_flann = "MAT_FLANN";
    }
    namespace selectors
    {
        string sel_nn = "SEL_NN";
        string sel_knn = "SEL_KNN";
    }
}

/* MAIN PROGRAM */
int main(int argc, const char *argv[])
{
    /* INIT VARIABLES AND DATA STRUCTURES */
    vector<string> detectorsArray{algs::detectors::sift,algs::detectors::harris,
                             algs::detectors::shi_tomas, algs::detectors::fast,
                             algs::detectors::brisk, algs::detectors::akaze,
                             algs::detectors::orb};
    vector<string> describtorsArray{algs::descriptors::orb, algs::descriptors::akaze,
                               algs::descriptors::brisk, algs::descriptors::sift,
                               algs::descriptors::brief, algs::descriptors::freak};

//    string detectorType = algs::detectors::sift;
//    string descriptor = algs::descriptors::sift; // BRIEF, ORB, FREAK, AKAZE, SIFT

    string descriptorType = algs::descriptors::types::binary; // DES_BINARY, DES_HOG


    string matcherType = algs::matchers::mat_bf;        // MAT_BF, MAT_FLANN
    string selectorType = algs::selectors::sel_knn;       // SEL_NN, SEL_KNN

    // data location
    string dataPath = "./";
    // camera
    string imgBasePath = dataPath + "images/";
    string imgPrefix = "KITTI/2011_09_26/image_00/data/000000"; // left camera, color
    string imgFileType = ".png";
    int imgStartIndex = 0; // first file index to load (assumes Lidar and camera names have identical naming convention)
    int imgEndIndex = 9;   // last file index to load
    int imgFillWidth = 4;  // no. of digits which make up the file index (e.g. img-0001.png)

    // misc
    int dataBufferSize = 2;       // no. of images which are held in memory (ring buffer) at the same time


    for (const auto& detectorType : detectorsArray)
    {
        for (const auto& describtor : describtorsArray)
        {
            std::cout << detectorType << " with " << describtor << "\n";
            try
            {
                auto t = (double)cv::getTickCount();
                if (describtor == algs::descriptors::sift)
                {
                    descriptorType = algs::descriptors::types::hog;
                }
                deque<DataFrame> dataBuffer{}; // list of data frames which are held in memory at the same time
                for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex++)
                {
                    std::cout << "*********************** " << imgIndex << "*********************\n";

                    ostringstream imgNumber;
                    imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex;
                    string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;
                    cv::Mat img, imgGray;
                    img = cv::imread(imgFullFilename);
                    cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);
                    DataFrame frame;
                    frame.cameraImg = imgGray;
                    if (dataBuffer.size() == dataBufferSize)
                    {
                        dataBuffer.pop_front();
                    }
                    dataBuffer.push_back(frame);
                    vector<cv::KeyPoint> keypoints; // create empty feature list for current image
                    if (detectorType.compare("SHITOMASI") == 0)
                    {
                        detKeypointsShiTomasi(keypoints, imgGray, false);
                    }
                    else if(detectorType.compare("HARRIS") == 0)
                    {
                        detKeypointsHarris(keypoints, imgGray, false);
                    }
                    else
                    {
                        detKeypointsModern(keypoints, imgGray, detectorType, false);
                    }
                    bool bFocusOnVehicle = true;
                    cv::Rect vehicleRect(535, 180, 180, 150);
                    if (bFocusOnVehicle)
                    {
                        auto it{std::remove_if(keypoints.begin(), keypoints.end(), [&vehicleRect](const cv::KeyPoint& kp)
                        {
                            return !(vehicleRect.contains(kp.pt));
                        })};
                        keypoints.erase(it, keypoints.end());
                    }
                    std::cout << detectorType << " gives number of keypoints: " << keypoints.size() << "\n";
//                    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
//                    cout << " detector time taken is: " << 1000 * t / 1.0 << " ms" << endl;
//                    t = (double)cv::getTickCount();
                    (dataBuffer.end() - 1)->keypoints = keypoints;
                    cv::Mat descriptors;
                    descKeypoints((dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->cameraImg, descriptors, describtor);
                    (dataBuffer.end() - 1)->descriptors = descriptors;
                    if (dataBuffer.size() > 1) // wait until at least two images have been processed
                    {
                        vector<cv::DMatch> matches;
                        matchDescriptors((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints,
                                         (dataBuffer.end() - 2)->descriptors, (dataBuffer.end() - 1)->descriptors,
                                         matches, descriptorType, matcherType, selectorType);
                        (dataBuffer.end() - 1)->kptMatches = matches;
                        std::cout << " number of matches: " << matches.size() << "\n";
                    }
                }
                t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
                cout << " time taken is: " << 1000 * t / 1.0 << " ms" << endl;
            }
            catch(...)
            {
                std::cout << detectorType << " does not work with " << describtor << "\n";
                std::cout << " ------------------------------------------------" << "\n";
                continue;
            }
    }
    } // eof loop over all images

    return 0;
}

# SFND 3D Object Tracking

Welcome to the final project of the camera course. By completing all the lessons, you now have a solid understanding of keypoint detectors, descriptors, and methods to match them between successive images. Also, you know how to detect objects in an image using the YOLO deep-learning framework. And finally, you know how to associate regions in a camera image with Lidar points in 3D space. Let's take a look at our program schematic to see what we already have accomplished and what's still missing.

<img src="images/course_code_structure.png" width="779" height="414" />

In this final project, you will implement the missing parts in the schematic. To do this, you will complete four major tasks: 
1. First, you will develop a way to match 3D objects over time by using keypoint correspondences. 
2. Second, you will compute the TTC based on Lidar measurements. 
3. You will then proceed to do the same using the camera, which requires to first associate keypoint matches to regions of interest and then to compute the TTC based on those matches. 
4. And lastly, you will conduct various tests with the framework. Your goal is to identify the most suitable detector/descriptor combination for TTC estimation and also to search for problems that can lead to faulty measurements by the camera or Lidar sensor. In the last course of this Nanodegree, you will learn about the Kalman filter, which is a great way to combine the two independent TTC measurements into an improved version which is much more reliable than a single sensor alone can be. But before we think about such things, let us focus on your final project in the camera course. 

## Dependencies for Running Locally
* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* Git LFS
  * Weight files are handled using [LFS](https://git-lfs.github.com/)
* OpenCV >= 4.1
  * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
  * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level project directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./3D_object_tracking`.

## Rubric

### 1. Match 3D Objects

**Criteria:**
Implement the method "matchBoundingBoxes", which takes as input both the previous and the current data frames and provides as output the ids of the matched regions of interest (i.e. the boxID property). Matches must be the ones with the highest number of keypoint correspondences.

**Solution:**
Construct `prev_curr_box_score` which records the frequency / score of box pair. Iterate matched key points. If the matched points belong to the bounding boxs, then the corresponding boxs score increse by one. Last, for each quiry box, select the train box id which has the biggest score.

```
void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{

    int prev_box_size = prevFrame.boundingBoxes.size();
    int curr_box_size = currFrame.boundingBoxes.size();
    int prev_curr_box_score[prev_box_size][curr_box_size] = {};

    // iterate pnt matchs, cnt box-box match score
    for (auto it = matches.begin(); it != matches.end() - 1; it++)
    {
        // prev pnt
        cv::KeyPoint prev_key_pnt = prevFrame.keypoints[it->queryIdx];
        cv::Point prev_pnt = cv::Point(prev_key_pnt.pt.x, prev_key_pnt.pt.y);

        // curr pnt
        cv::KeyPoint curr_key_pnt = currFrame.keypoints[it->trainIdx];
        cv::Point curr_pnt = cv::Point(curr_key_pnt.pt.x, curr_key_pnt.pt.y);

        // get corresponding box with the point
        std::vector<int> prev_box_id_list, curr_box_id_list;
        for (int i = 0; i < prev_box_size; ++i)
        {
            if (prevFrame.boundingBoxes[i].roi.contains(prev_pnt))
            {
                prev_box_id_list.push_back(i);
            }
        }
        for (int j = 0; j < curr_box_size; ++j)
        {
            if (currFrame.boundingBoxes[j].roi.contains(curr_pnt))
            {
                curr_box_id_list.push_back(j);
            }
        }

        // add cnt to prev_curr_box_score
        for (int i: prev_box_id_list)
        {
            for (int j: curr_box_id_list)
            {
                prev_curr_box_score[i][j] += 1;
            }
        }
    } // END OF THE PNT MATCH

    // for each box in prevFrame, find the box with highest score in currFrame
    for (int i = 0; i < prev_box_size; ++i) 
    {
        int max_score = 0;
        int best_idx = 0;
        
        for (int j = 0; j < curr_box_size; ++j)
        {
            if (prev_curr_box_score[i][j] > max_score)
            {
                max_score = prev_curr_box_score[i][j];
                best_idx = j;
            }
        }

        bbBestMatches[i] = best_idx;
    }
}
```

### 2. Compute Lidar-based TTC

**Criteria:**
Compute the time-to-collision in second for all matched 3D objects using only Lidar measurements from the matched bounding boxes between current and previous frame.

**Solution:**

### 3. Associate Keypoint Correspondences with Bounding Boxes

**Criteria:**
Prepare the TTC computation based on camera measurements by associating keypoint correspondences to the bounding boxes which enclose them. All matches which satisfy this condition must be added to a vector in the respective bounding box.


**Solution:**

### 4. Compute Camera-based TTC

**Criteria:**
Compute the time-to-collision in second for all matched 3D objects using only keypoint correspondences from the matched bounding boxes between current and previous frame.

**Solution:**

### 5. Performance Evaluation 1

**Criteria:**
Find examples where the TTC estimate of the Lidar sensor does not seem plausible. Describe your observations and provide a sound argumentation why you think this happened.

**Solution:**

### 6. Performance Evaluation 2

**Criteria:**
Run several detector / descriptor combinations and look at the differences in TTC estimation. Find out which methods perform best and also include several examples where camera-based TTC estimation is way off. As with Lidar, describe your observations again and also look into potential reasons.

**Solution:**

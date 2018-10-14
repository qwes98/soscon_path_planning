//
//  coveragepathplanning.hpp
//  CPP
//
//  Created by 欧阳伟斌 on 16/10/17.
//  Copyright © 2016年 欧阳伟斌. All rights reserved.
//

#ifndef coveragepathplanning_hpp
#define coveragepathplanning_hpp

#include <deque>
#include "opencv2/opencv.hpp"

// strategy macro
//#define WAVEFRONT_CONNECT_8 "_Wavefront-connect-8"
//#define PLANNING_CONNECT_8 "_Planning-connect-8"
//#define PLANNING_SOLID_BEGINNING "_Planning-solid-beginning"
#ifndef PLANNING_SOLID_BEGINNING
//#define PLANNING_ONE_ACCUMULATE_BEGINNING "_Planning-one-accumulate-beginning"
#endif

namespace cpp {
int CoveragePathPlanning(
    const cv::Mat &_binary, const cv::Point &_start, const cv::Point &_goal,
    std::deque<cv::Point> &_path, int _radius);
int ZigZagPathPlanning(const cv::Mat &_binary, const cv::Point &_tar, const cv::Point &_goal,
	std::deque<cv::Point> &_path, int _radius);
}

#endif /* coveragepathplanning_hpp */

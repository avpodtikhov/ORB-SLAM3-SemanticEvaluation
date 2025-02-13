/**
* This file is part of ORB-LINE-SLAM
*
* Copyright (C) 2020-2021 John Alamanos, National Technical University of Athens.
* Copyright (C) 2016-2018, Ruben Gomez-Ojeda, University of Malaga.
* Copyright (C) 2016-2018, David Zuñiga-Noël, University of Malaga.         
* Copyright (C) 2016-2018, MAPIR group, University of Malaga.    
*
* ORB-LINE-SLAM is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-LINE-SLAM is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-LINE-SLAM.
* If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

//STL
#include <utility>
#include <vector>

//OpenCV
#include <opencv2/core/core.hpp>

#include "gridStructure.h"
#include "Frame.h"
#include "MapPoint.h"
#include "MapLine.h"
#include "KeyFrame.h"

#include "Eigen/Core"
#include "sophus/se3.hpp"


namespace ORB_SLAM3 {

    class Frame;
    class MapPoint;
    class MapLine;

    typedef std::pair<int, int> point_2d;
    typedef std::pair<point_2d, point_2d> line_2d;

    inline float dot(const std::pair<float, float> &a, const std::pair<float, float> &b) {
        return (a.first*b.first + a.second*b.second);
    }

    inline void normalize(std::pair<float, float> &v) {
        float magnitude = std::sqrt(dot(v, v));

        v.first /= magnitude;
        v.second /= magnitude;
    }

    class LineMatcher
    {
    public:
        LineMatcher(float nnratio = 0.9, float th = 3.0, float angth = M_PI/8.0);

        int match(KeyFrame *pKF, Frame &F, vector<MapLine *> &vpMapLineMatches, const bool bestLRMatches);

        int match(const std::vector<MapLine*> &mvpLocalMapLines, Frame &CurrentFrame, std::vector<int> &matches_12);

        int match(const cv::Mat &desc1, const cv::Mat &desc2, std::vector<int> &matches_12);

        int matchGrid(const std::vector<line_2d> &lines1, const cv::Mat &desc1, const GridStructure &grid, const cv::Mat &desc2, const std::vector<std::pair<float, float>> &directions2, const GridWindow &w, std::vector<int> &matches_12);

        int SearchByProjection(Frame &CurrentFrame, Frame &LastFrame, const GridStructure &grid);

    public:
        static const int TH_HIGH;

    protected:
        int matchNNR(const cv::Mat &desc1, const cv::Mat &desc2, std::vector<int> &matches_12);

        int distance(const cv::Mat &a, const cv::Mat &b);

        float mfNNratio;
        float mTh;
        float mAngTh;
    };

} // namesapce ORB_SLAM3

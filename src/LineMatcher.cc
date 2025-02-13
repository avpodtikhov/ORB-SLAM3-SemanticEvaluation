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

#include "LineMatcher.h"

//STL
#include <cmath>
#include <functional>
#include <future>
#include <limits>
#include <stdexcept>

//OpenCV
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

#include "gridStructure.h"
#include "Converter.h"

namespace ORB_SLAM3 {
    const int LineMatcher::TH_HIGH = 120;

    LineMatcher::LineMatcher(float nnratio, float th, float angth) : mfNNratio(nnratio), mTh(th), mAngTh(angth)
    {
    }

    int LineMatcher::matchNNR(const cv::Mat &desc1, const cv::Mat &desc2, std::vector<int> &matches_12) {

        int matches = 0;
        matches_12.resize(desc1.rows, -1);

        std::vector<std::vector<cv::DMatch>> matches_;
        cv::Ptr<cv::BFMatcher> bfm = cv::BFMatcher::create(cv::NORM_HAMMING, false); // cross-check
        bfm->knnMatch(desc1, desc2, matches_, 2);

        if (desc1.rows != matches_.size())
            throw std::runtime_error("[matchNNR] Different size for matches and descriptors!");

        for (int idx = 0, nsize = desc1.rows; idx < nsize; ++idx) {
            if (matches_[idx][0].distance < matches_[idx][1].distance * mfNNratio) {
                matches_12[idx] = matches_[idx][0].trainIdx;
                matches++;
            }
        }

        return matches;
    }

    // int LineMatcher::match(const std::vector<MapLine*> &vpLocalMapLines, Frame &CurrentFrame, std::vector<int> &matches_12)
    // {

    //     cv::Mat desc1;
    //     desc1.reserve(vpLocalMapLines.size());
    //     for(int i=0,z=vpLocalMapLines.size(); i<z; ++i)
    //         desc1.push_back(vpLocalMapLines[i]->GetDescriptor());
    //     cv::Mat desc2;
    //     CurrentFrame.mDescriptorsLine.copyTo(desc2);

    //     bool bestLRMatches = true; // true if double-checking the matches between the two images
    //     int matches;
    //     if (bestLRMatches) {
    //         std::vector<int> matches_21;
    //         matches = matchNNR(desc1, desc2, matches_12);
    //         matchNNR(desc2, desc1, matches_21);
    //         for (int i1 = 0, nsize = matches_12.size(); i1 < nsize; ++i1) {
    //             int &i2 = matches_12[i1];
    //             if (i2 >= 0 && matches_21[i2] != i1) {
    //                 i2 = -1;
    //                 matches--;
    //             }
    //         }
    //     }
    //     else matches = matchNNR(desc1, desc2, matches_12);

    //     return matches;
    // }

    int LineMatcher::match(KeyFrame *pKF, Frame &F, vector<MapLine *> &vpMapLineMatches, const bool bestLRMatches)
    {
        const vector<MapLine *> vpMapLinesKF = pKF->GetMapLineMatches();
        vpMapLineMatches = vector<MapLine *>(F.N_Lines, static_cast<MapLine *>(NULL));

        std::vector<int> matches_12;
        int matches = matchNNR(pKF->mDescriptorsLine, F.mDescriptorsLine, matches_12);

        if (bestLRMatches) 
        {
            std::vector<int> matches_21;
            matchNNR(F.mDescriptorsLine, pKF->mDescriptorsLine, matches_21);

            for (int i1 = 0; i1 < matches_12.size(); ++i1) {
                int &i2 = matches_12[i1];
                if (i2 < 0 || matches_21[i2] != i1 || 
                    (F.mvpMapLines[i2] && F.mvpMapLines[i2]->Observations() > 0)) {
                    matches--;
                    continue;
                }
                vpMapLineMatches[i1] = vpMapLinesKF[i2];
            }
        } 
        else 
        {
            for (int i1 = 0; i1 < matches_12.size(); ++i1) {
                int &i2 = matches_12[i1];
                if (i2 < 0 || (F.mvpMapLines[i2] && F.mvpMapLines[i2]->Observations() > 0)) {
                    matches--;
                    continue;
                }
                vpMapLineMatches[i1] = vpMapLinesKF[i2];
            }
        }

        return matches;
    }

    int LineMatcher::SearchByProjection(Frame &CurrentFrame, Frame &LastFrame, const bool bestLRMatches) {

        std::vector<int> matches_12;
        int matches = matchNNR(LastFrame.mDescriptorsLine, CurrentFrame.mDescriptorsLine, matches_12);

        const double deltaWidth = (CurrentFrame.mnMaxX-CurrentFrame.mnMinX)*0.1;
        const double deltaHeight = (CurrentFrame.mnMaxY-CurrentFrame.mnMinY)*0.1;

        const Sophus::SE3f Tcw = CurrentFrame.GetPose();
        const Eigen::Vector3f twc = Tcw.inverse().translation();

        auto checkMatch = [&](int i1, int i2) -> bool {
            // Project In Image
            const Eigen::Vector3f x3Dw_sp = LastFrame.mvpMapLines[i1]->GetWorldPos().head(3);
            const Eigen::Vector3f x3Dw_ep = LastFrame.mvpMapLines[i1]->GetWorldPos().tail(3);

            Eigen::Vector3f x3Dc_sp = Tcw * x3Dw_sp + twc;
            Eigen::Vector3f x3Dc_ep = Tcw * x3Dw_ep + twc;

            const float invzc_sp = 1.0f / x3Dc_sp(2);
            const float invzc_ep = 1.0f / x3Dc_ep(2);

            if(invzc_sp < 0 || invzc_ep < 0) {
                return false;
            }

            Eigen::Vector2f uv_sp = CurrentFrame.mpCamera->project(x3Dc_sp);
            Eigen::Vector2f uv_ep = CurrentFrame.mpCamera->project(x3Dc_ep);
            
            if(uv_sp.x() < CurrentFrame.mnMinX || uv_sp.x() > CurrentFrame.mnMaxX || 
               uv_ep.x() < CurrentFrame.mnMinX || uv_ep.x() > CurrentFrame.mnMaxX ||
               uv_sp.y() < CurrentFrame.mnMinY || uv_sp.y() > CurrentFrame.mnMaxY || 
               uv_ep.y() < CurrentFrame.mnMinY || uv_ep.y() > CurrentFrame.mnMaxY)
                return false;
            
            // Check for orientation
            float theta = CurrentFrame.mvKeysUnLine[i2].angle - atan2(uv_ep.y() - uv_sp.y(), uv_ep.x() - uv_sp.x());

            if (theta < -M_PI) theta += 2*M_PI;
            else if (theta > M_PI) theta -= 2*M_PI;
            if(fabs(theta) > mAngTh)
                return false;

            // check for position in image
            const auto& currLine = CurrentFrame.mvKeysUnLine[i2];
            if(fabs(currLine.startPointX - uv_sp.x()) > deltaWidth || 
               fabs(currLine.endPointX - uv_ep.x()) > deltaWidth || 
               fabs(currLine.startPointY - uv_sp.y()) > deltaHeight || 
               fabs(currLine.endPointY - uv_ep.y()) > deltaHeight)
                return false;

            return true;
        };

        if (bestLRMatches) {
            std::vector<int> matches_21;
            matchNNR(CurrentFrame.mDescriptorsLine, LastFrame.mDescriptorsLine, matches_21);

            for (int i1 = 0; i1 < matches_12.size(); ++i1) {
                int &i2 = matches_12[i1];
                if (i2 < 0 || matches_21[i2] != i1 || 
                    (CurrentFrame.mvpMapLines[i2] && CurrentFrame.mvpMapLines[i2]->Observations() > 0)) {
                    matches--;
                    continue;
                }

                if (!checkMatch(i1, i2)) {
                    matches--;
                    continue;
                }
                
                CurrentFrame.mvpMapLines[i2] = LastFrame.mvpMapLines[i1];
            }
        } else {
            for (int i1 = 0; i1 < matches_12.size(); ++i1) {
                int &i2 = matches_12[i1];
                if (i2 < 0 || (CurrentFrame.mvpMapLines[i2] && CurrentFrame.mvpMapLines[i2]->Observations() > 0)) {
                    matches--;
                    continue;
                }

                if (!checkMatch(i1, i2)) {
                    matches--;
                    continue;
                }
                
                CurrentFrame.mvpMapLines[i2] = LastFrame.mvpMapLines[i1];
            }
        }

        return matches;
    }

    int LineMatcher::SearchByProjection(Frame &CurrentFrame, vector<MapLine*> &vpMapLines, const bool bestLRMatches) {

        cv::Mat desc1;
        desc1.reserve(vpMapLines.size());

        for (int i = 0,z = vpMapLines.size(); i < z; ++i)
            desc1.push_back(vpMapLines[i]->GetDescriptor());


        std::vector<int> matches_12;
        int matches = matchNNR(desc1, CurrentFrame.mDescriptorsLine, matches_12);

        const double deltaWidth = (CurrentFrame.mnMaxX-CurrentFrame.mnMinX)*0.1;
        const double deltaHeight = (CurrentFrame.mnMaxY-CurrentFrame.mnMinY)*0.1;

        const Sophus::SE3f Tcw = CurrentFrame.GetPose();
        const Eigen::Vector3f twc = Tcw.inverse().translation();


        auto checkMatch = [&](int i1, int i2) -> bool {
            MapLine* pML = vpMapLines[i1];
            float theta = CurrentFrame.mvKeysUnLine[i2].angle - pML->mnTrackangle;

            if (theta < -M_PI) theta += 2 * M_PI;
            else if (theta > M_PI) theta -= 2 * M_PI;

            if (fabs(theta) > mAngTh)
                return false;

            // check for position in image
            const float sX_curr = CurrentFrame.mvKeysUnLine[i2].startPointX;
            const float sX_last = pML->mTrackProjsX;
            const float sY_curr = CurrentFrame.mvKeysUnLine[i2].startPointY;
            const float sY_last = pML->mTrackProjsY;
            const float eX_curr = CurrentFrame.mvKeysUnLine[i2].endPointX;
            const float eX_last = pML->mTrackProjeX;
            const float eY_curr = CurrentFrame.mvKeysUnLine[i2].endPointY;
            const float eY_last = pML->mTrackProjeY;

            if(fabs(sX_curr - sX_last) > deltaWidth || 
               fabs(eX_curr - eX_last) > deltaWidth || 
               fabs(sY_curr - sY_last) > deltaHeight || 
               fabs(eY_curr - eY_last) > deltaHeight)
                return false;

            return true;
        };

        if (bestLRMatches) {
            std::vector<int> matches_21;
            matchNNR(CurrentFrame.mDescriptorsLine, desc1, matches_21);

            for (int i1 = 0; i1 < matches_12.size(); ++i1) {
                int &i2 = matches_12[i1];
                if (i2 < 0 || matches_21[i2] != i1 || 
                    (CurrentFrame.mvpMapLines[i2] && CurrentFrame.mvpMapLines[i2]->Observations() > 0)) {
                    matches--;
                    continue;
                }

                if (!checkMatch(i1, i2)) {
                    matches--;
                    continue;
                }
                
                CurrentFrame.mvpMapLines[i2] = vpMapLines[i1];
            }
        } else {
            for (int i1 = 0; i1 < matches_12.size(); ++i1) {
                int &i2 = matches_12[i1];
                if (i2 < 0 || (CurrentFrame.mvpMapLines[i2] && CurrentFrame.mvpMapLines[i2]->Observations() > 0)) {
                    matches--;
                    continue;
                }

                if (!checkMatch(i1, i2)) {
                    matches--;
                    continue;
                }
                
                CurrentFrame.mvpMapLines[i2] = vpMapLines[i1];
            }
        }

        return matches;
    }

    int LineMatcher::distance(const cv::Mat &a, const cv::Mat &b) {

        // adapted from: http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel

        const int *pa = a.ptr<int32_t>();
        const int *pb = b.ptr<int32_t>();

        int dist = 0;
        for(int i = 0; i < 8; i++, pa++, pb++) {
            unsigned  int v = *pa ^ *pb;
            v = v - ((v >> 1) & 0x55555555);
            v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
            dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
        }

        return dist;
    }

    // int LineMatcher::match(const cv::Mat &desc1, const cv::Mat &desc2, std::vector<int> &matches_12) {

    //     bool bestLRMatches = true; // true if double-checking the matches between the two images
    //     if (bestLRMatches) {
    //         int matches;
    //         std::vector<int> matches_21;
    //         matches = matchNNR(desc1, desc2, matches_12);
    //         matchNNR(desc2, desc1, matches_21);
    //         for (int i1 = 0, nsize = matches_12.size(); i1 < nsize; ++i1) {
    //             int &i2 = matches_12[i1];
    //             if (i2 >= 0 && matches_21[i2] != i1) {
    //                 i2 = -1;
    //                 matches--;
    //             }
    //         }

    //         return matches;
    //     } else
    //             return matchNNR(desc1, desc2, matches_12);
    // }


    int LineMatcher::matchGrid(const std::vector<line_2d> &lines1, const cv::Mat &desc1,
                const GridStructure &grid, const cv::Mat &desc2, const std::vector<std::pair<float, float>> &directions2,
                const GridWindow &w,
                std::vector<int> &matches_12) {

        if (lines1.size() != desc1.rows)
            throw std::runtime_error("[matchGrid] Each line needs a corresponding descriptor!");

        float lineSimTh = 0.75f;   // threshold for cosine similarity
        float minRatio12L = 0.9f;  // parameter to avoid outliers in line matching
        bool bestLRMatches = true; // true if double-checking the matches between the two images
        int matches = 0;
        matches_12.resize(desc1.rows, -1);

        int best_d, best_d2, best_idx;
        std::vector<int> matches_21, distances;
        if (bestLRMatches) {
            matches_21.resize(desc2.rows, -1);
            distances.resize(desc2.rows, std::numeric_limits<int>::max());
        }

        for (int i1 = 0, nsize = lines1.size(); i1 < nsize; ++i1) {

            best_d = std::numeric_limits<int>::max();
            best_d2 = std::numeric_limits<int>::max();
            best_idx = -1;

            const line_2d &coords = lines1[i1];
            cv::Mat desc = desc1.row(i1);

            const point_2d sp = coords.first;
            const point_2d ep = coords.second;

            std::pair<float, float> v = std::make_pair(ep.first - sp.first, ep.second - sp.second);
            normalize(v);

            std::unordered_set<int> candidates;
            grid.get(sp.first, sp.second, w, candidates);
            grid.get(ep.first, ep.second, w, candidates);

            if (candidates.empty()) continue;
            for (const int &i2 : candidates) {
                if (i2 < 0 || i2 >= desc2.rows) continue;

                if (std::abs(dot(v, directions2[i2])) < lineSimTh)
                    continue;

                const int d = distance(desc, desc2.row(i2));

                if (bestLRMatches) {
                    if (d < distances[i2]) {
                        distances[i2] = d;
                        matches_21[i2] = i1;
                    } else continue;
                }

                if (d < best_d) {
                    best_d2 = best_d;
                    best_d = d;
                    best_idx = i2;
                } else if (d < best_d2)
                    best_d2 = d;
            }

            if (best_d < best_d2 * minRatio12L) {
                matches_12[i1] = best_idx;
                matches++;
            }
        }

        if (bestLRMatches) {
            for (int i1 = 0, nsize = matches_12.size(); i1 < nsize; ++i1) {
                int &i2 = matches_12[i1];
                if (i2 >= 0 && matches_21[i2] != i1) {
                    i2 = -1;
                    matches--;
                }
            }
        }

        return matches;
    }

    // int LineMatcher::SearchByProjection(Frame &CurrentFrame, Frame &LastFrame, const GridStructure &grid)
    // {
    //     int matches = 0;

    //     const Sophus::SE3f Tcw = CurrentFrame.GetPose();
    //     const Eigen::Matrix3f Rcw = Tcw.rotationMatrix();
    //     const Eigen::Vector3f tcw = Tcw.translation();

    //     for(int i=0; i<LastFrame.N_Lines; i++)
    //     {
    //         MapLine* pML = LastFrame.mvpMapLines[i];
    //         if(pML)
    //         {
    //             if(!LastFrame.mvbOutlierLine[i])
    //             {
    //                 // Project Lines to the Image
    //                 Eigen::Vector3f x3Dw_sp = pML->GetWorldPos().head(3);
    //                 Eigen::Vector3f x3Dw_ep = pML->GetWorldPos().tail(3);
    //                 Eigen::Vector3f x3Dc_sp = Rcw * x3Dw_sp +tcw;
    //                 Eigen::Vector3f x3Dc_ep = Rcw * x3Dw_ep +tcw;

    //                 const float invzc_sp = 1.0 / x3Dc_sp(2);
    //                 const float invzc_ep = 1.0 / x3Dc_ep(2);

    //                 if(invzc_sp<0 || invzc_ep<0)
    //                     continue;

    //                 Eigen::Vector2f uv_sp = CurrentFrame.mpCamera->project(x3Dc_sp);
    //                 Eigen::Vector2f uv_ep = CurrentFrame.mpCamera->project(x3Dc_ep);

    //                 if(uv_sp.x() < CurrentFrame.mnMinX || uv_sp.x() > CurrentFrame.mnMaxX || uv_ep.x() < CurrentFrame.mnMinX || uv_ep.x() > CurrentFrame.mnMaxX)
    //                     continue;
    //                 if(uv_sp.y() < CurrentFrame.mnMinY || uv_sp.y() > CurrentFrame.mnMaxY || uv_ep.y() < CurrentFrame.mnMinY || uv_ep.y() > CurrentFrame.mnMaxY)
    //                     continue;

    //                 int nLastOctave = LastFrame.mvKeysLine[i].octave;

    //                 // Search in a window. Size depends on scale
    //                 int window = floor(mTh);

    //                 if(CurrentFrame.mvScaleFactorsLine[nLastOctave]>1)
    //                     window = floor(mTh+CurrentFrame.mvScaleFactorsLine[nLastOctave]);

    //                 GridWindow win;
    //                 win.width = std::make_pair(window, window);
    //                 win.height = std::make_pair(window, window);

    //                 const line_2d coords = std::make_pair(std::make_pair(uv_sp.x() * CurrentFrame.inv_width, uv_sp.y() * CurrentFrame.inv_height),
    //                                         std::make_pair(uv_ep.x() * CurrentFrame.inv_width, uv_ep.y() * CurrentFrame.inv_height)); 

    //                 const point_2d spoint = coords.first;
    //                 const point_2d epoint = coords.second;

    //                 std::unordered_set<int> candidates;
    //                 grid.get(spoint.first, spoint.second, win, candidates);
    //                 grid.get(epoint.first, epoint.second, win, candidates);

    //                 if (candidates.empty()) continue;

    //                 const cv::Mat dML = pML->GetDescriptor();

    //                 int bestDist = 256;
    //                 int bestidx = -1;

    //                 for (const int &i2 : candidates) 
    //                 {
    //                     if(CurrentFrame.mvpMapLines[i2])
    //                         if(CurrentFrame.mvpMapLines[i2]->Observations()>0)
    //                             continue;

    //                     const int dist = distance(dML, CurrentFrame.mDescriptorsLine.row(i2));

    //                     if(dist<bestDist)
    //                     {
    //                         bestDist=dist;
    //                         bestidx=i2;
    //                     }
    //                 }

    //                 if(bestDist<=TH_HIGH)
    //                 {
    //                     float theta = CurrentFrame.mvKeysUnLine[bestidx].angle-atan2(uv_ep.y() - uv_sp.y(), uv_ep.x() - uv_sp.x());;
    //                     if(theta<-M_PI) theta+=2*M_PI;
    //                     else if(theta>M_PI) theta-=2*M_PI;
    //                     if(fabs(theta)<mAngTh)
    //                     {
    //                         CurrentFrame.mvpMapLines[bestidx]=pML;
    //                         matches++;
    //                     }
    //                 }

    //             }
    //         }
    //     }
    //     return matches;
    // }

} //namesapce ORB_SLAM3

/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 * Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
 * the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with ORB-SLAM3.
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include "FrameDrawer.h"
#include "Tracking.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <mutex>

namespace ORB_SLAM3
{

    FrameDrawer::FrameDrawer(Atlas *pAtlas) : both(false), mpAtlas(pAtlas)
    {
        mState = Tracking::SYSTEM_NOT_READY;
        mIm = cv::Mat(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
        mImRight = cv::Mat(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
    }

    cv::Mat FrameDrawer::DrawFrame(float imageScale)
    {
        cv::Mat im;
        vector<cv::KeyPoint> vIniKeys;     // Initialization: KeyPoints in reference frame
        vector<int> vMatches;              // Initialization: correspondeces with reference keypoints
        vector<cv::KeyPoint> vCurrentKeys; // KeyPoints in current frame
        vector<bool> vbVO, vbMap;          // Tracked MapPoints in current frame
        vector<pair<cv::Point2f, cv::Point2f>> vTracks;
        int state; // Tracking state
        vector<float> vCurrentDepth;
        float thDepth;

        Frame currentFrame;
        vector<MapPoint *> vpLocalMap;
        vector<cv::KeyPoint> vMatchesKeys;
        vector<MapPoint *> vpMatchedMPs;
        vector<cv::KeyPoint> vOutlierKeys;
        vector<MapPoint *> vpOutlierMPs;
        vector<bool> vCurrentKeysMoving;
        vector<int> vCurrentKeysSemantic;
        vector<int> vCurrentMapSemantic;

        map<long unsigned int, cv::Point2f> mProjectPoints;
        map<long unsigned int, cv::Point2f> mMatchedInImage;

        cv::Scalar standardColor(0, 255, 0);
        cv::Scalar odometryColor(255, 0, 0);

        cv::Scalar color0(0, 0, 0);
        cv::Scalar color1(70, 70, 70);
        cv::Scalar color2(100, 40, 40);
        cv::Scalar color3(55, 90, 80);
        cv::Scalar color4(220, 20, 60);
        cv::Scalar color5(153, 153, 153);
        cv::Scalar color6(157, 234, 50);
        cv::Scalar color7(128, 64, 128);
        cv::Scalar color8(244, 35, 232);
        cv::Scalar color9(107, 142, 35);
        cv::Scalar color10(0, 0, 142);
        cv::Scalar color11(102, 102, 156);
        cv::Scalar color12(220, 220, 0);
        cv::Scalar color13(70, 130, 180);
        cv::Scalar color14(81, 0, 81);
        cv::Scalar color15(150, 100, 100);
        cv::Scalar color16(230, 150, 140);
        cv::Scalar color17(180, 165, 180);
        cv::Scalar color18(250, 170, 30);
        cv::Scalar color19(110, 190, 160);
        cv::Scalar color20(170, 120, 50);
        cv::Scalar color21(45, 60, 150);
        cv::Scalar color22(145, 170, 100);
        std::vector<cv::Scalar> color_palette = {color0, color1, color2, color3, color4, color5, color6, color7, color8, color9, color10, color11, color12, color13, color14, color15, color16, color17, color18, color19, color20, color21, color22};

        // Copy variables within scoped mutex
        {
            unique_lock<mutex> lock(mMutex);
            state = mState;
            if (mState == Tracking::SYSTEM_NOT_READY)
                mState = Tracking::NO_IMAGES_YET;

            mIm.copyTo(im);

            if (mState == Tracking::NOT_INITIALIZED)
            {
                vCurrentKeys = mvCurrentKeys;
                vCurrentKeysMoving = mvCurrentKeysMoving;
                vCurrentKeysSemantic = mvCurrentKeysSemantic;
                vCurrentMapSemantic = mvCurrentMapSemantic;

                vIniKeys = mvIniKeys;
                vMatches = mvIniMatches;
                vTracks = mvTracks;
            }
            else if (mState == Tracking::OK)
            {
                vCurrentKeys = mvCurrentKeys;
                vCurrentKeysMoving = mvCurrentKeysMoving;
                vCurrentKeysSemantic = mvCurrentKeysSemantic;
                vCurrentMapSemantic = mvCurrentMapSemantic;

                vbVO = mvbVO;
                vbMap = mvbMap;

                currentFrame = mCurrentFrame;
                vpLocalMap = mvpLocalMap;
                vMatchesKeys = mvMatchedKeys;
                vpMatchedMPs = mvpMatchedMPs;
                vOutlierKeys = mvOutlierKeys;
                vpOutlierMPs = mvpOutlierMPs;
                mProjectPoints = mmProjectPoints;
                mMatchedInImage = mmMatchedInImage;

                vCurrentDepth = mvCurrentDepth;
                thDepth = mThDepth;
            }
            else if (mState == Tracking::LOST)
            {
                vCurrentKeys = mvCurrentKeys;
                vCurrentKeysMoving = mvCurrentKeysMoving;
                vCurrentKeysSemantic = mvCurrentKeysSemantic;
                vCurrentMapSemantic = mvCurrentMapSemantic;
            }
        }

        if (imageScale != 1.f)
        {
            int imWidth = im.cols / imageScale;
            int imHeight = im.rows / imageScale;
            cv::resize(im, im, cv::Size(imWidth, imHeight));
        }

        if (im.channels() < 3) // this should be always true
            cvtColor(im, im, cv::COLOR_GRAY2BGR);

        // Draw
        if (state == Tracking::NOT_INITIALIZED)
        {
            for (unsigned int i = 0; i < vMatches.size(); i++)
            {
                if (vMatches[i] >= 0)
                {
                    cv::Point2f pt1, pt2;
                    if (imageScale != 1.f)
                    {
                        pt1 = vIniKeys[i].pt / imageScale;
                        pt2 = vCurrentKeys[vMatches[i]].pt / imageScale;
                    }
                    else
                    {
                        pt1 = vIniKeys[i].pt;
                        pt2 = vCurrentKeys[vMatches[i]].pt;
                    }
                    cv::line(im, pt1, pt2, standardColor);
                }
            }
            for (auto &vTrack : vTracks)
            {
                cv::Point2f pt1, pt2;
                if (imageScale != 1.f)
                {
                    pt1 = vTrack.first / imageScale;
                    pt2 = vTrack.second / imageScale;
                }
                else
                {
                    pt1 = vTrack.first;
                    pt2 = vTrack.second;
                }
                cv::line(im, pt1, pt2, standardColor, 5);
            }
        }
        else if (state == Tracking::OK) // TRACKING
        {
            mnTracked = 0;
            mnTrackedVO = 0;
            const float r = 5;
            int n = vCurrentKeys.size();
            for (int i = 0; i < n; i++)
            {
                if (vbVO[i] || vbMap[i])
                {
                    cv::Point2f pt1, pt2;
                    cv::Point2f point;
                    if (imageScale != 1.f)
                    {
                        point = vCurrentKeys[i].pt / imageScale;
                        float px = vCurrentKeys[i].pt.x / imageScale;
                        float py = vCurrentKeys[i].pt.y / imageScale;
                        pt1.x = px - r;
                        pt1.y = py - r;
                        pt2.x = px + r;
                        pt2.y = py + r;
                    }
                    else
                    {
                        point = vCurrentKeys[i].pt;
                        pt1.x = vCurrentKeys[i].pt.x - r;
                        pt1.y = vCurrentKeys[i].pt.y - r;
                        pt2.x = vCurrentKeys[i].pt.x + r;
                        pt2.y = vCurrentKeys[i].pt.y + r;
                    }

                    // This is a match to a MapPoint in the map
                    if (vbMap[i])
                    {
                        // std::cout << vCurrentKeysSemantic[i] << std::endl;
                        // switch (vCurrentKeysSemantic[i])
                        // {
                        // case 0:
                        //     cv::rectangle(im, pt1, pt2, color0);
                        // case 1:
                        //     cv::rectangle(im, pt1, pt2, color1);
                        // case 2:
                        //     cv::rectangle(im, pt1, pt2, color2);
                        // case 3:
                        //     cv::rectangle(im, pt1, pt2, color3);
                        // case 4:
                        //     cv::rectangle(im, pt1, pt2, color4);
                        // case 5:
                        //     cv::rectangle(im, pt1, pt2, color5);
                        // case 6:
                        //     cv::rectangle(im, pt1, pt2, color6);
                        // case 7:
                        //     cv::rectangle(im, pt1, pt2, color7);
                        // case 8:
                        //     cv::rectangle(im, pt1, pt2, color8);
                        // case 9:
                        //     cv::rectangle(im, pt1, pt2, color9);
                        // case 10:
                        //     cv::rectangle(im, pt1, pt2, color10);
                        // case 11:
                        //     cv::rectangle(im, pt1, pt2, color11);
                        // case 12:
                        //     cv::rectangle(im, pt1, pt2, color12);
                        // case 13:
                        //     cv::rectangle(im, pt1, pt2, color13);
                        // case 14:
                        //     cv::rectangle(im, pt1, pt2, color14);
                        // case 15:
                        //     cv::rectangle(im, pt1, pt2, color15);
                        // case 16:
                        //     cv::rectangle(im, pt1, pt2, color16);
                        // case 17:
                        //     cv::rectangle(im, pt1, pt2, color17);
                        // case 18:
                        //     cv::rectangle(im, pt1, pt2, color18);
                        // case 19:
                        //     cv::rectangle(im, pt1, pt2, color19);
                        // case 20:
                        //     cv::rectangle(im, pt1, pt2, color20);
                        // case 21:
                        //     cv::rectangle(im, pt1, pt2, color21);
                        // case 22:
                        //     cv::rectangle(im, pt1, pt2, color22);
                        // }

                        // switch (vCurrentKeysSemantic[i])
                        // {
                        // case 0:
                        //     cv::circle(im, point, 2, color0, -1);
                        // case 1:
                        //     cv::circle(im, point, 2, color1, -1);
                        // case 2:
                        //     cv::circle(im, point, 2, color2, -1);
                        // case 3:
                        //     cv::circle(im, point, 2, color3, -1);
                        // case 4:
                        //     cv::circle(im, point, 2, color4, -1);
                        // case 5:
                        //     cv::circle(im, point, 2, color5, -1);
                        // case 6:
                        //     cv::circle(im, point, 2, color6, -1);
                        // case 7:
                        //     cv::circle(im, point, 2, color7, -1);
                        // case 8:
                        //     cv::circle(im, point, 2, color8, -1);
                        // case 9:
                        //     cv::circle(im, point, 2, color9, -1);
                        // case 10:
                        //     cv::circle(im, point, 2, color10, -1);
                        // case 11:
                        //     cv::circle(im, point, 2, color11, -1);
                        // case 12:
                        //     cv::circle(im, point, 2, color12, -1);
                        // case 13:
                        //     cv::circle(im, point, 2, color13, -1);
                        // case 14:
                        //     cv::circle(im, point, 2, color14, -1);
                        // case 15:
                        //     cv::circle(im, point, 2, color15, -1);
                        // case 16:
                        //     cv::circle(im, point, 2, color16, -1);
                        // case 17:
                        //     cv::circle(im, point, 2, color17, -1);
                        // case 18:
                        //     cv::circle(im, point, 2, color18, -1);
                        // case 19:
                        //     cv::circle(im, point, 2, color19, -1);
                        // case 20:
                        //     cv::circle(im, point, 2, color20, -1);
                        // case 21:
                        //     cv::circle(im, point, 2, color21, -1);
                        // case 22:
                        //     cv::circle(im, point, 2, color22, -1);
                        // }
                        // std::cout << vCurrentKeysSemantic[i] << " " << color_palette[vCurrentKeysSemantic[i]] << std::endl;
                        if (vCurrentMapSemantic[i] != vCurrentKeysSemantic[i])
                            cv::rectangle(im, pt1, pt2, color_palette[vCurrentMapSemantic[i]]);
                        cv::circle(im, point, 2, color_palette[vCurrentKeysSemantic[i]], -1);
                        // cv::rectangle(im, pt1, pt2, standardColor);
                        // cv::circle(im, point, 2, standardColor, -1);
                        mnTracked++;
                    }
                    else // This is match to a "visual odometry" MapPoint created in the last frame
                    {
                        // cv::rectangle(im, pt1, pt2, odometryColor);
                        // cv::circle(im, point, 2, odometryColor, -1);
                        mnTrackedVO++;
                    }
                }
            }
        }

        cv::Mat imWithInfo;
        DrawTextInfo(im, state, imWithInfo);

        return imWithInfo;
    }

    cv::Mat FrameDrawer::DrawRightFrame(float imageScale)
    {
        cv::Mat im;
        vector<cv::KeyPoint> vIniKeys;     // Initialization: KeyPoints in reference frame
        vector<int> vMatches;              // Initialization: correspondeces with reference keypoints
        vector<cv::KeyPoint> vCurrentKeys; // KeyPoints in current frame
        vector<bool> vbVO, vbMap;          // Tracked MapPoints in current frame
        int state;                         // Tracking state

        // Copy variables within scoped mutex
        {
            unique_lock<mutex> lock(mMutex);
            state = mState;
            if (mState == Tracking::SYSTEM_NOT_READY)
                mState = Tracking::NO_IMAGES_YET;

            mImRight.copyTo(im);

            if (mState == Tracking::NOT_INITIALIZED)
            {
                vCurrentKeys = mvCurrentKeysRight;
                vIniKeys = mvIniKeys;
                vMatches = mvIniMatches;
            }
            else if (mState == Tracking::OK)
            {
                vCurrentKeys = mvCurrentKeysRight;
                vbVO = mvbVO;
                vbMap = mvbMap;
            }
            else if (mState == Tracking::LOST)
            {
                vCurrentKeys = mvCurrentKeysRight;
            }
        } // destroy scoped mutex -> release mutex

        if (imageScale != 1.f)
        {
            int imWidth = im.cols / imageScale;
            int imHeight = im.rows / imageScale;
            cv::resize(im, im, cv::Size(imWidth, imHeight));
        }

        if (im.channels() < 3) // this should be always true
            cvtColor(im, im, cv::COLOR_GRAY2BGR);

        // Draw
        if (state == Tracking::NOT_INITIALIZED) // INITIALIZING
        {
            for (unsigned int i = 0; i < vMatches.size(); i++)
            {
                if (vMatches[i] >= 0)
                {
                    cv::Point2f pt1, pt2;
                    if (imageScale != 1.f)
                    {
                        pt1 = vIniKeys[i].pt / imageScale;
                        pt2 = vCurrentKeys[vMatches[i]].pt / imageScale;
                    }
                    else
                    {
                        pt1 = vIniKeys[i].pt;
                        pt2 = vCurrentKeys[vMatches[i]].pt;
                    }

                    cv::line(im, pt1, pt2, cv::Scalar(0, 255, 0));
                }
            }
        }
        else if (state == Tracking::OK) // TRACKING
        {
            mnTracked = 0;
            mnTrackedVO = 0;
            const float r = 5;
            const int n = mvCurrentKeysRight.size();
            const int Nleft = mvCurrentKeys.size();

            for (int i = 0; i < n; i++)
            {
                if (vbVO[i + Nleft] || vbMap[i + Nleft])
                {
                    cv::Point2f pt1, pt2;
                    cv::Point2f point;
                    if (imageScale != 1.f)
                    {
                        point = mvCurrentKeysRight[i].pt / imageScale;
                        float px = mvCurrentKeysRight[i].pt.x / imageScale;
                        float py = mvCurrentKeysRight[i].pt.y / imageScale;
                        pt1.x = px - r;
                        pt1.y = py - r;
                        pt2.x = px + r;
                        pt2.y = py + r;
                    }
                    else
                    {
                        point = mvCurrentKeysRight[i].pt;
                        pt1.x = mvCurrentKeysRight[i].pt.x - r;
                        pt1.y = mvCurrentKeysRight[i].pt.y - r;
                        pt2.x = mvCurrentKeysRight[i].pt.x + r;
                        pt2.y = mvCurrentKeysRight[i].pt.y + r;
                    }

                    // This is a match to a MapPoint in the map
                    if (vbMap[i + Nleft])
                    {
                        cv::rectangle(im, pt1, pt2, cv::Scalar(0, 255, 0));
                        cv::circle(im, point, 2, cv::Scalar(0, 255, 0), -1);
                        mnTracked++;
                    }
                    else // This is match to a "visual odometry" MapPoint created in the last frame
                    {
                        cv::rectangle(im, pt1, pt2, cv::Scalar(255, 0, 0));
                        cv::circle(im, point, 2, cv::Scalar(255, 0, 0), -1);
                        mnTrackedVO++;
                    }
                }
            }
        }

        cv::Mat imWithInfo;
        DrawTextInfo(im, state, imWithInfo);

        return imWithInfo;
    }

    void FrameDrawer::DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText)
    {
        stringstream s;
        if (nState == Tracking::NO_IMAGES_YET)
            s << " WAITING FOR IMAGES";
        else if (nState == Tracking::NOT_INITIALIZED)
            s << " TRYING TO INITIALIZE ";
        else if (nState == Tracking::OK)
        {
            if (!mbOnlyTracking)
                s << "SLAM MODE |  ";
            else
                s << "LOCALIZATION | ";
            int nMaps = mpAtlas->CountMaps();
            int nKFs = mpAtlas->KeyFramesInMap();
            int nMPs = mpAtlas->MapPointsInMap();
            s << "Maps: " << nMaps << ", KFs: " << nKFs << ", MPs: " << nMPs << ", Matches: " << mnTracked;
            if (mnTrackedVO > 0)
                s << ", + VO matches: " << mnTrackedVO;
        }
        else if (nState == Tracking::LOST)
        {
            s << " TRACK LOST. TRYING TO RELOCALIZE ";
        }
        else if (nState == Tracking::SYSTEM_NOT_READY)
        {
            s << " LOADING ORB VOCABULARY. PLEASE WAIT...";
        }

        int baseline = 0;
        cv::Size textSize = cv::getTextSize(s.str(), cv::FONT_HERSHEY_PLAIN, 1, 1, &baseline);

        imText = cv::Mat(im.rows + textSize.height + 10, im.cols, im.type());
        im.copyTo(imText.rowRange(0, im.rows).colRange(0, im.cols));
        imText.rowRange(im.rows, imText.rows) = cv::Mat::zeros(textSize.height + 10, im.cols, im.type());
        cv::putText(imText, s.str(), cv::Point(5, imText.rows - 5), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255), 1, 8);
    }

    void FrameDrawer::Update(Tracking *pTracker)
    {
        unique_lock<mutex> lock(mMutex);
        pTracker->mImGray.copyTo(mIm);
        mvCurrentKeys = pTracker->mCurrentFrame.mvKeys;
        mvCurrentKeysMoving = pTracker->mCurrentFrame.mvKeysMoving;
        mvCurrentKeysSemantic = pTracker->mCurrentFrame.mvSemanticCls;

        mThDepth = pTracker->mCurrentFrame.mThDepth;
        mvCurrentDepth = pTracker->mCurrentFrame.mvDepth;

        if (both)
        {
            mvCurrentKeysRight = pTracker->mCurrentFrame.mvKeysRight;
            pTracker->mImRight.copyTo(mImRight);
            N = mvCurrentKeys.size() + mvCurrentKeysRight.size();
        }
        else
        {
            N = mvCurrentKeys.size();
        }

        mvbVO = vector<bool>(N, false);
        mvbMap = vector<bool>(N, false);
        mbOnlyTracking = pTracker->mbOnlyTracking;

        // Variables for the new visualization
        mCurrentFrame = pTracker->mCurrentFrame;
        mmProjectPoints = mCurrentFrame.mmProjectPoints;
        mmMatchedInImage.clear();

        mvpLocalMap = pTracker->GetLocalMapMPS();
        mvMatchedKeys.clear();
        mvMatchedKeys.reserve(N);
        mvpMatchedMPs.clear();
        mvpMatchedMPs.reserve(N);
        mvOutlierKeys.clear();
        mvOutlierKeys.reserve(N);
        mvpOutlierMPs.clear();
        mvpOutlierMPs.reserve(N);
        mvCurrentMapSemantic.clear();
        mvCurrentMapSemantic.resize(N);

        if (pTracker->mLastProcessedState == Tracking::NOT_INITIALIZED)
        {
            mvIniKeys = pTracker->mInitialFrame.mvKeys;
            mvIniMatches = pTracker->mvIniMatches;
        }
        else if (pTracker->mLastProcessedState == Tracking::OK)
        {
            for (int i = 0; i < N; i++)
            {
                MapPoint *pMP = pTracker->mCurrentFrame.mvpMapPoints[i];
                if (pMP)
                {
                    if (!pTracker->mCurrentFrame.mvbOutlier[i])
                    {
                        if (pMP->Observations() > 0)
                            mvbMap[i] = true;
                        else
                            mvbVO[i] = true;

                        mvCurrentMapSemantic[i] = pMP->mvSemanticCls;
                        mmMatchedInImage[pMP->mnId] = mvCurrentKeys[i].pt;
                    }
                    else
                    {
                        mvpOutlierMPs.push_back(pMP);
                        mvOutlierKeys.push_back(mvCurrentKeys[i]);
                    }
                }
            }
        }
        mState = static_cast<int>(pTracker->mLastProcessedState);
    }

} // namespace ORB_SLAM

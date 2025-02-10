/**
* This file is part of ORB-LINE-SLAM
*
* Copyright (C) 2020-2021 John Alamanos, National Technical University of Athens.
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
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

#ifndef MAPLINE_H
#define MAPLINE_H

#include"KeyFrame.h"
#include"Frame.h"
#include"Map.h"

#include <opencv2/opencv.hpp>
#include <mutex>
#include <map>

#include "Eigen/Core"

namespace ORB_SLAM3
{
class ObservationLine{
public:
    ObservationLine() = default;
    ObservationLine(KeyFrame* projKeyFrame, const unsigned long int& projIndex, KeyFrame* referenceKeyframe, const unsigned long int& referenceLineIndex);

    KeyFrame* projKeyframe{};
    unsigned long int  projIndex{};

    int projOctave{};

    KeyFrame* refKeyframe{};
    unsigned long int  refIndex{};
    int refOctave{};
};

class KeyFrame;
class Map;
class Frame;

class MapLine
{
public:
    MapLine(const Eigen::Vector3f &sP, const Eigen::Vector3f &eP, Map* pMap);
    MapLine(const Eigen::Vector3f &sP, const Eigen::Vector3f &eP, KeyFrame* pRefKF, Map* pMap);
    MapLine(const Eigen::Vector3f &sP, const Eigen::Vector3f &eP,  Map* pMap, Frame* pFrame, const int &idxF);

    void SetWorldPos(const Eigen::Vector3f &sP, const Eigen::Vector3f &eP);
    Eigen::Matrix<float, 6, 1> GetWorldPos();

    Eigen::Vector3f GetNormal();

    KeyFrame* GetReferenceKeyFrame();

    std::map<unsigned long int, ObservationLine> GetObservations();
    int Observations();

    void AddObservation(KeyFrame* pKF, const unsigned long int &projIndex);
    void EraseObservation(KeyFrame* pKF);

    int GetIndexInKeyFrame(KeyFrame* pKF);
    bool IsInKeyFrame(KeyFrame* pKF);

    void SetBadFlag();
    bool isBad();

    void Replace(MapLine* pML);    
    MapLine* GetReplaced();

    void IncreaseVisible(int n=1);
    void IncreaseFound(int n=1);
    float GetFoundRatio();
    inline int GetFound(){
        return mnFound;
    }
    MapLine* ComputeDistinctiveDescriptors();

    cv::Mat GetDescriptor();

    void UpdateNormalAndDepth();
    void SetNormalVector(Eigen::Vector3f& normal);

    void UpdateMap(Map* pMap);

    float GetMinDistanceInvariance();
    float GetMaxDistanceInvariance();

    KeyFrame* SetReferenceKeyFrame(KeyFrame* RFKF);
    KeyFrame* GetCurrentRefKeyframe();
    void SetCurrentRefKeyframeIndex(const int& refKeyframeIndex);

    Map* GetMap();

public:
    long unsigned int mnId;
    static long unsigned int nNextId;
    long int mnFirstKFid;
    long int mnFirstFrame;
    int nObs;

    // Variables used by loop closing
    long unsigned int mnCorrectedByKF;
    long unsigned int mnCorrectedReference;    

    static std::mutex mGlobalMutex;

     // Position in absolute coordinates
     Eigen::Vector3f mWorldPos_sP;
     Eigen::Vector3f mWorldPos_eP;

     // Tracking
     bool mbTrackInView;
     float mTrackProjsX;
     float mTrackProjsY;
     float mTrackProjeX;
     float mTrackProjeY;
     double mnTrackangle;
     long unsigned int mnTrackReferenceForFrame;
     long unsigned int mnLastFrameSeen;

    // Variables used by local mapping
    long unsigned int mnBALocalForKF;

    // Used for Loop Closing
    Eigen::Vector3f mPosGBA;
    long unsigned int mnBAGlobalForKF;
    long unsigned int mnBALocalForMerge;

    // Variable used by merging
    Eigen::Vector3f mPosMerge;
    Eigen::Vector3f mNormalVectorMerge;
    
protected:    

     // Keyframes observing the line and associated index in keyframe
     std::map<unsigned long int, ObservationLine> mObservations;
     KeyFrame* currentReferenceKeyframe;
     unsigned long int currentReferenceKeypointIndex;

    // Mean viewing direction
     Eigen::Vector3f mNormalVector;

     // Best descriptor to fast matching
     cv::Mat mDescriptor;

     // Reference KeyFrame
     KeyFrame* mpRefKF;

     // Tracking counters
     int mnVisible;
     int mnFound;

     // Bad flag (we do not currently erase MapPoint from memory)
     bool mbBad;
     MapLine* mpReplaced;

     // Scale invariance distances
     float mfMinDistance;
     float mfMaxDistance;

     Map* mpMap;

     std::mutex mMutexMap;
     std::mutex mMutexPos;
     std::mutex mMutexFeatures;
};

} //namespace ORB_SLAM

#endif // MAPLINE_H

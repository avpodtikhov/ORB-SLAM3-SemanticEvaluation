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

#include "MapLine.h"
#include "ORBmatcher.h"
#include "Converter.h"

#include <mutex>

namespace ORB_SLAM3
{
    ObservationLine::ObservationLine(KeyFrame *projKeyframe, const unsigned long int &projIndex,
                             KeyFrame *refKeyframe, const unsigned long int &refIndex) : projKeyframe(projKeyframe), projIndex(projIndex),
                                                                                          refKeyframe(refKeyframe), refIndex(refIndex)
    {
        projOctave = projKeyframe->mvKeysUnLine[projIndex].octave;
        refOctave = refKeyframe->mvKeysUnLine[refIndex].octave;
    }

    long unsigned int MapLine::nNextId=0;
    mutex MapLine::mGlobalMutex;

    MapLine::MapLine(const Eigen::Vector3f &sP, const Eigen::Vector3f &eP, Map* pMap):
        mnFirstKFid(-1), mnFirstFrame(0), nObs(0), mnTrackReferenceForFrame(0),mnLastFrameSeen(0), mnCorrectedByKF(0), mnCorrectedReference(0),
        mpRefKF(static_cast<KeyFrame*>(NULL)), mnVisible(1), mnFound(1), mnBALocalForKF(0),
        mbBad(false), mpReplaced(static_cast<MapLine*>(NULL)), mpMap(pMap)
    {
        SetWorldPos(sP, eP);
        mNormalVector.setZero();

        // MapLines can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
        unique_lock<mutex> lock(mpMap->mMutexLineCreation);
        mnId = nNextId++;
        currentReferenceKeyframe = static_cast<KeyFrame *>(nullptr);
        currentReferenceKeypointIndex = -1;
    }

    MapLine::MapLine(const Eigen::Vector3f &sP, const Eigen::Vector3f &eP, KeyFrame* pRefKF, Map* pMap):
        mnFirstKFid(pRefKF->mnId), mnFirstFrame(pRefKF->mnFrameId), nObs(0), mnTrackReferenceForFrame(0),mnLastFrameSeen(0), mnCorrectedByKF(0), mnCorrectedReference(0),
        mpRefKF(pRefKF), mnVisible(1), mnFound(1), mnBALocalForKF(0),
        mbBad(false), mpReplaced(static_cast<MapLine*>(NULL)), mpMap(pMap)
    {
        SetWorldPos(sP, eP);
        mNormalVector.setZero();

        // MapLines can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
        unique_lock<mutex> lock(mpMap->mMutexLineCreation);
        mnId = nNextId++;
        currentReferenceKeyframe = pRefKF;
        currentReferenceKeypointIndex = -1;
    }

    MapLine::MapLine(const Eigen::Vector3f &sP, const Eigen::Vector3f &eP,  Map* pMap, Frame* pFrame, const int &idxF):
        mnFirstKFid(-1), mnFirstFrame(pFrame->mnId), nObs(0), mnTrackReferenceForFrame(0),mnLastFrameSeen(0), mnCorrectedByKF(0), mnCorrectedReference(0),
        mpRefKF(static_cast<KeyFrame*>(NULL)), mnVisible(1), mnFound(1), mnBALocalForKF(0),
        mbBad(false), mpReplaced(static_cast<MapLine*>(NULL)), mpMap(pMap)
    {
        SetWorldPos(sP, eP);

        Eigen::Vector3f MidPoint = (sP + eP)/2;

        Eigen::Vector3f Ow = pFrame->GetCameraCenter();

        mNormalVector = MidPoint - Ow;
        mNormalVector = mNormalVector / mNormalVector.norm();

        Eigen::Vector3f PC = MidPoint - Ow;
        const float dist = PC.norm();
        const int level = pFrame->mvKeysUnLine[idxF].octave;
        const float levelScaleFactor =  pFrame->mvScaleFactorsLine[level];
        const int nLevels = pFrame->mnScaleLevelsLine;

        mfMaxDistance = dist*levelScaleFactor;
        mfMinDistance = mfMaxDistance/pFrame->mvScaleFactorsLine[nLevels-1]; 

        pFrame->mDescriptorsLine.row(idxF).copyTo(mDescriptor);

        // MapLines can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
        unique_lock<mutex> lock(mpMap->mMutexLineCreation);
        mnId = nNextId++;
        currentReferenceKeyframe = static_cast<KeyFrame *>(nullptr);
        currentReferenceKeypointIndex = -1;
    }

    void MapLine::SetWorldPos(const Eigen::Vector3f &sP, const Eigen::Vector3f &eP)
    {
        unique_lock<mutex> lock2(mGlobalMutex);
        unique_lock<mutex> lock(mMutexPos);
        mWorldPos_sP = sP;
        mWorldPos_eP = eP;
    }

    Eigen::Matrix<float, 6, 1> MapLine::GetWorldPos()
    {
        unique_lock<mutex> lock(mMutexPos);
        Eigen::Matrix<float, 6, 1> sep;
        sep.head(3) = mWorldPos_sP;
        sep.tail(3) = mWorldPos_eP;
        return sep;
    }

    Map* MapLine::GetMap()
    {
        unique_lock<mutex> lock(mMutexMap);
        return mpMap;
    }

    KeyFrame* MapLine::GetReferenceKeyFrame()
    {
        unique_lock<mutex> lock(mMutexFeatures);
        return mpRefKF;
    }

    KeyFrame* MapLine::GetCurrentRefKeyframe()
    {
        return currentReferenceKeyframe;
    }

    void MapLine::SetCurrentRefKeyframeIndex(const int& refKeyframeIndex)
    {
        currentReferenceKeypointIndex = refKeyframeIndex;
    }


    void MapLine::AddObservation(KeyFrame* pKF, const unsigned long int &projIndex)
    {
        // unique_lock<mutex> lock(mMutexFeatures);
        if(mObservations.count(pKF->mnId))
            return;

        mObservations[pKF->mnId] = ObservationLine(pKF, projIndex,
                                            currentReferenceKeyframe,
                                            currentReferenceKeypointIndex);

        if(pKF->mvDepthLine[projIndex].first >= 0 && pKF->mvDepthLine[projIndex].second >= 0)
            nObs+=2;
        else
            nObs++;
    }

    void MapLine::EraseObservation(KeyFrame* pKF)
    {
        bool bBad = false;
        {
            unique_lock<mutex> lock(mMutexFeatures);
            if (mObservations.count(pKF->mnId))
            {
                unsigned long int projIndex = mObservations[pKF->mnId].projIndex;
                if(pKF->mvDepthLine[projIndex].first >= 0 && pKF->mvDepthLine[projIndex].second >= 0)
                    nObs-=2;
                else
                    nObs--;

                mObservations.erase(pKF->mnId);

                if(mpRefKF==pKF)
                    mpRefKF = mObservations.begin()->second.refKeyframe;

                // If only 2 observations or less, discard point
                if(nObs<=2)
                    bBad=true;
            }
        }

        if(bBad)
            SetBadFlag();
    }

    map<unsigned long int, ObservationLine> MapLine::GetObservations()
    {
        unique_lock<mutex> lock(mMutexFeatures);
        return mObservations;
    } 

    int MapLine::Observations()
    {
        unique_lock<mutex> lock(mMutexFeatures);
        return nObs;
    }

    void MapLine::SetBadFlag()
    {
        map<unsigned long int, ObservationLine> obs;
        {
            // unique_lock<mutex> lock1(mMutexFeatures);
            // unique_lock<mutex> lock2(mMutexPos);
            mbBad = true;
            obs = mObservations;
            mObservations.clear();
        }
        for(auto &ob : obs)
        {
            KeyFrame* pKF = ob.second.projKeyframe;
            pKF->EraseMapLineMatch(ob.second.projIndex);
        }

        mpMap->EraseMapLine(this);
    }

    MapLine* MapLine::GetReplaced()
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        return mpReplaced;
    }

    void MapLine::Replace(MapLine* pML)
    {
        if(pML->mnId == this->mnId)
            return;

        int nvisible, nfound;
        map<unsigned long int, ObservationLine> obs;
        {
            unique_lock<mutex> lock1(mMutexFeatures);
            unique_lock<mutex> lock2(mMutexPos);
            obs = mObservations;
            mObservations.clear();
            mbBad = true;
            nvisible = mnVisible;
            nfound = mnFound;
            mpReplaced = pML;
        }

        for(auto &ob : obs)
        {
            // Replace measurement in keyframe
            KeyFrame* pKF = ob.second.projKeyframe;

            if(!pML->IsInKeyFrame(pKF))
            {
                pKF->ReplaceMapLineMatch(ob.second.projIndex, pML);
                pML->AddObservation(pKF, ob.second.projIndex);
            }
            else
            {
                pKF->EraseMapLineMatch(ob.second.projIndex);
            }
        }
        pML->IncreaseFound(nfound);
        pML->IncreaseVisible(nvisible);
        pML->ComputeDistinctiveDescriptors();

        mpMap->EraseMapLine(this);
    }

    bool MapLine::isBad()
    {
        unique_lock<mutex> lock1(mMutexFeatures,std::defer_lock);
        unique_lock<mutex> lock2(mMutexPos,std::defer_lock);
        lock(lock1, lock2);

        return mbBad;
    }

    void MapLine::IncreaseVisible(int n)
    {
        unique_lock<mutex> lock(mMutexFeatures);
        mnVisible+=n;
    }

    void MapLine::IncreaseFound(int n)
    {
        unique_lock<mutex> lock(mMutexFeatures);
        mnFound+=n;
    }

    float MapLine::GetFoundRatio()
    {
        unique_lock<mutex> lock(mMutexFeatures);
        return static_cast<float>(mnFound) / mnVisible;
    }

    MapLine *MapLine::ComputeDistinctiveDescriptors()
    {
        // Retrieve all observed descriptors
        vector<cv::Mat> vDescriptors;
        vector<unsigned long int> projIndexes{};
        vector<KeyFrame *> projectionKeyframes{};

        map<unsigned long int, ObservationLine> observations;

        {
            unique_lock<mutex> lock1(mMutexFeatures);
            if (mbBad)
                return this;
            observations=mObservations;
        }

        if (observations.empty())
            return this;

        vDescriptors.reserve(observations.size());

        for(auto &ob : observations)
        {
            KeyFrame* pKF = ob.second.projKeyframe;

            if(!pKF->isBad()) 
            {
                projIndexes.push_back(ob.second.projIndex);
                projectionKeyframes.push_back(pKF);
                vDescriptors.push_back(pKF->mDescriptorsLine.row(ob.second.projIndex));
            }
        }

        if(vDescriptors.empty())
            return this;

        // Compute distances between them
        const size_t N = vDescriptors.size();

        float Distances[N][N];
        for (size_t i = 0; i < N; i++)
        {
            Distances[i][i] = 0;
            for (size_t j = i + 1; j < N; j++)
            {
                int distij = ORBmatcher::DescriptorDistance(vDescriptors[i],vDescriptors[j]);
                Distances[i][j] = distij;
                Distances[j][i] = distij;
            }
        }

        // Take the descriptor with least median distance to the rest
        int BestMedian = INT_MAX;
        int BestIdx = 0;
        for(size_t i=0;i<N;i++)
        {
            vector<int> vDists(Distances[i],Distances[i]+N);
            sort(vDists.begin(),vDists.end());
            int median = vDists[0.5*(N-1)];

            if(median<BestMedian)
            {
                BestMedian = median;
                BestIdx = i;
            }
        }

        {
            unique_lock<mutex> lock(mMutexFeatures);
            mDescriptor = vDescriptors[BestIdx].clone();
            currentReferenceKeypointIndex = projIndexes[BestIdx];
            currentReferenceKeyframe = projectionKeyframes[BestIdx];
        }
        return this;
    }

    cv::Mat MapLine::GetDescriptor()
    {
        unique_lock<mutex> lock(mMutexFeatures);
        return mDescriptor.clone();
    }

    void MapLine::UpdateNormalAndDepth()
    {
        map<unsigned long int, ObservationLine> observations;
        KeyFrame* pRefKF;
        Eigen::Vector3f Pos;
        {
            unique_lock<mutex> lock1(mMutexFeatures);
            unique_lock<mutex> lock2(mMutexPos);
            if(mbBad)
                return;
            observations = mObservations;
            pRefKF = mpRefKF;
            Pos = (mWorldPos_sP + mWorldPos_eP) / 2;
        }

        if(observations.empty())
            return;

        Eigen::Vector3f normal = Eigen::Vector3f::Zero();
        int n=0;
        for(auto &ob : observations)
        {
            KeyFrame* pKF = ob.second.projKeyframe;
            Eigen::Vector3f Owi = pKF->GetCameraCenter();
            Eigen::Vector3f normali = Pos - Owi;
            normal = normal + normali/normali.norm();
            n++;
        }

        Eigen::Vector3f PC = Pos - pRefKF->GetCameraCenter();
        const float dist = PC.norm();
        const int level = pRefKF->mvKeysUnLine[pRefKF->mnId].octave;
        const float levelScaleFactor =  pRefKF->mvScaleFactorsLine[level];
        const int nLevels = pRefKF->mnScaleLevelsLine;

        {
            unique_lock<mutex> lock3(mMutexPos);
            mfMaxDistance = dist * levelScaleFactor;
            mfMinDistance = mfMaxDistance / pRefKF->mvScaleFactorsLine[nLevels-1];
            mNormalVector = normal/n;
        }
    }

    float MapLine::GetMinDistanceInvariance()
    {
        unique_lock<mutex> lock(mMutexPos);
        return 0.8f*mfMinDistance;
    }

    float MapLine::GetMaxDistanceInvariance()
    {
        unique_lock<mutex> lock(mMutexPos);
        return 1.2f*mfMaxDistance;
    }

    Eigen::Vector3f MapLine::GetNormal()
    {
        unique_lock<mutex> lock(mMutexPos);
        return mNormalVector;
    }

    int MapLine::GetIndexInKeyFrame(KeyFrame *pKF)
    {
        unique_lock<mutex> lock(mMutexFeatures);
        if(mObservations.count(pKF->mnId))
            return mObservations[pKF->mnId].projIndex;
        else
            return -1;
    }

    bool MapLine::IsInKeyFrame(KeyFrame *pKF)
    {
        unique_lock<mutex> lock(mMutexFeatures);
        return (mObservations.count(pKF->mnId));
    }

    // KeyFrame* MapLine::SetReferenceKeyFrame(KeyFrame* RFKF)
    // {
    //     return mpRefKF = RFKF;
    // }

    void MapLine::UpdateMap(Map* pMap)
    {
        unique_lock<mutex> lock(mMutexMap);
        mpMap = pMap;
    }

    void MapLine::SetNormalVector(Eigen::Vector3f& normal)
    {
        unique_lock<mutex> lock3(mMutexPos);
        mNormalVector = normal;
    }
} //namespace ORB_SLAM

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

#include "MapPoint.h"
#include "ORBmatcher.h"

#include <mutex>

namespace ORB_SLAM3
{

    Observation::Observation(KeyFrame *projKeyframe, const tuple<unsigned long int, unsigned long int> &projIndex,
                             KeyFrame *refKeyframe, const tuple<unsigned long int, unsigned long int> &refIndex) : projKeyframe(projKeyframe), projIndex(projIndex),
                                                                                                                   refKeyframe(refKeyframe), refIndex(refIndex)
    {
        int leftIndex = get<0>(projIndex), rightIndex = get<1>(projIndex);
        if (projKeyframe->NLeft == -1)
        {
            projOctave = projKeyframe->mvKeysUn[leftIndex].octave;
        }
        else if (leftIndex != -1)
        {
            projOctave = projKeyframe->mvKeys[leftIndex].octave;
        }
        else
        {
            projOctave = projKeyframe->mvKeysRight[rightIndex - projKeyframe->NLeft].octave;
        }
        leftIndex = get<0>(refIndex), rightIndex = get<1>(refIndex);
        if (projKeyframe->NLeft == -1)
        {
            refOctave = refKeyframe->mvKeysUn[leftIndex].octave;
        }
        else if (leftIndex != -1)
        {
            refOctave = refKeyframe->mvKeys[leftIndex].octave;
        }
        else
        {
            refOctave = refKeyframe->mvKeysRight[rightIndex - refKeyframe->NLeft].octave;
        }

        //        projOctave = projKeyframe->mvKeysUn[projIndex].octave;
        //        refOctave = refKeyframe->mvKeysUn[refIndex].octave;
    }

    long unsigned int MapPoint::nNextId = 0;
    mutex MapPoint::mGlobalMutex;

    MapPoint::MapPoint() : mnFirstKFid(0), mnFirstFrame(0), nObs(0), mnTrackReferenceForFrame(0),
                           mnLastFrameSeen(0), mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
                           mnCorrectedReference(0), mnBAGlobalForKF(0), mnVisible(1), mnFound(1), mbBad(false),
                           mpReplaced(static_cast<MapPoint *>(nullptr))
    {
        mpReplaced = static_cast<MapPoint *>(nullptr);
    }

    MapPoint::MapPoint(const Eigen::Vector3f &Pos, KeyFrame *pRefKF, Map *pMap) : mnFirstKFid(pRefKF->mnId), mnFirstFrame(pRefKF->mnFrameId), nObs(0), mnTrackReferenceForFrame(0),
                                                                                  mnLastFrameSeen(0), mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
                                                                                  mnCorrectedReference(0), mnBAGlobalForKF(0), mnOriginMapId(pMap->GetId()), mpRefKF(pRefKF), mnVisible(1), mnFound(1),
                                                                                  mbBad(false), mpReplaced(static_cast<MapPoint *>(nullptr)), mfMinDistance(0), mfMaxDistance(0),
                                                                                  mpMap(pMap)
    {
        SetWorldPos(Pos);

        mNormalVector.setZero();

        mbTrackInViewR = false;
        mbTrackInView = false;

        // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
        unique_lock<mutex> lock(mpMap->mMutexPointCreation);
        mnId = nNextId++;
        currentReferenceKeyframe = pRefKF;
        currentReferenceKeypointIndex = tuple<int, int>(-1, -1);
    }

    MapPoint::MapPoint(const Eigen::Vector3f &Pos, KeyFrame *pRefKF, Map *pMap, const int semantic_cls, const int instance_cls) : mnFirstKFid(pRefKF->mnId), mnFirstFrame(pRefKF->mnFrameId), nObs(0), mnTrackReferenceForFrame(0),
                                                                                                                                  mnLastFrameSeen(0), mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
                                                                                                                                  mnCorrectedReference(0), mnBAGlobalForKF(0), mnOriginMapId(pMap->GetId()), mpRefKF(pRefKF), mnVisible(1), mnFound(1),
                                                                                                                                  mbBad(false), mpReplaced(static_cast<MapPoint *>(nullptr)), mfMinDistance(0), mfMaxDistance(0),
                                                                                                                                  mpMap(pMap)
    {
        mvSemanticCls = semantic_cls;
        mvInstanceCls = instance_cls;
        mvSemanticDistr = vector<int>(23, 0);
        mvSemanticDistr[semantic_cls] = 1;
        SetWorldPos(Pos);

        mNormalVector.setZero();

        mbTrackInViewR = false;
        mbTrackInView = false;

        // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
        unique_lock<mutex> lock(mpMap->mMutexPointCreation);
        mnId = nNextId++;
        currentReferenceKeyframe = pRefKF;
        currentReferenceKeypointIndex = tuple<int, int>(-1, -1);
    }

    MapPoint::MapPoint(const double invDepth, cv::Point2f uv_init, KeyFrame *pRefKF, KeyFrame *pHostKF, Map *pMap) : mnFirstKFid(pRefKF->mnId), mnFirstFrame(pRefKF->mnFrameId), nObs(0), mnTrackReferenceForFrame(0),
                                                                                                                     mnLastFrameSeen(0), mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
                                                                                                                     mnCorrectedReference(0), mnBAGlobalForKF(0), mnOriginMapId(pMap->GetId()), mpRefKF(pRefKF), mnVisible(1), mnFound(1),
                                                                                                                     mbBad(false), mpReplaced(static_cast<MapPoint *>(nullptr)), mfMinDistance(0), mfMaxDistance(0),
                                                                                                                     mpMap(pMap)
    {
        mInvDepth = invDepth;
        mInitU = (double)uv_init.x;
        mInitV = (double)uv_init.y;
        mpHostKF = pHostKF;

        mNormalVector.setZero();

        // Worldpos is not set
        // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
        unique_lock<mutex> lock(mpMap->mMutexPointCreation);
        mnId = nNextId++;
        currentReferenceKeyframe = pRefKF;
        currentReferenceKeypointIndex = tuple<int, int>(-1, -1);
    }

    MapPoint::MapPoint(const Eigen::Vector3f &Pos, Map *pMap, Frame *pFrame, const int &idxF) : mnFirstKFid(-1), mnFirstFrame(pFrame->mnId), nObs(0), mnTrackReferenceForFrame(0), mnLastFrameSeen(0),
                                                                                                mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
                                                                                                mnCorrectedReference(0), mnBAGlobalForKF(0), mnOriginMapId(pMap->GetId()), mpRefKF(static_cast<KeyFrame *>(nullptr)),
                                                                                                mnVisible(1), mnFound(1), mbBad(false), mpReplaced(nullptr), mpMap(pMap)
    {
        SetWorldPos(Pos);

        Eigen::Vector3f Ow;
        if (pFrame->Nleft == -1 || idxF < pFrame->Nleft)
        {
            Ow = pFrame->GetCameraCenter();
        }
        else
        {
            Eigen::Matrix3f Rwl = pFrame->GetRwc();
            Eigen::Vector3f tlr = pFrame->GetRelativePoseTlr().translation();
            Eigen::Vector3f twl = pFrame->GetOw();

            Ow = Rwl * tlr + twl;
        }
        mNormalVector = mWorldPos - Ow;
        mNormalVector = mNormalVector / mNormalVector.norm();

        Eigen::Vector3f PC = mWorldPos - Ow;
        const float dist = PC.norm();
        const int level = (pFrame->Nleft == -1)    ? pFrame->mvKeysUn[idxF].octave
                          : (idxF < pFrame->Nleft) ? pFrame->mvKeys[idxF].octave
                                                   : pFrame->mvKeysRight[idxF].octave;
        const float levelScaleFactor = pFrame->mvScaleFactors[level];
        const int nLevels = pFrame->mnScaleLevels;

        mfMaxDistance = dist * levelScaleFactor;
        mfMinDistance = mfMaxDistance / pFrame->mvScaleFactors[nLevels - 1];

        pFrame->mDescriptors.row(idxF).copyTo(mDescriptor);

        // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
        unique_lock<mutex> lock(mpMap->mMutexPointCreation);
        mnId = nNextId++;
        currentReferenceKeyframe = static_cast<KeyFrame *>(nullptr);
        currentReferenceKeypointIndex = tuple<int, int>(-1, -1);
    }
    MapPoint::MapPoint(const Eigen::Vector3f &Pos, Map *pMap, Frame *pFrame, const int &idxF, const int semantic_cls, const int instance_cls) : mnFirstKFid(-1), mnFirstFrame(pFrame->mnId), nObs(0), mnTrackReferenceForFrame(0), mnLastFrameSeen(0),
                                                                                                                                                mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
                                                                                                                                                mnCorrectedReference(0), mnBAGlobalForKF(0), mnOriginMapId(pMap->GetId()), mpRefKF(static_cast<KeyFrame *>(nullptr)),
                                                                                                                                                mnVisible(1), mnFound(1), mbBad(false), mpReplaced(nullptr), mpMap(pMap)
    {
        mvSemanticCls = semantic_cls;
        mvInstanceCls = instance_cls;
        mvSemanticDistr = vector<int>(23, 0);
        mvSemanticDistr[semantic_cls] = 1;
        mvInstanceDistr[instance_cls] = 1;

        SetWorldPos(Pos);

        Eigen::Vector3f Ow;
        if (pFrame->Nleft == -1 || idxF < pFrame->Nleft)
        {
            Ow = pFrame->GetCameraCenter();
        }
        else
        {
            Eigen::Matrix3f Rwl = pFrame->GetRwc();
            Eigen::Vector3f tlr = pFrame->GetRelativePoseTlr().translation();
            Eigen::Vector3f twl = pFrame->GetOw();

            Ow = Rwl * tlr + twl;
        }
        mNormalVector = mWorldPos - Ow;
        mNormalVector = mNormalVector / mNormalVector.norm();

        Eigen::Vector3f PC = mWorldPos - Ow;
        const float dist = PC.norm();
        const int level = (pFrame->Nleft == -1)    ? pFrame->mvKeysUn[idxF].octave
                          : (idxF < pFrame->Nleft) ? pFrame->mvKeys[idxF].octave
                                                   : pFrame->mvKeysRight[idxF].octave;
        const float levelScaleFactor = pFrame->mvScaleFactors[level];
        const int nLevels = pFrame->mnScaleLevels;

        mfMaxDistance = dist * levelScaleFactor;
        mfMinDistance = mfMaxDistance / pFrame->mvScaleFactors[nLevels - 1];

        pFrame->mDescriptors.row(idxF).copyTo(mDescriptor);

        // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
        unique_lock<mutex> lock(mpMap->mMutexPointCreation);
        mnId = nNextId++;
        currentReferenceKeyframe = static_cast<KeyFrame *>(nullptr);
        currentReferenceKeypointIndex = tuple<int, int>(-1, -1);
    }

    void MapPoint::SetWorldPos(const Eigen::Vector3f &Pos)
    {
        unique_lock<mutex> lock2(mGlobalMutex);
        unique_lock<mutex> lock(mMutexPos);
        mWorldPos = Pos;
    }

    Eigen::Vector3f MapPoint::GetWorldPos()
    {
        unique_lock<mutex> lock(mMutexPos);
        return mWorldPos;
    }

    Eigen::Vector3f MapPoint::GetNormal()
    {
        unique_lock<mutex> lock(mMutexPos);
        return mNormalVector;
    }

    KeyFrame *MapPoint::GetReferenceKeyFrame()
    {
        unique_lock<mutex> lock(mMutexFeatures);
        return mpRefKF;
    }

    void MapPoint::SetCurrentRefKeyframeIndex(const tuple<unsigned long int, unsigned long int> &refKeyframeIndex)
    {
        currentReferenceKeypointIndex = refKeyframeIndex;
    }

    KeyFrame *MapPoint::GetCurrentRefKeyframe()
    {
        return currentReferenceKeyframe;
    }

    void MapPoint::AddSemObservation(int semantic_cls)
    {
        mvSemanticDistr[semantic_cls] += 1;
        auto max_value_iter = std::max_element(mvSemanticDistr.begin(), mvSemanticDistr.end());
        int max_value_idx = std::distance(mvSemanticDistr.begin(), max_value_iter);
        if (mvSemanticDistr[mvSemanticCls] == *max_value_iter)
            return;
        mvSemanticCls = max_value_idx;
    }

    void MapPoint::AddInstanceObservation(int instance_cls)
    {
        if (mvInstanceDistr.count(instance_cls))
            mvInstanceDistr[instance_cls] += 1;
        else
            mvInstanceDistr[instance_cls] = 1;
        auto max_value_iter = std::max_element(mvInstanceDistr.begin(), mvInstanceDistr.end(), [](const std::pair<char, int> &a, const std::pair<char, int> &b) -> bool
                                               { return a.second < b.second; });
        int max_value_idx = max_value_iter->first;
        if (mvInstanceDistr[mvSemanticCls] == max_value_iter->second)
            return;
        mvInstanceCls = max_value_idx;
    }

    void MapPoint::AddObservation(KeyFrame *pKF, const unsigned long int &projIndex)
    {
        // unique_lock<mutex> lock(mMutexFeatures);
        tuple<int, int> indexes;

        if (mObservations.count(pKF->mnId))
        {
            indexes = mObservations[pKF->mnId].projIndex;
        }
        else
        {
            indexes = tuple<int, int>(-1, -1);
        }

        if (pKF->NLeft != -1 && projIndex >= pKF->NLeft)
        {
            get<1>(indexes) = projIndex;
        }
        else
        {
            get<0>(indexes) = projIndex;
        }

        mObservations[pKF->mnId] = Observation(pKF, indexes,
                                               currentReferenceKeyframe,
                                               currentReferenceKeypointIndex);

        if (!pKF->mpCamera2 && pKF->mvuRight[projIndex] >= 0)
            nObs += 2;
        else
            nObs++;
        // ComputeDistinctiveDescriptors()->UpdateNormalAndDepth();
    }

    void MapPoint::EraseObservation(KeyFrame *pKF)
    {
        bool bBad = false;
        {
            unique_lock<mutex> lock(mMutexFeatures);
            if (mObservations.count(pKF->mnId))
            {
                tuple<int, int> indexes = mObservations[pKF->mnId].projIndex;
                int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);

                if (leftIndex != -1)
                {
                    if (!pKF->mpCamera2 && pKF->mvuRight[leftIndex] >= 0)
                        nObs -= 2;
                    else
                        nObs--;
                }
                if (rightIndex != -1)
                {
                    nObs--;
                }

                mObservations.erase(pKF->mnId);

                if (mpRefKF == pKF)
                    mpRefKF = mObservations.begin()->second.projKeyframe;

                // If only 2 observations or less, discard point
                if (nObs <= 2)
                    bBad = true;
            }
        }

        if (bBad)
            SetBadFlag();
        else
            ComputeDistinctiveDescriptors()->UpdateNormalAndDepth();
    }

    std::map<unsigned long int, Observation> MapPoint::GetObservations()
    {
        unique_lock<mutex> lock(mMutexFeatures);
        return mObservations;
    }

    int MapPoint::Observations()
    {
        unique_lock<mutex> lock(mMutexFeatures);
        return nObs;
    }

    int MapPoint::GetNumberOfObservations()
    {
        return int(mObservations.size());
    }

    void MapPoint::SetBadFlag()
    {
        map<unsigned long int, Observation> obs;
        {
            // unique_lock<mutex> lock1(mMutexFeatures);
            // unique_lock<mutex> lock2(mMutexPos);
            mbBad = true;
            obs = mObservations;
            mObservations.clear();
        }
        for (auto &ob : obs)
        {
            KeyFrame *pKF = ob.second.projKeyframe;
            int leftIndex = get<0>(ob.second.projIndex), rightIndex = get<1>(ob.second.projIndex);
            if (leftIndex != -1)
            {
                pKF->EraseMapPointMatch(leftIndex);
            }
            if (rightIndex != -1)
            {
                pKF->EraseMapPointMatch(rightIndex);
            }
        }

        mpMap->EraseMapPoint(this);
    }

    MapPoint *MapPoint::GetReplaced()
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        return mpReplaced;
    }

    void MapPoint::Replace(MapPoint *pMP)
    {
        if (pMP->mnId == this->mnId)
            return;

        int nvisible, nfound;
        map<unsigned long int, Observation> obs;
        {
            unique_lock<mutex> lock1(mMutexFeatures);
            unique_lock<mutex> lock2(mMutexPos);
            obs = mObservations;
            mObservations.clear();
            mbBad = true;
            nvisible = mnVisible;
            nfound = mnFound;
            mpReplaced = pMP;
        }

        for (auto &ob : obs)
        {
            // Replace measurement in keyframe
            KeyFrame *pKF = ob.second.projKeyframe;

            tuple<int, int> indexes = ob.second.projIndex;
            int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);

            if (!pMP->IsInKeyFrame(pKF))
            {
                if (leftIndex != -1)
                {
                    pKF->ReplaceMapPointMatch(leftIndex, pMP);
                    pMP->AddObservation(pKF, leftIndex);
                }
                if (rightIndex != -1)
                {
                    pKF->ReplaceMapPointMatch(rightIndex, pMP);
                    pMP->AddObservation(pKF, rightIndex);
                }
            }
            else
            {
                if (leftIndex != -1)
                {
                    pKF->EraseMapPointMatch(leftIndex);
                }
                if (rightIndex != -1)
                {
                    pKF->EraseMapPointMatch(rightIndex);
                }
            }
        }
        pMP->IncreaseFound(nfound);
        pMP->IncreaseVisible(nvisible);
        pMP->ComputeDistinctiveDescriptors();

        mpMap->EraseMapPoint(this);
    }

    bool MapPoint::isBad()
    {
        unique_lock<mutex> lock1(mMutexFeatures, std::defer_lock);
        unique_lock<mutex> lock2(mMutexPos, std::defer_lock);
        lock(lock1, lock2);

        return mbBad;
    }

    void MapPoint::IncreaseVisible(int n)
    {
        unique_lock<mutex> lock(mMutexFeatures);
        mnVisible += n;
    }

    void MapPoint::IncreaseFound(int n)
    {
        unique_lock<mutex> lock(mMutexFeatures);
        mnFound += n;
    }

    float MapPoint::GetFoundRatio()
    {
        unique_lock<mutex> lock(mMutexFeatures);
        return static_cast<float>(mnFound) / mnVisible;
    }

    MapPoint *MapPoint::ComputeDistinctiveDescriptors()
    {
        // Retrieve all observed descriptors
        vector<cv::Mat> vDescriptors;
        vector<tuple<unsigned long int, unsigned long int>> projIndexes{};
        vector<KeyFrame *> projectionKeyframes{};

        map<unsigned long int, Observation> observations;

        {
            unique_lock<mutex> lock1(mMutexFeatures);
            if (mbBad)
                return this;
            observations = mObservations;
        }

        if (observations.empty())
            return this;

        vDescriptors.reserve(observations.size());

        for (auto &observation : observations)
        {
            KeyFrame *pKF = observation.second.projKeyframe;

            if (!pKF->isBad())
            {
                tuple<int, int> indexes = observation.second.projIndex;
                projIndexes.push_back(observation.second.projIndex);
                projectionKeyframes.push_back(pKF);

                int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);

                if (leftIndex != -1)
                {
                    vDescriptors.push_back(pKF->mDescriptors.row(leftIndex));
                }
                if (rightIndex != -1)
                {
                    vDescriptors.push_back(pKF->mDescriptors.row(rightIndex));
                }
            }
        }

        if (vDescriptors.empty())
            return this;

        // Compute distances between them
        const size_t N = vDescriptors.size();

        float Distances[N][N];
        for (size_t i = 0; i < N; i++)
        {
            Distances[i][i] = 0;
            for (size_t j = i + 1; j < N; j++)
            {
                int distij = ORBmatcher::DescriptorDistance(vDescriptors[i], vDescriptors[j]);
                Distances[i][j] = distij;
                Distances[j][i] = distij;
            }
        }

        // Take the descriptor with least median distance to the rest
        int BestMedian = INT_MAX;
        int BestIdx = 0;
        for (size_t i = 0; i < N; i++)
        {
            vector<int> vDists(Distances[i], Distances[i] + N);
            sort(vDists.begin(), vDists.end());
            int median = vDists[0.5 * (N - 1)];

            if (median < BestMedian)
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

    cv::Mat MapPoint::GetDescriptor()
    {
        unique_lock<mutex> lock(mMutexFeatures);
        return mDescriptor.clone();
    }

    tuple<int, int> MapPoint::GetIndexInKeyFrame(KeyFrame *pKF)
    {
        unique_lock<mutex> lock(mMutexFeatures);
        if (mObservations.count(pKF->mnId))
            return mObservations[pKF->mnId].projIndex;
        else
            return tuple<int, int>(-1, -1);
    }

    bool MapPoint::IsInKeyFrame(KeyFrame *pKF)
    {
        unique_lock<mutex> lock(mMutexFeatures);
        return (mObservations.count(pKF->mnId));
    }

    void MapPoint::UpdateNormalAndDepth()
    {
        map<unsigned long int, Observation> observations;
        KeyFrame *pRefKF;
        Eigen::Vector3f Pos;
        {
            unique_lock<mutex> lock1(mMutexFeatures);
            unique_lock<mutex> lock2(mMutexPos);
            if (mbBad)
                return;
            observations = mObservations;
            pRefKF = mpRefKF;
            Pos = mWorldPos;
        }

        if (observations.empty())
            return;

        Eigen::Vector3f normal;
        normal.setZero();
        int n = 0;
        for (auto &observation : observations)
        {
            KeyFrame *pKF = observation.second.projKeyframe;

            tuple<int, int> indexes = observation.second.projIndex;
            int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);

            if (leftIndex != -1)
            {
                Eigen::Vector3f Owi = pKF->GetCameraCenter();
                Eigen::Vector3f normali = Pos - Owi;
                normal = normal + normali / normali.norm();
                n++;
            }
            if (rightIndex != -1)
            {
                Eigen::Vector3f Owi = pKF->GetRightCameraCenter();
                Eigen::Vector3f normali = Pos - Owi;
                normal = normal + normali / normali.norm();
                n++;
            }
        }

        Eigen::Vector3f PC = Pos - pRefKF->GetCameraCenter();
        const float dist = PC.norm();

        tuple<int, int> indexes = observations[pRefKF->mnId].projIndex;
        int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);
        int level;
        if (pRefKF->NLeft == -1)
        {
            level = pRefKF->mvKeysUn[leftIndex].octave;
        }
        else if (leftIndex != -1)
        {
            level = pRefKF->mvKeys[leftIndex].octave;
        }
        else
        {
            level = pRefKF->mvKeysRight[rightIndex - pRefKF->NLeft].octave;
        }

        // const int level = pRefKF->mvKeysUn[observations[pRefKF]].octave;
        const float levelScaleFactor = pRefKF->mvScaleFactors[level];
        const int nLevels = pRefKF->mnScaleLevels;

        {
            unique_lock<mutex> lock3(mMutexPos);
            mfMaxDistance = dist * levelScaleFactor;
            mfMinDistance = mfMaxDistance / pRefKF->mvScaleFactors[nLevels - 1];
            mNormalVector = normal / n;
        }
    }

    void MapPoint::SetNormalVector(const Eigen::Vector3f &normal)
    {
        unique_lock<mutex> lock3(mMutexPos);
        mNormalVector = normal;
    }

    float MapPoint::GetMinDistanceInvariance()
    {
        unique_lock<mutex> lock(mMutexPos);
        return 0.8f * mfMinDistance;
    }

    float MapPoint::GetMaxDistanceInvariance()
    {
        unique_lock<mutex> lock(mMutexPos);
        return 1.2f * mfMaxDistance;
    }

    int MapPoint::PredictScale(const float &currentDist, KeyFrame *pKF)
    {
        float ratio;
        {
            unique_lock<mutex> lock(mMutexPos);
            ratio = mfMaxDistance / currentDist;
        }

        int nScale = ceil(log(ratio) / pKF->mfLogScaleFactor);
        if (nScale < 0)
            nScale = 0;
        else if (nScale >= pKF->mnScaleLevels)
            nScale = pKF->mnScaleLevels - 1;

        return nScale;
    }

    int MapPoint::PredictScale(const float &currentDist, Frame *pF)
    {
        float ratio;
        {
            unique_lock<mutex> lock(mMutexPos);
            ratio = mfMaxDistance / currentDist;
        }

        int nScale = ceil(log(ratio) / pF->mfLogScaleFactor);
        if (nScale < 0)
            nScale = 0;
        else if (nScale >= pF->mnScaleLevels)
            nScale = pF->mnScaleLevels - 1;

        return nScale;
    }

    void MapPoint::PrintObservations()
    {
        cout << "MP_OBS: MP " << mnId << endl;
        for (auto &mObservation : mObservations)
        {
            KeyFrame *pKFi = mObservation.second.projKeyframe;
            tuple<int, int> indexes = mObservation.second.projIndex;
            int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);
            cout << "--OBS in KF " << pKFi->mnId << " in map " << pKFi->GetMap()->GetId() << endl;
        }
    }

    Map *MapPoint::GetMap()
    {
        unique_lock<mutex> lock(mMutexMap);
        return mpMap;
    }

    void MapPoint::UpdateMap(Map *pMap)
    {
        unique_lock<mutex> lock(mMutexMap);
        mpMap = pMap;
    }

    void MapPoint::PreSave(std::map<long unsigned int, KeyFrame *> &spKF, std::map<long unsigned int, MapPoint *> &spMP)
    {
        mBackupReplacedId = -1;
        if (mpReplaced && spMP.find(mpReplaced->mnId) != spMP.end())
            mBackupReplacedId = mpReplaced->mnId;

        mBackupObservationsId1.clear();
        mBackupObservationsId2.clear();
        // Save the id and position in each KF who view it
        for (auto &mObservation : mObservations)
        {
            KeyFrame *pKFi = mObservation.second.projKeyframe;
            if (spKF.find(pKFi->mnId) != spKF.end())
            {
                mBackupObservationsId1[mObservation.first] = get<0>(mObservation.second.projIndex);
                mBackupObservationsId2[mObservation.first] = get<1>(mObservation.second.projIndex);
            }
            else
            {
                EraseObservation(pKFi);
            }
        }

        // Save the id of the reference KF
        if (spKF.find(mpRefKF->mnId) != spKF.end())
        {
            mBackupRefKFId = mpRefKF->mnId;
        }
    }

    void MapPoint::PostLoad(map<long unsigned int, KeyFrame *> &mpKFid, map<long unsigned int, MapPoint *> &mpMPid)
    {
        mpRefKF = mpKFid[mBackupRefKFId];
        if (!mpRefKF)
        {
            cout << "ERROR: MP without KF reference " << mBackupRefKFId << "; Num obs: " << nObs << endl;
        }
        mpReplaced = static_cast<MapPoint *>(nullptr);
        if (mBackupReplacedId >= 0)
        {
            auto it = mpMPid.find(mBackupReplacedId);
            if (it != mpMPid.end())
                mpReplaced = it->second;
        }

        mObservations.clear();

        for (auto it : mBackupObservationsId1)
        {
            KeyFrame *pKFi = mpKFid[it.first];
            map<long unsigned int, int>::const_iterator it2 = mBackupObservationsId2.find(it.first);
            std::tuple<int, int> indexes = tuple<int, int>(it.second, it2->second);
            if (pKFi)
            {
                mObservations[pKFi->mnId] = Observation(pKFi, indexes,
                                                        currentReferenceKeyframe,
                                                        currentReferenceKeypointIndex);
            }
        }

        mBackupObservationsId1.clear();
        mBackupObservationsId2.clear();
    }

} // namespace ORB_SLAM

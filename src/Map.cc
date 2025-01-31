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

#include "Map.h"

#include <mutex>

namespace ORB_SLAM3
{

    long unsigned int Map::nNextId = 0;

    Map::Map() : mpFirstRegionKF(static_cast<KeyFrame *>(nullptr)), mbFail(false), mbImuInitialized(false), mnMapChange(0), mnMapChangeNotified(0),
                 mnMaxKFid(0), mnBigChangeIdx(0), mIsInUse(false), mHasTumbnail(false), mbBad(false), mbIsInertial(false), mbIMU_BA1(false), mbIMU_BA2(false)
    {
        mnId = nNextId++;
        mThumbnail = static_cast<GLubyte *>(nullptr);
    }

    Map::Map(int initKFid) : mpFirstRegionKF(static_cast<KeyFrame *>(nullptr)), mbFail(false), /*mnLastLoopKFid(initKFid),*/ mbImuInitialized(false), mnMapChange(0),
                             mnMapChangeNotified(0), mnInitKFid(initKFid), mnMaxKFid(initKFid), mnBigChangeIdx(0),
                             mIsInUse(false), mHasTumbnail(false), mbBad(false), mbIsInertial(false), mbIMU_BA1(false), mbIMU_BA2(false)
    {
        mnId = nNextId++;
        mThumbnail = static_cast<GLubyte *>(nullptr);
    }

    Map::~Map()
    {
        // TODO: erase all points from memory
        mspMapPoints.clear();

        // TODO: erase all keyframes from memory
        mspKeyFrames.clear();

        if (mThumbnail)
            delete mThumbnail;
        mThumbnail = static_cast<GLubyte *>(nullptr);

        mvpReferenceMapPoints.clear();
        mvpKeyFrameOrigins.clear();
    }

    void Map::AddKeyFrame(KeyFrame *pKF)
    {
        unique_lock<mutex> lock(mMutexMap);
        if (mspKeyFrames.empty())
        {
            cout << "First KF:" << pKF->mnId << "; Map init KF:" << mnInitKFid << endl;
            mnInitKFid = pKF->mnId;
            mpKFinitial = pKF;
            mpKFlowerID = pKF;
        }
        mspKeyFrames.insert(make_pair(pKF->mnId, pKF));
        if (pKF->mnId > mnMaxKFid)
        {
            mnMaxKFid = pKF->mnId;
        }
        if (pKF->mnId < mpKFlowerID->mnId)
        {
            mpKFlowerID = pKF;
        }
    }

    void Map::AddMapPoint(MapPoint *pMP)
    {
        unique_lock<mutex> lock(mMutexMap);
        mspMapPoints.insert(make_pair(pMP->mnId, pMP));
    }

    void Map::SetImuInitialized()
    {
        unique_lock<mutex> lock(mMutexMap);
        mbImuInitialized = true;
    }

    bool Map::isImuInitialized()
    {
        unique_lock<mutex> lock(mMutexMap);
        return mbImuInitialized;
    }

    void Map::EraseMapPoint(MapPoint *pMP)
    {
        // unique_lock<mutex> lock(mMutexMap);
        mspMapPoints.erase(pMP->mnId);

        // TODO: This only erase the pointer.
        // Delete the MapPoint
    }

    void Map::EraseKeyFrame(KeyFrame *pKF)
    {
        unique_lock<mutex> lock(mMutexMap);
        mspKeyFrames.erase(pKF->mnId);
        if (!mspKeyFrames.empty())
        {
            if (pKF->mnId == mpKFlowerID->mnId)
            {
                vector<KeyFrame *> vpKFs;
                for (pair<const unsigned long, KeyFrame *> pkf : mspKeyFrames)
                {
                    vpKFs.push_back(pkf.second);
                }
                sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);
                mpKFlowerID = vpKFs[0];
            }
        }
        else
        {
            mpKFlowerID = 0;
        }

        // TODO: This only erase the pointer.
        // Delete the MapPoint
    }

    void Map::SetReferenceMapPoints(const vector<MapPoint *> &vpMPs)
    {
        unique_lock<mutex> lock(mMutexMap);
        mvpReferenceMapPoints = vpMPs;
    }

    void Map::InformNewBigChange()
    {
        unique_lock<mutex> lock(mMutexMap);
        mnBigChangeIdx++;
    }

    int Map::GetLastBigChangeIdx()
    {
        unique_lock<mutex> lock(mMutexMap);
        return mnBigChangeIdx;
    }

    vector<KeyFrame *> Map::GetAllKeyFrames()
    {
        unique_lock<mutex> lock(mMutexMap);
        vector<KeyFrame *> vpKFs;
        for (pair<const unsigned long, KeyFrame *> pkf : mspKeyFrames)
        {
            vpKFs.push_back(pkf.second);
        }
        return vpKFs;
    }

    vector<MapPoint *> Map::GetAllMapPoints()
    {
        unique_lock<mutex> lock(mMutexMap);
        vector<MapPoint *> vpKFs;
        for (pair<const unsigned long, MapPoint *> pkf : mspMapPoints)
        {
            vpKFs.push_back(pkf.second);
        }
        return vpKFs;
    }

    long unsigned int Map::MapPointsInMap()
    {
        unique_lock<mutex> lock(mMutexMap);
        return mspMapPoints.size();
    }

    long unsigned int Map::KeyFramesInMap()
    {
        unique_lock<mutex> lock(mMutexMap);
        return mspKeyFrames.size();
    }

    vector<MapPoint *> Map::GetReferenceMapPoints()
    {
        unique_lock<mutex> lock(mMutexMap);
        return mvpReferenceMapPoints;
    }

    long unsigned int Map::GetId()
    {
        return mnId;
    }
    long unsigned int Map::GetInitKFid()
    {
        unique_lock<mutex> lock(mMutexMap);
        return mnInitKFid;
    }

    void Map::SetInitKFid(long unsigned int initKFif)
    {
        unique_lock<mutex> lock(mMutexMap);
        mnInitKFid = initKFif;
    }

    long unsigned int Map::GetMaxKFid()
    {
        unique_lock<mutex> lock(mMutexMap);
        return mnMaxKFid;
    }

    KeyFrame *Map::GetOriginKF()
    {
        return mpKFinitial;
    }

    void Map::SetCurrentMap()
    {
        mIsInUse = true;
    }

    void Map::SetStoredMap()
    {
        mIsInUse = false;
    }

    void Map::clear()
    {
        //    for(set<MapPoint*>::iterator sit=mspMapPoints.begin(), send=mspMapPoints.end(); sit!=send; sit++)
        //        delete *sit;

        for (auto &mspKeyFrame : mspKeyFrames)
        {
            KeyFrame *pKF = mspKeyFrame.second;
            pKF->UpdateMap(static_cast<Map *>(nullptr));
            //        delete *sit;
        }

        mspMapPoints.clear();
        mspKeyFrames.clear();
        mnMaxKFid = mnInitKFid;
        mbImuInitialized = false;
        mvpReferenceMapPoints.clear();
        mvpKeyFrameOrigins.clear();
        mbIMU_BA1 = false;
        mbIMU_BA2 = false;
    }

    bool Map::IsInUse()
    {
        return mIsInUse;
    }

    void Map::SetBad()
    {
        mbBad = true;
    }

    bool Map::IsBad()
    {
        return mbBad;
    }

    void Map::ApplyScaledRotation(const Sophus::SE3f &T, const float s, const bool bScaledVel)
    {
        unique_lock<mutex> lock(mMutexMap);

        // Body position (IMU) of first keyframe is fixed to (0,0,0)
        Sophus::SE3f Tyw = T;
        Eigen::Matrix3f Ryw = Tyw.rotationMatrix();
        Eigen::Vector3f tyw = Tyw.translation();

        for (auto &mspKeyFrame : mspKeyFrames)
        {
            KeyFrame *pKF = mspKeyFrame.second;
            Sophus::SE3f Twc = pKF->GetPoseInverse();
            Twc.translation() *= s;
            Sophus::SE3f Tyc = Tyw * Twc;
            Sophus::SE3f Tcy = Tyc.inverse();
            pKF->SetPose(Tcy);
            Eigen::Vector3f Vw = pKF->GetVelocity();
            if (!bScaledVel)
                pKF->SetVelocity(Ryw * Vw);
            else
                pKF->SetVelocity(Ryw * Vw * s);
        }
        for (auto &mspMapPoint : mspMapPoints)
        {
            MapPoint *pMP = mspMapPoint.second;
            pMP->SetWorldPos(s * Ryw * pMP->GetWorldPos() + tyw);
            pMP->UpdateNormalAndDepth();
        }
        mnMapChange++;
    }

    void Map::SetInertialSensor()
    {
        unique_lock<mutex> lock(mMutexMap);
        mbIsInertial = true;
    }

    bool Map::IsInertial()
    {
        unique_lock<mutex> lock(mMutexMap);
        return mbIsInertial;
    }

    void Map::SetIniertialBA1()
    {
        unique_lock<mutex> lock(mMutexMap);
        mbIMU_BA1 = true;
    }

    void Map::SetIniertialBA2()
    {
        unique_lock<mutex> lock(mMutexMap);
        mbIMU_BA2 = true;
    }

    bool Map::GetIniertialBA1()
    {
        unique_lock<mutex> lock(mMutexMap);
        return mbIMU_BA1;
    }

    bool Map::GetIniertialBA2()
    {
        unique_lock<mutex> lock(mMutexMap);
        return mbIMU_BA2;
    }

    void Map::ChangeId(long unsigned int nId)
    {
        mnId = nId;
    }

    unsigned int Map::GetLowerKFID()
    {
        unique_lock<mutex> lock(mMutexMap);
        if (mpKFlowerID)
        {
            return mpKFlowerID->mnId;
        }
        return 0;
    }

    int Map::GetMapChangeIndex()
    {
        unique_lock<mutex> lock(mMutexMap);
        return mnMapChange;
    }

    void Map::IncreaseChangeIndex()
    {
        unique_lock<mutex> lock(mMutexMap);
        mnMapChange++;
    }

    int Map::GetLastMapChange()
    {
        unique_lock<mutex> lock(mMutexMap);
        return mnMapChangeNotified;
    }

    void Map::SetLastMapChange(int currentChangeId)
    {
        unique_lock<mutex> lock(mMutexMap);
        mnMapChangeNotified = currentChangeId;
    }

    void Map::PreSave(std::set<GeometricCamera *> &spCams)
    {
        int nMPWithoutObs = 0;
        for (pair<const unsigned long, MapPoint *> pMPi : mspMapPoints)
        {
            if (!pMPi.second || pMPi.second->isBad())
                continue;

            if (pMPi.second->GetObservations().empty())
            {
                nMPWithoutObs++;
            }
            map<unsigned long, Observation> mpObs = pMPi.second->GetObservations();
            for (auto &mpOb : mpObs)
            {
                if (mpOb.second.projKeyframe->GetMap() != this || mpOb.second.projKeyframe->isBad())
                {
                    pMPi.second->EraseObservation(mpOb.second.projKeyframe);
                }
            }
        }

        // Saves the id of KF origins
        mvBackupKeyFrameOriginsId.clear();
        mvBackupKeyFrameOriginsId.reserve(mvpKeyFrameOrigins.size());
        for (auto &mvpKeyFrameOrigin : mvpKeyFrameOrigins)
        {
            mvBackupKeyFrameOriginsId.push_back(mvpKeyFrameOrigin->mnId);
        }

        // Backup of MapPoints
        mvpBackupMapPoints.clear();
        for (pair<const unsigned long, MapPoint *> pMPi : mspMapPoints)
        {
            if (!pMPi.second || pMPi.second->isBad())
                continue;

            mvpBackupMapPoints.push_back(pMPi.second);
            pMPi.second->PreSave(mspKeyFrames, mspMapPoints);
        }

        // Backup of KeyFrames
        mvpBackupKeyFrames.clear();
        for (pair<const unsigned long, KeyFrame *> pKFi : mspKeyFrames)
        {
            if (!pKFi.second || pKFi.second->isBad())
                continue;

            mvpBackupKeyFrames.push_back(pKFi.second);
            pKFi.second->PreSave(mspKeyFrames, mspMapPoints, spCams);
        }

        mnBackupKFinitialID = -1;
        if (mpKFinitial)
        {
            mnBackupKFinitialID = mpKFinitial->mnId;
        }

        mnBackupKFlowerID = -1;
        if (mpKFlowerID)
        {
            mnBackupKFlowerID = mpKFlowerID->mnId;
        }
    }

    void Map::PostLoad(KeyFrameDatabase *pKFDB, ORBVocabulary *pORBVoc /*, map<long unsigned int, KeyFrame*>& mpKeyFrameId*/, map<unsigned int, GeometricCamera *> &mpCams)
    {
        for (MapPoint *mp : mvpBackupMapPoints)
        {
            mspMapPoints[mp->mnId] = mp;
        }
        for (KeyFrame *mp : mvpBackupKeyFrames)
        {
            mspKeyFrames[mp->mnId] = mp;
        }

        map<long unsigned int, MapPoint *> mpMapPointId;
        for (pair<const unsigned long, MapPoint *> pMPi : mspMapPoints)
        {
            if (!pMPi.second || pMPi.second->isBad())
                continue;

            pMPi.second->UpdateMap(this);
            mpMapPointId[pMPi.first] = pMPi.second;
        }

        map<long unsigned int, KeyFrame *> mpKeyFrameId;
        for (pair<const unsigned long, KeyFrame *> pKFi : mspKeyFrames)
        {
            if (!pKFi.second || pKFi.second->isBad())
                continue;

            pKFi.second->UpdateMap(this);
            pKFi.second->SetORBVocabulary(pORBVoc);
            pKFi.second->SetKeyFrameDatabase(pKFDB);
            mpKeyFrameId[pKFi.first] = pKFi.second;
        }

        // References reconstruction between different instances
        for (pair<const unsigned long, MapPoint *> pMPi : mspMapPoints)
        {
            if (!pMPi.second || pMPi.second->isBad())
                continue;

            pMPi.second->PostLoad(mpKeyFrameId, mpMapPointId);
        }

        for (pair<const unsigned long, KeyFrame *> pKFi : mspKeyFrames)
        {
            if (!pKFi.second || pKFi.second->isBad())
                continue;

            pKFi.second->PostLoad(mpKeyFrameId, mpMapPointId, mpCams);
            pKFDB->add(pKFi.second);
        }

        if (mnBackupKFinitialID != -1)
        {
            mpKFinitial = mpKeyFrameId[mnBackupKFinitialID];
        }

        if (mnBackupKFlowerID != -1)
        {
            mpKFlowerID = mpKeyFrameId[mnBackupKFlowerID];
        }

        mvpKeyFrameOrigins.clear();
        mvpKeyFrameOrigins.reserve(mvBackupKeyFrameOriginsId.size());
        for (unsigned long i : mvBackupKeyFrameOriginsId)
        {
            mvpKeyFrameOrigins.push_back(mpKeyFrameId[i]);
        }

        mvpBackupMapPoints.clear();
    }

} // namespace ORB_SLAM3

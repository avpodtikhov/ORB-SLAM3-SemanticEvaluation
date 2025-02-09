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

#include "Frame.h"

#include "G2oTypes.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "ORBextractor.h"
#include "Converter.h"
#include "ORBmatcher.h"
#include "GeometricCamera.h"

#include <thread>
#include <utility>
#include <include/CameraModels/Pinhole.h>
#include <include/CameraModels/KannalaBrandt8.h>

#include "MapLine.h"
#include "LineMatcher.h"

namespace ORB_SLAM3
{

    long unsigned int Frame::nNextId = 0;
    bool Frame::mbInitialComputations = true;
    float Frame::cx, Frame::cy, Frame::fx, Frame::fy, Frame::invfx, Frame::invfy;
    float Frame::mnMinX, Frame::mnMinY, Frame::mnMaxX, Frame::mnMaxY;
    float Frame::mfGridElementWidthInv, Frame::mfGridElementHeightInv;

    // For stereo fisheye matching
    cv::BFMatcher Frame::BFmatcher = cv::BFMatcher(cv::NORM_HAMMING);

    Frame::Frame() : mpcpi(nullptr), mbHasPose(false), mbHasVelocity(false), mpImuPreintegrated(nullptr), mpPrevFrame(nullptr), mpImuPreintegratedFrame(nullptr), mpReferenceKF(static_cast<KeyFrame *>(nullptr)), mbIsSet(false), mbImuPreintegrated(false)
    {
#ifdef REGISTER_TIMES
        mTimeStereoMatch = 0;
        mTimeORB_Ext = 0;
        mTimeLine_Ext = 0;
        mTimeStereoMatch_Lines = 0;
#endif
    }
    void Frame::copyPoseData(const Frame& frame) {
        mTcw = frame.mTcw;
        mTlr = frame.mTlr;
        mTrl = frame.mTrl;
        mRlr = frame.mRlr;
        mtlr = frame.mtlr;
        mbHasPose = false;
        mbHasVelocity = false;
        mVw = frame.mVw;
    }

    void Frame::copyCameraParameters(const Frame& frame) {
        mK = frame.mK.clone();
        mK_ = Converter::toMatrix3f(frame.mK);
        mDistCoef = frame.mDistCoef.clone();
        mbf = frame.mbf;
        mb = frame.mb;
        mThDepth = frame.mThDepth;
        mpCamera = frame.mpCamera;
        mpCamera2 = frame.mpCamera2;
    }

    void Frame::copyFeatureExtractors(const Frame& frame) {
        mpORBvocabulary = frame.mpORBvocabulary;
        mpORBextractorLeft = frame.mpORBextractorLeft;
        mpORBextractorRight = frame.mpORBextractorRight;
        mpLineVocabulary = frame.mpLineVocabulary;
        mpLineExtractorLeft = frame.mpLineExtractorLeft;
        mpLineExtractorRight = frame.mpLineExtractorRight;
        mTimeStamp = frame.mTimeStamp;
    }

    void Frame::copyPointFeatures(const Frame& frame) {
        N = frame.N;
        mvKeys = frame.mvKeys;
        mvKeysRight = frame.mvKeysRight;
        mvKeysUn = frame.mvKeysUn;
        mvpMapPoints = frame.mvpMapPoints;
        mvuRight = frame.mvuRight;
        mvDepth = frame.mvDepth;
        mDescriptors = frame.mDescriptors.clone();
        mDescriptorsRight = frame.mDescriptorsRight.clone();
        mvbOutlier = frame.mvbOutlier;
        mnCloseMPs = frame.mnCloseMPs;
    }

    void Frame::copyLineFeatures(const Frame& frame) {
        N_Lines = frame.N_Lines;
        mvKeysLine = frame.mvKeysLine;
        mvKeysLineRight = frame.mvKeysLineRight;
        mvKeysUnLines = frame.mvKeysUnLines;
        mvpMapLines = frame.mvpMapLines;
        mvDepthLine = frame.mvDepthLine;
        mDescriptorsLine = frame.mDescriptorsLine.clone();
        mDescriptorsLineRight = frame.mDescriptorsLineRight.clone();
        mvbOutlierLine = frame.mvbOutlierLine;
        mnCloseMLs = frame.mnCloseMLs;
        mvle_Lines = frame.mvle_Lines;
    }

    void Frame::copyScaleParameters(const Frame& frame) {
        mnScaleLevels = frame.mnScaleLevels;
        mnScaleLevelsLine = frame.mnScaleLevelsLine;
        mfScaleFactor = frame.mfScaleFactor;
        mfLogScaleFactor = frame.mfLogScaleFactor;
        mvScaleFactors = frame.mvScaleFactors;
        mvScaleFactorsLine = frame.mvScaleFactorsLine;
        mvInvScaleFactors = frame.mvInvScaleFactors;
        mvInvScaleFactorsLine = frame.mvInvScaleFactorsLine;
        mvLevelSigma2 = frame.mvLevelSigma2;
        mvLevelSigma2Line = frame.mvLevelSigma2Line;
        mvInvLevelSigma2 = frame.mvInvLevelSigma2;
        mvInvLevelSigma2Line = frame.mvInvLevelSigma2Line;
    }

    void Frame::copySemanticData(const Frame& frame) {
        mvKeysMoving = frame.mvKeysMoving;
        mvSemanticCls = frame.mvSemanticCls;
        mvInstanceCls = frame.mvInstanceCls;
        semantic_meta = frame.semantic_meta;
    }

    void Frame::copyIMUData(const Frame& frame) {
        mImuBias = frame.mImuBias;
        mImuCalib = frame.mImuCalib;
        mpImuPreintegrated = frame.mpImuPreintegrated;
        mpLastKeyFrame = frame.mpLastKeyFrame;
        mpPrevFrame = frame.mpPrevFrame;
        mpImuPreintegratedFrame = frame.mpImuPreintegratedFrame;
        mbImuPreintegrated = frame.mbImuPreintegrated;
        mpMutexImu = frame.mpMutexImu;
    }

    void Frame::copyBoWData(const Frame& frame) {
        mBowVec = frame.mBowVec;
        mFeatVec = frame.mFeatVec;
        mpcpi = frame.mpcpi;
        mnId = frame.mnId;
        mpReferenceKF = frame.mpReferenceKF;
        mNameFile = frame.mNameFile;
        mnDataset = frame.mnDataset;
        mbIsSet = frame.mbIsSet;
        Nleft = frame.Nleft;
        Nright = frame.Nright;
        monoLeft = frame.monoLeft;
        monoRight = frame.monoRight;
        mvLeftToRightMatch = frame.mvLeftToRightMatch;
        mvRightToLeftMatch = frame.mvRightToLeftMatch;
        mvStereo3Dpoints = frame.mvStereo3Dpoints;
        inv_width = frame.inv_width;
        inv_height = frame.inv_height;
    }

    // Copy Constructor
    Frame::Frame(const Frame &frame)
    {
        copyPoseData(frame);
        copyCameraParameters(frame);
        copyFeatureExtractors(frame);
        copyPointFeatures(frame);
        copyLineFeatures(frame);
        copyScaleParameters(frame);
        copySemanticData(frame);
        copyIMUData(frame);
        copyBoWData(frame);

        for (int i = 0; i < FRAME_GRID_COLS; i++)
            for (int j = 0; j < FRAME_GRID_ROWS; j++)
            {
                mGrid[i][j] = frame.mGrid[i][j];
                if (frame.Nleft > 0)
                {
                    mGridRight[i][j] = frame.mGridRight[i][j];
                }
            }

        if (frame.mbHasPose)
            SetPose(frame.GetPose());

        if (frame.HasVelocity())
        {
            SetVelocity(frame.GetVelocity());
        }

        mmProjectPoints = frame.mmProjectPoints;
        mmMatchedInImage = frame.mmMatchedInImage;

#ifdef REGISTER_TIMES
        mTimeStereoMatch = frame.mTimeStereoMatch;
        mTimeORB_Ext = frame.mTimeORB_Ext;
        mTimeLine_Ext = frame.mTimeLine_Ext;
        mTimeStereoMatch_Lines = frame.mTimeStereoMatch_Lines;
#endif
    }

    Frame::Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, ORBextractor *extractorLeft, ORBextractor *extractorRight, ORBVocabulary *voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth, GeometricCamera *pCamera, Frame *pPrevF, const IMU::Calib &ImuCalib)
        : mpcpi(nullptr), mbHasPose(false), mbHasVelocity(false), mpORBvocabulary(voc), mpORBextractorLeft(extractorLeft), mpORBextractorRight(extractorRight), mTimeStamp(timeStamp), mK(K.clone()), mK_(Converter::toMatrix3f(K)), mDistCoef(distCoef.clone()),
          mbf(bf), mThDepth(thDepth), mImuCalib(ImuCalib), mpImuPreintegrated(nullptr), mpPrevFrame(pPrevF), mpImuPreintegratedFrame(nullptr), mpReferenceKF(static_cast<KeyFrame *>(nullptr)),
          mbIsSet(false), mbImuPreintegrated(false), mpCamera(pCamera), mpCamera2(nullptr)
    {
        // Frame ID
        mnId = nNextId++;

        // Scale Level Info
        mnScaleLevels = mpORBextractorLeft->GetLevels();
        mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
        mfLogScaleFactor = log(mfScaleFactor);
        mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
        mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
        mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
        mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

        // ORB extraction
#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_StartExtORB = std::chrono::steady_clock::now();
#endif
        thread threadLeft(&Frame::ExtractORB, this, 0, imLeft, 0, 0);
        thread threadRight(&Frame::ExtractORB, this, 1, imRight, 0, 0);
        threadLeft.join();
        threadRight.join();
#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_EndExtORB = std::chrono::steady_clock::now();

        mTimeORB_Ext = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(time_EndExtORB - time_StartExtORB).count();
#endif

        N = mvKeys.size();
        if (mvKeys.empty())
            return;

        UndistortKeyPoints();

#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_StartStereoMatches = std::chrono::steady_clock::now();
#endif
        ComputeStereoMatches();
#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_EndStereoMatches = std::chrono::steady_clock::now();

        mTimeStereoMatch = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(time_EndStereoMatches - time_StartStereoMatches).count();
#endif

        mvpMapPoints = vector<MapPoint *>(N, static_cast<MapPoint *>(NULL));
        mvbOutlier = vector<bool>(N, false);
        mmProjectPoints.clear();
        mmMatchedInImage.clear();

        // This is done only for the first Frame (or after a change in the calibration)
        if (mbInitialComputations)
        {
            ComputeImageBounds(imLeft);

            mfGridElementWidthInv = static_cast<float>(FRAME_GRID_COLS) / (mnMaxX - mnMinX);
            mfGridElementHeightInv = static_cast<float>(FRAME_GRID_ROWS) / (mnMaxY - mnMinY);

            fx = K.at<float>(0, 0);
            fy = K.at<float>(1, 1);
            cx = K.at<float>(0, 2);
            cy = K.at<float>(1, 2);
            invfx = 1.0f / fx;
            invfy = 1.0f / fy;

            mbInitialComputations = false;
        }

        mb = mbf / fx;

        if (pPrevF)
        {
            if (pPrevF->HasVelocity())
                SetVelocity(pPrevF->GetVelocity());
        }
        else
        {
            mVw.setZero();
        }

        mpMutexImu = new std::mutex();

        // Set no stereo fisheye information
        Nleft = -1;
        Nright = -1;
        mvLeftToRightMatch = vector<int>(0);
        mvRightToLeftMatch = vector<int>(0);
        mvStereo3Dpoints = vector<Eigen::Vector3f>(0);
        monoLeft = -1;
        monoRight = -1;

        AssignFeaturesToGrid();
    }

    Frame::Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const cv::Mat &imLeftSem, const unordered_map<int, bool> &seg_meta, const double &timeStamp, ORBextractor *extractorLeft, ORBextractor *extractorRight, ORBVocabulary *voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth, GeometricCamera *pCamera, Frame *pPrevF, const IMU::Calib &ImuCalib, const bool moving_flag, const bool dynamic_flag, const bool semantic_flag, const bool instance_flag)
        : mpcpi(nullptr), mbHasPose(false), mbHasVelocity(false), mpORBvocabulary(voc), mpORBextractorLeft(extractorLeft), mpORBextractorRight(extractorRight), mTimeStamp(timeStamp), mK(K.clone()), mK_(Converter::toMatrix3f(K)), mDistCoef(distCoef.clone()),
          mbf(bf), mThDepth(thDepth), mImuCalib(ImuCalib), mpImuPreintegrated(nullptr), mpPrevFrame(pPrevF), mpImuPreintegratedFrame(nullptr), mpReferenceKF(static_cast<KeyFrame *>(nullptr)),
          mbIsSet(false), mbImuPreintegrated(false), mpCamera(pCamera), mpCamera2(nullptr)
    {
        inv_width  = FRAME_GRID_COLS / static_cast<double>(imLeft.cols);
        inv_height = FRAME_GRID_ROWS / static_cast<double>(imRight.rows);
        semantic_meta = seg_meta;
        // Frame ID
        mnId = nNextId++;

        // Scale Level Info
        mnScaleLevels = mpORBextractorLeft->GetLevels();
        mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
        mfLogScaleFactor = log(mfScaleFactor);
        mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
        mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
        mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
        mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

        // ORB extraction
#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_StartExtORB = std::chrono::steady_clock::now();
#endif
        // thread threadLeft(&Frame::ExtractORBSem, this, 0, imLeft, 0, 0, imLeftSem);
        thread threadLeft(&Frame::ExtractORB, this, 0, imLeft, 0, 0);
        thread threadRight(&Frame::ExtractORB, this, 1, imRight, 0, 0);
        threadLeft.join();
        threadRight.join();
#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_EndExtORB = std::chrono::steady_clock::now();

        mTimeORB_Ext = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(time_EndExtORB - time_StartExtORB).count();
#endif

        processSemanticKeyPoints(imLeftSem, dynamic_flag);

        N = mvKeys.size();
        if (mvKeys.empty())
            return;

        N = mvKeys.size();
        if (mvKeys.empty())
            return;

        UndistortKeyPoints();

#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_StartStereoMatches = std::chrono::steady_clock::now();
#endif
        ComputeStereoMatches();
#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_EndStereoMatches = std::chrono::steady_clock::now();

        mTimeStereoMatch = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(time_EndStereoMatches - time_StartStereoMatches).count();
#endif

        mvpMapPoints = vector<MapPoint *>(N, static_cast<MapPoint *>(nullptr));
        mvbOutlier = vector<bool>(N, false);
        mmProjectPoints.clear();
        mmMatchedInImage.clear();
        
        mvSemanticCls = vector<int>(N, 0);
        mvInstanceCls = vector<int>(N, 0);
        mvKeysMoving = vector<bool>(N, false);

        for (int i = 0; i < mvKeys.size(); i++) {
            updateSemanticInfo(mvKeys[i], imLeftSem);
        }

        // This is done only for the first Frame (or after a change in the calibration)
        if (mbInitialComputations)
        {
            ComputeImageBounds(imLeft);

            mfGridElementWidthInv = static_cast<float>(FRAME_GRID_COLS) / (mnMaxX - mnMinX);
            mfGridElementHeightInv = static_cast<float>(FRAME_GRID_ROWS) / (mnMaxY - mnMinY);

            fx = K.at<float>(0, 0);
            fy = K.at<float>(1, 1);
            cx = K.at<float>(0, 2);
            cy = K.at<float>(1, 2);
            invfx = 1.0f / fx;
            invfy = 1.0f / fy;

            mbInitialComputations = false;
        }

        mb = mbf / fx;

        if (pPrevF)
        {
            if (pPrevF->HasVelocity())
                SetVelocity(pPrevF->GetVelocity());
        }
        else
        {
            mVw.setZero();
        }

        mpMutexImu = new std::mutex();

        // Set no stereo fisheye information
        Nleft = -1;
        Nright = -1;
        mvLeftToRightMatch = vector<int>(0);
        mvRightToLeftMatch = vector<int>(0);
        mvStereo3Dpoints = vector<Eigen::Vector3f>(0);
        monoLeft = -1;
        monoRight = -1;

        AssignFeaturesToGrid();
    }

    //  Semantic Stereo Frame with Lines
    Frame::Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const cv::Mat &imLeftSem, const unordered_map<int, bool> &seg_meta, const double &timeStamp, ORBextractor *extractorLeft, ORBextractor *extractorRight,
    Lineextractor* LineExtractorLeft, Lineextractor* LineextractorRight, ORBVocabulary *voc, LineVocabulary* voc_line,
    cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth, GeometricCamera *pCamera, Frame *pPrevF, const IMU::Calib &ImuCalib, const bool moving_flag, const bool dynamic_flag, const bool semantic_flag, const bool instance_flag)
        : mpcpi(nullptr), mbHasPose(false), mbHasVelocity(false), mpORBvocabulary(voc), mpLineVocabulary(voc_line), mpORBextractorLeft(extractorLeft), mpORBextractorRight(extractorRight), mpLineExtractorLeft(LineExtractorLeft), mpLineExtractorRight(LineExtractorRight), mTimeStamp(timeStamp), mK(K.clone()), mK_(Converter::toMatrix3f(K)), mDistCoef(distCoef.clone()),
          mbf(bf), mThDepth(thDepth), mImuCalib(ImuCalib), mpImuPreintegrated(nullptr), mpPrevFrame(pPrevF), mpImuPreintegratedFrame(nullptr), mpReferenceKF(static_cast<KeyFrame *>(nullptr)),
          mbIsSet(false), mbImuPreintegrated(false), mpCamera(pCamera), mpCamera2(nullptr)
    {
        semantic_meta = seg_meta;
        // Frame ID
        mnId = nNextId++;

        // Scale Level Info
        mnScaleLevels = mpORBextractorLeft->GetLevels();
        mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
        mfLogScaleFactor = log(mfScaleFactor);
        mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
        mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
        mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
        mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

        mnScaleLevelsLine = mpLineExtractorLeft->GetLevels();
        mvScaleFactorsLine = mpLineExtractorLeft->GetScaleFactors();
        mvInvScaleFactorsLine = mpLineExtractorLeft->GetInverseScaleFactors();
        mvLevelSigma2Line =  mpLineExtractorLeft->GetScaleSigmaSquares();
        mvInvLevelSigma2Line = mpLineExtractorLeft->GetInverseScaleSigmaSquares();

        // ORB extraction
#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_StartExtORB = std::chrono::steady_clock::now();
#endif
        // thread threadLeft(&Frame::ExtractORBSem, this, 0, imLeft, 0, 0, imLeftSem);
        thread threadLeft(&Frame::ExtractORB, this, 0, imLeft, 0, 0);
        thread threadRight(&Frame::ExtractORB, this, 1, imRight, 0, 0);
        threadLeft.join();
        threadRight.join();
#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_EndExtORB = std::chrono::steady_clock::now();

        mTimeORB_Ext = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(time_EndExtORB - time_StartExtORB).count();
#endif
        processSemanticKeyPoints(imLeftSem, dynamic_flag);

        N = mvKeys.size();
        if (mvKeys.empty())
            return;

        N = mvKeys.size();
        if (mvKeys.empty())
            return;

        UndistortKeyPoints();

#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_StartStereoMatches = std::chrono::steady_clock::now();
#endif
        ComputeStereoMatches();
#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_EndStereoMatches = std::chrono::steady_clock::now();

        mTimeStereoMatch = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(time_EndStereoMatches - time_StartStereoMatches).count();
#endif
        mvpMapPoints = vector<MapPoint *>(N, static_cast<MapPoint *>(nullptr));
        mvbOutlier = vector<bool>(N, false);
        mmProjectPoints.clear();
        mmMatchedInImage.clear();


        mvSemanticCls = vector<int>(N, 0);
        mvInstanceCls = vector<int>(N, 0);
        mvKeysMoving = vector<bool>(N, false);

        for (int i = 0; i < mvKeys.size(); i++) {
            updateSemanticInfo(mvKeys[i], imLeftSem);
        }
#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_StartExtLine = std::chrono::steady_clock::now();
#endif
        thread threadLeft_Line(&Frame::ExtractLine,this,0,imLeft);
        thread threadRight_Line(&Frame::ExtractLine,this,1,imRight);
        threadLeft_Line.join();
        threadRight_Line.join();
#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_EndExtLine = std::chrono::steady_clock::now();

        mTimeLine_Ext = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(time_EndExtLine - time_StartExtLine).count();
#endif
        N_Lines = mvKeysLine.size();
       
        UndistortKeyLines();

#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_StartStereoMatches_Lines = std::chrono::steady_clock::now();
#endif
        ComputeStereoMatches_Lines();
#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_EndStereoMatches_Lines = std::chrono::steady_clock::now();

        mTimeStereoMatch_Lines = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(time_EndStereoMatches_Lines - time_StartStereoMatches_Lines).count();
#endif
        mvpMapLines = vector<MapLine*>(N_Lines,static_cast<MapLine*>(nullptr));
        mvbOutlierLine = vector<bool>(N_Lines,false);

        // This is done only for the first Frame (or after a change in the calibration)
        if (mbInitialComputations)
        {
            ComputeImageBounds(imLeft);

            mfGridElementWidthInv = static_cast<float>(FRAME_GRID_COLS) / (mnMaxX - mnMinX);
            mfGridElementHeightInv = static_cast<float>(FRAME_GRID_ROWS) / (mnMaxY - mnMinY);

            fx = K.at<float>(0, 0);
            fy = K.at<float>(1, 1);
            cx = K.at<float>(0, 2);
            cy = K.at<float>(1, 2);
            invfx = 1.0f / fx;
            invfy = 1.0f / fy;

            mbInitialComputations = false;
        }

        mb = mbf / fx;

        if (pPrevF)
        {
            if (pPrevF->HasVelocity())
                SetVelocity(pPrevF->GetVelocity());
        }
        else
        {
            mVw.setZero();
        }

        mpMutexImu = new std::mutex();

        // Set no stereo fisheye information
        Nleft = -1;
        Nright = -1;
        mvLeftToRightMatch = vector<int>(0);
        mvRightToLeftMatch = vector<int>(0);
        mvStereo3Dpoints = vector<Eigen::Vector3f>(0);
        monoLeft = -1;
        monoRight = -1;

        AssignFeaturesToGrid();
    }

    Frame::Frame(const cv::Mat &imGray, const cv::Mat &imDepth, const double &timeStamp, ORBextractor *extractor, ORBVocabulary *voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth, GeometricCamera *pCamera, Frame *pPrevF, const IMU::Calib &ImuCalib)
        : mpcpi(nullptr), mbHasPose(false), mbHasVelocity(false), mpORBvocabulary(voc),
          mpORBextractorLeft(extractor), mpORBextractorRight(static_cast<ORBextractor *>(nullptr)), mTimeStamp(timeStamp), mK(K.clone()), mK_(Converter::toMatrix3f(K)), mDistCoef(distCoef.clone()),
          mbf(bf), mThDepth(thDepth), mImuCalib(ImuCalib), mpImuPreintegrated(nullptr), mpPrevFrame(pPrevF), mpImuPreintegratedFrame(nullptr), mpReferenceKF(static_cast<KeyFrame *>(nullptr)),
          mbIsSet(false), mbImuPreintegrated(false), mpCamera(pCamera), mpCamera2(nullptr)
    {
        // Frame ID
        mnId = nNextId++;

        // Scale Level Info
        mnScaleLevels = mpORBextractorLeft->GetLevels();
        mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
        mfLogScaleFactor = log(mfScaleFactor);
        mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
        mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
        mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
        mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

        // ORB extraction
#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_StartExtORB = std::chrono::steady_clock::now();
#endif
        ExtractORB(0, imGray, 0, 0);
#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_EndExtORB = std::chrono::steady_clock::now();

        mTimeORB_Ext = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(time_EndExtORB - time_StartExtORB).count();
#endif

        N = mvKeys.size();

        if (mvKeys.empty())
            return;

        UndistortKeyPoints();

        ComputeStereoFromRGBD(imDepth);

        mvpMapPoints = vector<MapPoint *>(N, static_cast<MapPoint *>(NULL));

        mmProjectPoints.clear();
        mmMatchedInImage.clear();

        mvbOutlier = vector<bool>(N, false);

        // This is done only for the first Frame (or after a change in the calibration)
        if (mbInitialComputations)
        {
            ComputeImageBounds(imGray);

            mfGridElementWidthInv = static_cast<float>(FRAME_GRID_COLS) / static_cast<float>(mnMaxX - mnMinX);
            mfGridElementHeightInv = static_cast<float>(FRAME_GRID_ROWS) / static_cast<float>(mnMaxY - mnMinY);

            fx = K.at<float>(0, 0);
            fy = K.at<float>(1, 1);
            cx = K.at<float>(0, 2);
            cy = K.at<float>(1, 2);
            invfx = 1.0f / fx;
            invfy = 1.0f / fy;

            mbInitialComputations = false;
        }

        mb = mbf / fx;

        if (pPrevF)
        {
            if (pPrevF->HasVelocity())
                SetVelocity(pPrevF->GetVelocity());
        }
        else
        {
            mVw.setZero();
        }

        mpMutexImu = new std::mutex();

        // Set no stereo fisheye information
        Nleft = -1;
        Nright = -1;
        mvLeftToRightMatch = vector<int>(0);
        mvRightToLeftMatch = vector<int>(0);
        mvStereo3Dpoints = vector<Eigen::Vector3f>(0);
        monoLeft = -1;
        monoRight = -1;

        AssignFeaturesToGrid();
    }

    Frame::Frame(const cv::Mat &imGray, const double &timeStamp, ORBextractor *extractor, ORBVocabulary *voc, GeometricCamera *pCamera, cv::Mat &distCoef, const float &bf, const float &thDepth, Frame *pPrevF, const IMU::Calib &ImuCalib)
        : mpcpi(nullptr), mbHasPose(false), mbHasVelocity(false), mpORBvocabulary(voc),
          mpORBextractorLeft(extractor), mpORBextractorRight(static_cast<ORBextractor *>(nullptr)), mTimeStamp(timeStamp), mK(dynamic_cast<Pinhole *>(pCamera)->toK()), mK_(dynamic_cast<Pinhole *>(pCamera)->toK_()), mDistCoef(distCoef.clone()),
          mbf(bf), mThDepth(thDepth), mImuCalib(ImuCalib), mpImuPreintegrated(nullptr), mpPrevFrame(pPrevF), mpImuPreintegratedFrame(nullptr), mpReferenceKF(static_cast<KeyFrame *>(nullptr)), mbIsSet(false),
          mbImuPreintegrated(false), mpCamera(pCamera), mpCamera2(nullptr)
    {
        // Frame ID
        mnId = nNextId++;

        // Scale Level Info
        mnScaleLevels = mpORBextractorLeft->GetLevels();
        mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
        mfLogScaleFactor = log(mfScaleFactor);
        mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
        mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
        mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
        mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

        // ORB extraction
#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_StartExtORB = std::chrono::steady_clock::now();
#endif
        ExtractORB(0, imGray, 0, 1000);
#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_EndExtORB = std::chrono::steady_clock::now();

        mTimeORB_Ext = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(time_EndExtORB - time_StartExtORB).count();
#endif

        N = mvKeys.size();
        if (mvKeys.empty())
            return;

        UndistortKeyPoints();

        // Set no stereo information
        mvuRight = vector<float>(N, -1);
        mvDepth = vector<float>(N, -1);
        mnCloseMPs = 0;

        mvpMapPoints = vector<MapPoint *>(N, static_cast<MapPoint *>(NULL));

        mmProjectPoints.clear(); // = map<long unsigned int, cv::Point2f>(N, static_cast<cv::Point2f>(NULL));
        mmMatchedInImage.clear();

        mvbOutlier = vector<bool>(N, false);

        // This is done only for the first Frame (or after a change in the calibration)
        if (mbInitialComputations)
        {
            ComputeImageBounds(imGray);

            mfGridElementWidthInv = static_cast<float>(FRAME_GRID_COLS) / static_cast<float>(mnMaxX - mnMinX);
            mfGridElementHeightInv = static_cast<float>(FRAME_GRID_ROWS) / static_cast<float>(mnMaxY - mnMinY);

            fx = dynamic_cast<Pinhole *>(mpCamera)->toK().at<float>(0, 0);
            fy = dynamic_cast<Pinhole *>(mpCamera)->toK().at<float>(1, 1);
            cx = dynamic_cast<Pinhole *>(mpCamera)->toK().at<float>(0, 2);
            cy = dynamic_cast<Pinhole *>(mpCamera)->toK().at<float>(1, 2);
            invfx = 1.0f / fx;
            invfy = 1.0f / fy;

            mbInitialComputations = false;
        }

        mb = mbf / fx;

        // Set no stereo fisheye information
        Nleft = -1;
        Nright = -1;
        mvLeftToRightMatch = vector<int>(0);
        mvRightToLeftMatch = vector<int>(0);
        mvStereo3Dpoints = vector<Eigen::Vector3f>(0);
        monoLeft = -1;
        monoRight = -1;

        AssignFeaturesToGrid();

        if (pPrevF)
        {
            if (pPrevF->HasVelocity())
            {
                SetVelocity(pPrevF->GetVelocity());
            }
        }
        else
        {
            mVw.setZero();
        }

        mpMutexImu = new std::mutex();
    }

    void Frame::processSemanticKeyPoints(const cv::Mat &imLeftSem, bool dynamic_flag) {
        if (dynamic_flag) {
            std::vector<cv::KeyPoint> KeysClear;
            cv::Mat DescriptorsClear;
            
            // Filter keypoints based on semantic information
            for (auto & mvKey : mvKeys) {
                bool flag = false;
                for (int i1 = -5; i1 < 6; i1++) {
                    for (int i2 = -5; i2 < 6; i2++) {
                        if (((int)mvKey.pt.x + i1 < 0) || ((int)mvKey.pt.x + i1 >= imLeftSem.rows)) {
                            continue;
                        }
                        if (((int)mvKey.pt.y + i2 < 0) || ((int)mvKey.pt.y + i2 >= imLeftSem.cols)) {
                            continue;
                        }
                        int cls = (int)(imLeftSem.at<cv::Vec3b>(cv::Point((int)mvKey.pt.x + i1, (int)mvKey.pt.y + i2)))[0];
                        if ((cls == 4) || (cls == 10)) {
                            flag = true;
                            break;
                        }
                    }
                    if (flag) break;
                }
                if (!flag) {
                    KeysClear.push_back(mvKey);
                }
            }

            // Update descriptors
            DescriptorsClear.create(KeysClear.size(), 32, CV_8U);
            int j = 0;
            for (int i = 0; i < mvKeys.size(); i++) {
                bool flag = false;
                for (int i1 = -5; i1 < 6; i1++) {
                    for (int i2 = -5; i2 < 6; i2++) {
                        if (((int)mvKeys[i].pt.x + i1 < 0) || ((int)mvKeys[i].pt.x + i1 >= imLeftSem.rows)) {
                            continue;
                        }
                        if (((int)mvKeys[i].pt.y + i2 < 0) || ((int)mvKeys[i].pt.y + i2 >= imLeftSem.cols)) {
                            continue;
                        }
                        int cls = (int)(imLeftSem.at<cv::Vec3b>(cv::Point((int)mvKeys[i].pt.x + i1, (int)mvKeys[i].pt.y + i2)))[0];
                        if ((cls == 4) || (cls == 10)) {
                            flag = true;
                            break;
                        }
                    }
                    if (flag) break;
                }
                if (!flag) {
                    mDescriptors.row(i).copyTo(DescriptorsClear.row(j));
                    j += 1;
                }
            }
            mvKeys = KeysClear;
            mDescriptors = DescriptorsClear;
        }
    }

    void Frame::updateSemanticInfo(const cv::KeyPoint &kp, const cv::Mat &imLeftSem) {
        bool flag = false;
        for (int i1 = -5; i1 < 6; i1++) {
            for (int i2 = -5; i2 < 6; i2++) {
                if (((int)kp.pt.x + i1 < 0) || ((int)kp.pt.x + i1 >= imLeftSem.rows)) {
                    continue;
                }
                if (((int)kp.pt.y + i2 < 0) || ((int)kp.pt.y + i2 >= imLeftSem.cols)) {
                    continue;
                }
                int cls = (int)(imLeftSem.at<cv::Vec3b>(cv::Point((int)kp.pt.x + i1, (int)kp.pt.y + i2)))[0];
                if ((cls == 4) || (cls == 10)) {
                    flag = true;
                    int instance_cls = (int)(imLeftSem.at<cv::Vec3b>(cv::Point((int)kp.pt.x + i1, (int)kp.pt.y + i2))[1]) * 1000 + 
                                    (int)(imLeftSem.at<cv::Vec3b>(cv::Point((int)kp.pt.x + i1, (int)kp.pt.y + i2)))[2];
                    mvSemanticCls[i] = cls;
                    mvInstanceCls[i] = instance_cls;
                    if ((semantic_meta.find(instance_cls) != semantic_meta.end()) && semantic_meta[instance_cls]) {
                        mvKeysMoving[i] = true;
                    }
                    break;
                }
            }
            if (flag) break;
        }
        if (!flag) {
            int instance_cls = (int)(imLeftSem.at<cv::Vec3b>(cv::Point((int)kp.pt.x, (int)kp.pt.y))[1]) * 1000 + 
                            (int)(imLeftSem.at<cv::Vec3b>(cv::Point((int)kp.pt.x, (int)kp.pt.y)))[2];
            int cls = (int)(imLeftSem.at<cv::Vec3b>(cv::Point((int)kp.pt.x, (int)kp.pt.y))[0]);
            mvSemanticCls[i] = cls;
            mvInstanceCls[i] = instance_cls;
        }
    }

    void Frame::AssignFeaturesToGrid()
    {
        // Fill matrix with points
        const int nCells = FRAME_GRID_COLS * FRAME_GRID_ROWS;

        int nReserve = 0.5f * N / (nCells);

        for (unsigned int i = 0; i < FRAME_GRID_COLS; i++)
            for (unsigned int j = 0; j < FRAME_GRID_ROWS; j++)
            {
                mGrid[i][j].reserve(nReserve);
                if (Nleft != -1)
                {
                    mGridRight[i][j].reserve(nReserve);
                }
            }

        for (int i = 0; i < N; i++)
        {
            const cv::KeyPoint &kp = (Nleft == -1) ? mvKeysUn[i]
                                     : (i < Nleft) ? mvKeys[i]
                                                   : mvKeysRight[i - Nleft];

            int nGridPosX, nGridPosY;
            if (PosInGrid(kp, nGridPosX, nGridPosY))
            {
                if (Nleft == -1 || i < Nleft)
                    mGrid[nGridPosX][nGridPosY].push_back(i);
                else
                    mGridRight[nGridPosX][nGridPosY].push_back(i - Nleft);
            }
        }
    }


    void Frame::ExtractORB(int flag, const cv::Mat &im, const int x0, const int x1)
    {
        vector<int> vLapping = {x0, x1};
        if (flag == 0)
            monoLeft = (*mpORBextractorLeft)(im, cv::Mat(), mvKeys, mDescriptors, vLapping);
        else
            monoRight = (*mpORBextractorRight)(im, cv::Mat(), mvKeysRight, mDescriptorsRight, vLapping);
    }

    void Frame::ExtractORBSem(int flag, const cv::Mat &im, const int x0, const int x1, const cv::Mat &imSem)
    {
        vector<int> vLapping = {x0, x1};
        if (flag == 0)
            monoLeft = (*mpORBextractorLeft)(im, imSem, mvKeys, mDescriptors, vLapping);
        else
            monoRight = (*mpORBextractorRight)(im, cv::Mat(), mvKeysRight, mDescriptorsRight, vLapping);
    }

    void Frame::ExtractLine(int flag, const cv::Mat &im)
    {
        if(flag==0)
            (*mpLineExtractorLeft)(im,cv::Mat(),mvKeys_Line,mDescriptors_Line);
        else
            (*mpLineExtractorRight)(im,cv::Mat(),mvKeysRight_Line,mDescriptorsRight_Line);
    }

    bool Frame::isSet() const
    {
        return mbIsSet;
    }

    void Frame::SetPose(const Sophus::SE3<float> &Tcw)
    {
        mTcw = Tcw;

        UpdatePoseMatrices();
        mbIsSet = true;
        mbHasPose = true;
    }

    void Frame::SetNewBias(const IMU::Bias &b)
    {
        mImuBias = b;
        if (mpImuPreintegrated)
            mpImuPreintegrated->SetNewBias(b);
    }

    void Frame::SetVelocity(Eigen::Vector3f Vwb)
    {
        mVw = std::move(Vwb);
        mbHasVelocity = true;
    }

    Eigen::Vector3f Frame::GetVelocity() const
    {
        return mVw;
    }

    void Frame::SetImuPoseVelocity(const Eigen::Matrix3f &Rwb, const Eigen::Vector3f &twb, const Eigen::Vector3f &Vwb)
    {
        mVw = Vwb;
        mbHasVelocity = true;

        Sophus::SE3f Twb(Rwb, twb);
        Sophus::SE3f Tbw = Twb.inverse();

        mTcw = mImuCalib.mTcb * Tbw;

        UpdatePoseMatrices();
        mbIsSet = true;
        mbHasPose = true;
    }

    void Frame::UpdatePoseMatrices()
    {
        Sophus::SE3<float> Twc = mTcw.inverse();
        mRwc = Twc.rotationMatrix();
        mOw = Twc.translation();
        mRcw = mTcw.rotationMatrix();
        mtcw = mTcw.translation();
    }

    Eigen::Matrix<float, 3, 1> Frame::GetImuPosition() const
    {
        return mRwc * mImuCalib.mTcb.translation() + mOw;
    }

    Eigen::Matrix<float, 3, 3> Frame::GetImuRotation()
    {
        return mRwc * mImuCalib.mTcb.rotationMatrix();
    }

    Sophus::SE3<float> Frame::GetImuPose()
    {
        return mTcw.inverse() * mImuCalib.mTcb;
    }

    Sophus::SE3f Frame::GetRelativePoseTrl()
    {
        return mTrl;
    }

    Sophus::SE3f Frame::GetRelativePoseTlr()
    {
        return mTlr;
    }

    Eigen::Matrix3f Frame::GetRelativePoseTlr_rotation()
    {
        return mTlr.rotationMatrix();
    }

    Eigen::Vector3f Frame::GetRelativePoseTlr_translation()
    {
        return mTlr.translation();
    }

    bool Frame::isInFrustum(MapPoint *pMP, float viewingCosLimit)
    {
        if (Nleft == -1)
        {
            pMP->mbTrackInView = false;
            pMP->mTrackProjX = -1;
            pMP->mTrackProjY = -1;

            // 3D in absolute coordinates
            Eigen::Matrix<float, 3, 1> P = pMP->GetWorldPos();

            // 3D in camera coordinates
            const Eigen::Matrix<float, 3, 1> Pc = mRcw * P + mtcw;
            const float Pc_dist = Pc.norm();

            // Check positive depth
            const float &PcZ = Pc(2);
            const float invz = 1.0f / PcZ;
            if (PcZ < 0.0f)
                return false;

            const Eigen::Vector2f uv = mpCamera->project(Pc);

            if (uv(0) < mnMinX || uv(0) > mnMaxX)
                return false;
            if (uv(1) < mnMinY || uv(1) > mnMaxY)
                return false;

            pMP->mTrackProjX = uv(0);
            pMP->mTrackProjY = uv(1);

            // Check distance is in the scale invariance region of the MapPoint
            const float maxDistance = pMP->GetMaxDistanceInvariance();
            const float minDistance = pMP->GetMinDistanceInvariance();
            const Eigen::Vector3f PO = P - mOw;
            const float dist = PO.norm();

            if (dist < minDistance || dist > maxDistance)
                return false;

            // Check viewing angle
            Eigen::Vector3f Pn = pMP->GetNormal();

            const float viewCos = PO.dot(Pn) / dist;

            if (viewCos < viewingCosLimit)
                return false;

            // Predict scale in the image
            const int nPredictedLevel = pMP->PredictScale(dist, this);

            // Data used by the tracking
            pMP->mbTrackInView = true;
            pMP->mTrackProjX = uv(0);
            pMP->mTrackProjXR = uv(0) - mbf * invz;

            pMP->mTrackDepth = Pc_dist;

            pMP->mTrackProjY = uv(1);
            pMP->mnTrackScaleLevel = nPredictedLevel;
            pMP->mTrackViewCos = viewCos;

            return true;
        }
        else
        {
            pMP->mbTrackInView = false;
            pMP->mbTrackInViewR = false;
            pMP->mnTrackScaleLevel = -1;
            pMP->mnTrackScaleLevelR = -1;

            pMP->mbTrackInView = isInFrustumChecks(pMP, viewingCosLimit);
            pMP->mbTrackInViewR = isInFrustumChecks(pMP, viewingCosLimit, true);

            return pMP->mbTrackInView || pMP->mbTrackInViewR;
        }
    }

    bool Frame::isInFrustumLine(MapLine *pML, float viewingCosLimit)
    {
        pML->mbTrackInView = false;

        // 3D in absolute coordinates
        Eigen::Matrix<float, 6, 1> sep = pML->GetWorldPos();
        Eigen::Matrix<float, 3, 1> sp_eigen = sep.head(3);
        Eigen::Matrix<float, 3, 1> ep_eigen = sep.tail(3);
        {
            const Eigen::Matrix<float, 3, 1> Pc = mRcw * sp_eigen + mtcw;
            const float Pc_dist = Pc.norm();

            // Check positive depth
            const float &PcZ = Pc(2);
            const float invz = 1.0f / PcZ;
            if (PcZ < 0.0f)
                return false;

            const Eigen::Vector2f uv = mpCamera->project(Pc);

            if (uv(0) < mnMinX || uv(0) > mnMaxX)
                return false;
            if (uv(1) < mnMinY || uv(1) > mnMaxY)
                return false;

            pML->mTrackProjsX = uv(0);
            pML->mTrackProjsY = uv(1);
        }
        {
            // 3D in camera coordinates
            const Eigen::Matrix<float, 3, 1> Pc = mRcw * ep_eigen + mtcw;
            const float Pc_dist = Pc.norm();

            // Check positive depth
            const float &PcZ = Pc(2);
            const float invz = 1.0f / PcZ;
            if (PcZ < 0.0f)
                return false;

            const Eigen::Vector2f uv = mpCamera->project(Pc);

            if (uv(0) < mnMinX || uv(0) > mnMaxX)
                return false;
            if (uv(1) < mnMinY || uv(1) > mnMaxY)
                return false;

            pML->mTrackProjeX = uv(0);
            pML->mTrackProjeY = uv(1);
        }

        Eigen::Matrix<float, 3, 1> MidPoint = (sp_eigen+ep_eigen)/2;

        // Check distance is in the scale invariance region of the MapPoint
        const float maxDistance = pML->GetMaxDistanceInvariance();
        const float minDistance = pML->GetMinDistanceInvariance();
        const Eigen::Vector3f PO = MidPoint - mOw;
        const float dist = PO.norm();

        if (dist < minDistance || dist > maxDistance)
            return false;

        // Check viewing angle
        Eigen::Vector3f Pn = pML->GetNormal();

        const float viewCos = PO.dot(Pn) / dist;

        if(viewCos < viewingCosLimit)
            return false;

        // Data used by the tracking
        pML->mbTrackInView = true;
        pML->mnTrackangle = atan2(pML->mTrackProjeY - pML->mTrackProjsY, pML->mTrackProjeX - pML->mTrackProjsX);

        return true;
    }

    bool Frame::ProjectPointDistort(MapPoint *pMP, cv::Point2f &kp, float &u, float &v)
    {

        // 3D in absolute coordinates
        Eigen::Vector3f P = pMP->GetWorldPos();

        // 3D in camera coordinates
        const Eigen::Vector3f Pc = mRcw * P + mtcw;
        const float &PcX = Pc(0);
        const float &PcY = Pc(1);
        const float &PcZ = Pc(2);

        // Check positive depth
        if (PcZ < 0.0f)
        {
            cout << "Negative depth: " << PcZ << endl;
            return false;
        }

        // Project in image and check it is not outside
        const float invz = 1.0f / PcZ;
        u = fx * PcX * invz + cx;
        v = fy * PcY * invz + cy;

        if (u < mnMinX || u > mnMaxX)
            return false;
        if (v < mnMinY || v > mnMaxY)
            return false;

        float u_distort, v_distort;

        float x = (u - cx) * invfx;
        float y = (v - cy) * invfy;
        float r2 = x * x + y * y;
        float k1 = mDistCoef.at<float>(0);
        float k2 = mDistCoef.at<float>(1);
        float p1 = mDistCoef.at<float>(2);
        float p2 = mDistCoef.at<float>(3);
        float k3 = 0;
        if (mDistCoef.total() == 5)
        {
            k3 = mDistCoef.at<float>(4);
        }

        // Radial distorsion
        float x_distort = x * (1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);
        float y_distort = y * (1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);

        // Tangential distorsion
        x_distort = x_distort + (2 * p1 * x * y + p2 * (r2 + 2 * x * x));
        y_distort = y_distort + (p1 * (r2 + 2 * y * y) + 2 * p2 * x * y);

        u_distort = x_distort * fx + cx;
        v_distort = y_distort * fy + cy;

        u = u_distort;
        v = v_distort;

        kp = cv::Point2f(u, v);

        return true;
    }

    Eigen::Vector3f Frame::inRefCoordinates(Eigen::Vector3f pCw)
    {
        return mRcw * pCw + mtcw;
    }

    vector<size_t> Frame::GetFeaturesInArea(const float &x, const float &y, const float &r, const int minLevel, const int maxLevel, const bool bRight) const
    {
        vector<size_t> vIndices;
        vIndices.reserve(N);

        float factorX = r;
        float factorY = r;

        const int nMinCellX = max(0, (int)floor((x - mnMinX - factorX) * mfGridElementWidthInv));
        if (nMinCellX >= FRAME_GRID_COLS)
        {
            return vIndices;
        }

        const int nMaxCellX = min((int)FRAME_GRID_COLS - 1, (int)ceil((x - mnMinX + factorX) * mfGridElementWidthInv));
        if (nMaxCellX < 0)
        {
            return vIndices;
        }

        const int nMinCellY = max(0, (int)floor((y - mnMinY - factorY) * mfGridElementHeightInv));
        if (nMinCellY >= FRAME_GRID_ROWS)
        {
            return vIndices;
        }

        const int nMaxCellY = min((int)FRAME_GRID_ROWS - 1, (int)ceil((y - mnMinY + factorY) * mfGridElementHeightInv));
        if (nMaxCellY < 0)
        {
            return vIndices;
        }

        const bool bCheckLevels = (minLevel > 0) || (maxLevel >= 0);

        for (int ix = nMinCellX; ix <= nMaxCellX; ix++)
        {
            for (int iy = nMinCellY; iy <= nMaxCellY; iy++)
            {
                const vector<size_t> vCell = (!bRight) ? mGrid[ix][iy] : mGridRight[ix][iy];
                if (vCell.empty())
                    continue;

                for (unsigned long j : vCell)
                {
                    const cv::KeyPoint &kpUn = (Nleft == -1) ? mvKeysUn[j]
                                               : (!bRight)   ? mvKeys[j]
                                                             : mvKeysRight[j];
                    if (bCheckLevels)
                    {
                        if (kpUn.octave < minLevel)
                            continue;
                        if (maxLevel >= 0)
                            if (kpUn.octave > maxLevel)
                                continue;
                    }

                    const float distx = kpUn.pt.x - x;
                    const float disty = kpUn.pt.y - y;

                    if (fabs(distx) < factorX && fabs(disty) < factorY)
                        vIndices.push_back(j);
                }
            }
        }

        return vIndices;
    }

    bool Frame::PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY)
    {
        posX = round((kp.pt.x - mnMinX) * mfGridElementWidthInv);
        posY = round((kp.pt.y - mnMinY) * mfGridElementHeightInv);

        // Keypoint's coordinates are undistorted, which could cause to go out of the image
        if (posX < 0 || posX >= FRAME_GRID_COLS || posY < 0 || posY >= FRAME_GRID_ROWS)
            return false;

        return true;
    }

    void Frame::ComputeBoW()
    {
        if (mBowVec.empty())
        {
            vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);
            mpORBvocabulary->transform(vCurrentDesc, mBowVec, mFeatVec, 4);
        }
    }

    void Frame::UndistortKeyPoints()
    {
        if (mDistCoef.at<float>(0) == 0.0)
        {
            mvKeysUn = mvKeys;
            return;
        }

        // Fill matrix with points
        cv::Mat mat(N, 2, CV_32F);

        for (int i = 0; i < N; i++)
        {
            mat.at<float>(i, 0) = mvKeys[i].pt.x;
            mat.at<float>(i, 1) = mvKeys[i].pt.y;
        }

        // Undistort points
        mat = mat.reshape(2);
        cv::undistortPoints(mat, mat, dynamic_cast<Pinhole *>(mpCamera)->toK(), mDistCoef, cv::Mat(), mK);
        mat = mat.reshape(1);

        // Fill undistorted keypoint vector
        mvKeysUn.resize(N);
        for (int i = 0; i < N; i++)
        {
            cv::KeyPoint kp = mvKeys[i];
            kp.pt.x = mat.at<float>(i, 0);
            kp.pt.y = mat.at<float>(i, 1);
            mvKeysUn[i] = kp;
        }
    }

    void Frame::UndistortKeyLines()
    {
        if(mDistCoef.at<float>(0)==0.0)
        {
            mvKeysUn_Line = mvKeys_Line;
            return;
        }

        N_Lines = mvKeys_Line.size();   // update N_l

        // Fill matrix with points
        cv::Mat mat_s(N_Lines, 2, CV_32F);
        cv::Mat mat_e(N_Lines, 2, CV_32F);
    
        for(int i=0; i < N_Lines; i++)
        {
            mat_s.at<float>(i,0) = mvKeys_Line[i].startPointX;
            mat_s.at<float>(i,1) = mvKeys_Line[i].startPointY;
            mat_e.at<float>(i,0) = mvKeys_Line[i].endPointX;
            mat_e.at<float>(i,1) = mvKeys_Line[i].endPointY;
        }

        // Undistort points
        mat_s = mat_s.reshape(2);
        mat_e = mat_e.reshape(2);

        cv::undistortPoints(mat_s, mat_s, dynamic_cast<Pinhole *>(mpCamera)->toK(), mDistCoef, cv::Mat(), mK);
        cv::undistortPoints(mat_e, mat_e, dynamic_cast<Pinhole *>(mpCamera)->toK(), mDistCoef, cv::Mat(), mK);

        mat_s = mat_s.reshape(1);
        mat_e = mat_e.reshape(1);

        // Fill undistorted keypoint vector
        mvKeysUn_Line.resize(N_Lines);
        for(int i = 0; i < N_Lines; i++)
        { 
            mvKeysUn_Line[i].startPointX = mat_s.at<float>(i,0);
            mvKeysUn_Line[i].startPointY = mat_s.at<float>(i,1);
            mvKeysUn_Line[i].endPointX = mat_e.at<float>(i,0);
            mvKeysUn_Line[i].endPointY = mat_e.at<float>(i,1);
        }  
    }

    void Frame::ComputeImageBounds(const cv::Mat &imLeft)
    {
        if (mDistCoef.at<float>(0) != 0.0)
        {
            cv::Mat mat(4, 2, CV_32F);
            mat.at<float>(0, 0) = 0.0;
            mat.at<float>(0, 1) = 0.0;
            mat.at<float>(1, 0) = imLeft.cols;
            mat.at<float>(1, 1) = 0.0;
            mat.at<float>(2, 0) = 0.0;
            mat.at<float>(2, 1) = imLeft.rows;
            mat.at<float>(3, 0) = imLeft.cols;
            mat.at<float>(3, 1) = imLeft.rows;

            mat = mat.reshape(2);
            cv::undistortPoints(mat, mat, dynamic_cast<Pinhole *>(mpCamera)->toK(), mDistCoef, cv::Mat(), mK);
            mat = mat.reshape(1);

            // Undistort corners
            mnMinX = min(mat.at<float>(0, 0), mat.at<float>(2, 0));
            mnMaxX = max(mat.at<float>(1, 0), mat.at<float>(3, 0));
            mnMinY = min(mat.at<float>(0, 1), mat.at<float>(1, 1));
            mnMaxY = max(mat.at<float>(2, 1), mat.at<float>(3, 1));
        }
        else
        {
            mnMinX = 0.0f;
            mnMaxX = imLeft.cols;
            mnMinY = 0.0f;
            mnMaxY = imLeft.rows;
        }
    }

    void Frame::ComputeStereoMatches()
    {
        mvuRight = vector<float>(N, -1.0f);
        mvDepth = vector<float>(N, -1.0f);

        const int thOrbDist = (ORBmatcher::TH_HIGH + ORBmatcher::TH_LOW) / 2;

        const int nRows = mpORBextractorLeft->mvImagePyramid[0].rows;

        // Assign keypoints to row table
        vector<vector<size_t>> vRowIndices(nRows, vector<size_t>());

        for (int i = 0; i < nRows; i++)
            vRowIndices[i].reserve(200);

        const int Nr = mvKeysRight.size();

        for (int iR = 0; iR < Nr; iR++)
        {
            const cv::KeyPoint &kp = mvKeysRight[iR];
            const float &kpY = kp.pt.y;
            const float r = 2.0f * mvScaleFactors[mvKeysRight[iR].octave];
            const int maxr = ceil(kpY + r);
            const int minr = floor(kpY - r);

            for (int yi = minr; yi <= maxr; yi++)
                vRowIndices[yi].push_back(iR);
        }

        // Set limits for search
        const float minZ = mb;
        const float minD = 0;
        const float maxD = mbf / minZ;

        // For each left keypoint search a match in the right image
        vector<pair<int, int>> vDistIdx;
        vDistIdx.reserve(N);

        for (int iL = 0; iL < N; iL++)
        {
            const cv::KeyPoint &kpL = mvKeys[iL];
            const int &levelL = kpL.octave;
            const float &vL = kpL.pt.y;
            const float &uL = kpL.pt.x;

            const vector<size_t> &vCandidates = vRowIndices[vL];

            if (vCandidates.empty())
                continue;

            const float minU = uL - maxD;
            const float maxU = uL - minD;

            if (maxU < 0)
                continue;

            int bestDist = ORBmatcher::TH_HIGH;
            size_t bestIdxR = 0;

            const cv::Mat &dL = mDescriptors.row(iL);

            // Compare descriptor to right keypoints
            for (unsigned long iR : vCandidates)
            {
                const cv::KeyPoint &kpR = mvKeysRight[iR];

                if (kpR.octave < levelL - 1 || kpR.octave > levelL + 1)
                    continue;

                const float &uR = kpR.pt.x;

                if (uR >= minU && uR <= maxU)
                {
                    const cv::Mat &dR = mDescriptorsRight.row(iR);
                    const int dist = ORBmatcher::DescriptorDistance(dL, dR);

                    if (dist < bestDist)
                    {
                        bestDist = dist;
                        bestIdxR = iR;
                    }
                }
            }

            // Subpixel match by correlation
            if (bestDist < thOrbDist)
            {
                // coordinates in image pyramid at keypoint scale
                const float uR0 = mvKeysRight[bestIdxR].pt.x;
                const float scaleFactor = mvInvScaleFactors[kpL.octave];
                const float scaleduL = round(kpL.pt.x * scaleFactor);
                const float scaledvL = round(kpL.pt.y * scaleFactor);
                const float scaleduR0 = round(uR0 * scaleFactor);

                // sliding window search
                const int w = 5;
                cv::Mat IL = mpORBextractorLeft->mvImagePyramid[kpL.octave].rowRange(scaledvL - w, scaledvL + w + 1).colRange(scaleduL - w, scaleduL + w + 1);

                int bestDist = INT_MAX;
                int bestincR = 0;
                const int L = 5;
                vector<float> vDists;
                vDists.resize(2 * L + 1);

                const float iniu = scaleduR0 + L - w;
                const float endu = scaleduR0 + L + w + 1;
                if (iniu < 0 || endu >= mpORBextractorRight->mvImagePyramid[kpL.octave].cols)
                    continue;

                for (int incR = -L; incR <= +L; incR++)
                {
                    cv::Mat IR = mpORBextractorRight->mvImagePyramid[kpL.octave].rowRange(scaledvL - w, scaledvL + w + 1).colRange(scaleduR0 + incR - w, scaleduR0 + incR + w + 1);

                    float dist = cv::norm(IL, IR, cv::NORM_L1);
                    if (dist < bestDist)
                    {
                        bestDist = dist;
                        bestincR = incR;
                    }

                    vDists[L + incR] = dist;
                }

                if (bestincR == -L || bestincR == L)
                    continue;

                // Sub-pixel match (Parabola fitting)
                const float dist1 = vDists[L + bestincR - 1];
                const float dist2 = vDists[L + bestincR];
                const float dist3 = vDists[L + bestincR + 1];

                const float deltaR = (dist1 - dist3) / (2.0f * (dist1 + dist3 - 2.0f * dist2));

                if (deltaR < -1 || deltaR > 1)
                    continue;

                // Re-scaled coordinate
                float bestuR = mvScaleFactors[kpL.octave] * ((float)scaleduR0 + (float)bestincR + deltaR);

                float disparity = (uL - bestuR);

                if (disparity >= minD && disparity < maxD)
                {
                    if (disparity <= 0)
                    {
                        disparity = 0.01;
                        bestuR = uL - 0.01;
                    }
                    mvDepth[iL] = mbf / disparity;
                    mvuRight[iL] = bestuR;
                    vDistIdx.emplace_back(bestDist, iL);
                }
            }
        }

        sort(vDistIdx.begin(), vDistIdx.end());
        const float median = vDistIdx[vDistIdx.size() / 2].first;
        const float thDist = 1.5f * 1.4f * median;

        for (int i = vDistIdx.size() - 1; i >= 0; i--)
        {
            if (vDistIdx[i].first < thDist)
                break;
            else
            {
                mvuRight[vDistIdx[i].second] = -1;
                mvDepth[vDistIdx[i].second] = -1;
            }
        }
    }

    void Frame::ComputeStereoMatches_Lines()
    {
        // Depth, Disparity and the 3D vector that expresses the observed inﬁnite line in the image plane
        mvDepth_Lines.resize(N_Lines,pair<float,float>(-1.0f,-1.0f));
        // mvDisparity_Lines.clear();
        // mvle_Lines.clear();
        // mvDisparity_Lines.resize(mvKeys_Line.size(),pair<float,float>(-1,-1));
        // mvle_Lines.resize(mvKeys_Line.size(),Vector3d(0,0,0));

        // Line segments stereo matching
        // --------------------------------------------------------------------------------------------------------------------
        if (mvKeys_Line.empty() || mvKeysRight_Line.empty())
            return;

        std::vector<line_2d> coords;   // line_2d type definition in LineMatcher.h
        coords.reserve(mvKeys_Line.size());
        for (const KeyLine &kl : mvKeys_Line)
            coords.push_back(std::make_pair(std::make_pair(kl.startPointX * inv_width, kl.startPointY * inv_height),
                                            std::make_pair(kl.endPointX * inv_width, kl.endPointY * inv_height))); 

        //Fill in grid & directions
        list<pair<int, int>> line_coords;
        GridStructure grid(FRAME_GRID_ROWS, FRAME_GRID_COLS);
        
        std::vector<std::pair<float, float>> directions(mvKeysRight_Line.size());
        for (unsigned int idx = 0; idx < mvKeysRight_Line.size(); ++idx) {
            const KeyLine &kl = mvKeysRight_Line[idx];

            std::pair<float, float> &v = directions[idx];
            v = std::make_pair((kl.endPointX - kl.startPointX) * inv_width, (kl.endPointY - kl.startPointY) * inv_height);
            normalize(v);

            getLineCoords(kl.startPointX * inv_width, kl.startPointY * inv_height, kl.endPointX * inv_width, kl.endPointY * inv_height, line_coords);
            for (const std::pair<int, int> &p : line_coords)
                grid.at(p.first, p.second).push_back(idx);
        }

        GridWindow w;
        int size_width = 7;
        int size_height = 2; // You can increase this value for non rectified images
        w.width = std::make_pair(size_width, 0);
        w.height = std::make_pair(size_height, size_height);

        std::vector<int> matches_12;
        LineMatcher::matchGrid(coords, mDescriptors_Line, grid, mDescriptorsRight_Line, directions, w, matches_12);

        // bucle around left matches
        cv::Mat mDescriptors_Line_aux;
        for (unsigned int i1 = 0; i1 < matches_12.size(); ++i1) {
            const int i2 = matches_12[i1];
            if (i2 < 0) continue;

            // estimate the disparity of the endpoints
            Eigen::Vector3f sp_l; sp_l << mvKeys_Line[i1].startPointX, mvKeys_Line[i1].startPointY, 1.0;
            Eigen::Vector3f ep_l; ep_l << mvKeys_Line[i1].endPointX,   mvKeys_Line[i1].endPointY,   1.0;
            Eigen::Vector3f sp_r; sp_r << mvKeysRight_Line[i2].startPointX, mvKeysRight_Line[i2].startPointY, 1.0;
            Eigen::Vector3f ep_r; ep_r << mvKeysRight_Line[i2].endPointX,   mvKeysRight_Line[i2].endPointY,   1.0;
            Eigen::Vector3f le_r; le_r << sp_r.cross(ep_r);

            float overlap = lineSegmentOverlapStereo( sp_l(1), ep_l(1), sp_r(1), ep_r(1) );

            float disp_s, disp_e;
            sp_r << ( sp_r(0)*( sp_l(1) - ep_r(1) ) + ep_r(0)*( sp_r(1) - sp_l(1) ) ) / ( sp_r(1)-ep_r(1) ) , sp_l(1) ,  1.0;
            ep_r << ( sp_r(0)*( ep_l(1) - ep_r(1) ) + ep_r(0)*( sp_r(1) - ep_l(1) ) ) / ( sp_r(1)-ep_r(1) ) , ep_l(1) ,  1.0;
            filterLineSegmentDisparity( sp_l.head(2), ep_l.head(2), sp_r.head(2), ep_r.head(2), disp_s, disp_e );

            // check minimal disparity
            int minDisp = 1;
            float lineHorizTh = 0.1;
            float stereoOverlapTh = 0.75;
            if( disp_s >= minDisp && disp_e >= minDisp
                && std::abs( sp_l(1)-ep_l(1) ) > lineHorizTh
                && std::abs( sp_r(1)-ep_r(1) ) > lineHorizTh
                && overlap > stereoOverlapTh )
            {
                // mvDisparity_l[i1] = make_pair(disp_s,disp_e);
                mvDepth_Lines[i1] = pair<float,float>(mbf/float(disp_s), mbf/float(disp_e));
            }
        }
        // for (int i=0; i < N_Lines; i++) {
        //     Vector3d sp_lun; sp_lun << mvKeysUn_Line[i].startPointX, mvKeysUn_Line[i].startPointY, 1.0;
        //     Vector3d ep_lun; ep_lun << mvKeysUn_Line[i].endPointX,   mvKeysUn_Line[i].endPointY,   1.0;
        //     Vector3d le_l; le_l << sp_lun.cross(ep_lun); le_l = le_l / std::sqrt( le_l(0)*le_l(0) + le_l(1)*le_l(1) );
        //     mvle_Lines[i] = le_l;
        // }
    }


    float Frame::lineSegmentOverlapStereo(float spl_obs, float epl_obs, float spl_proj, float epl_proj)
    {
        float overlap = 1.f;
        float lineHorizTh = 0.1;

        if( fabs( epl_obs - spl_obs ) > lineHorizTh ) // normal lines (verticals included)
        {
            float sln    = min(spl_obs,  epl_obs);
            float eln    = max(spl_obs,  epl_obs);
            float spn    = min(spl_proj, epl_proj);
            float epn    = max(spl_proj, epl_proj);

            float length = eln-spn;

            if ( (epn < sln) || (spn > eln) )
                overlap = 0.f;
            else{
                if ( (epn>eln) && (spn<sln) )
                    overlap = eln-sln;
                else
                    overlap = min(eln,epn) - max(sln,spn);
            }

            if(length>0.01f)
                overlap = overlap / length;
            else
                overlap = 0.f;

            if( overlap > 1.f )
                overlap = 1.f;

        }

        return overlap;
    }

    void Frame::filterLineSegmentDisparity( Eigen::Vector2f spl, Eigen::Vector2f epl, Eigen::Vector2f spr, Eigen::Vector2f epr, float &disp_s, float &disp_e )
    {
        disp_s = spl(0) - spr(0);
        disp_e = epl(0) - epr(0);
        // if they are too different, ignore them
        float lsMinDispRatio = 0.7;
        if(  min( disp_s, disp_e ) / max( disp_s, disp_e ) < lsMinDispRatio )
        {
            disp_s = -1.0;
            disp_e = -1.0;
        }
    }



    void Frame::ComputeStereoFromRGBD(const cv::Mat &imDepth)
    {
        mvuRight = vector<float>(N, -1);
        mvDepth = vector<float>(N, -1);

        for (int i = 0; i < N; i++)
        {
            const cv::KeyPoint &kp = mvKeys[i];
            const cv::KeyPoint &kpU = mvKeysUn[i];

            const float &v = kp.pt.y;
            const float &u = kp.pt.x;

            const float d = imDepth.at<float>(v, u);

            if (d > 0)
            {
                mvDepth[i] = d;
                mvuRight[i] = kpU.pt.x - mbf / d;
            }
        }
    }

    bool Frame::UnprojectStereo(const int &i, Eigen::Vector3f &x3D)
    {
        const float z = mvDepth[i];
        if (z > 0)
        {
            const float u = mvKeysUn[i].pt.x;
            const float v = mvKeysUn[i].pt.y;
            const float x = (u - cx) * z * invfx;
            const float y = (v - cy) * z * invfy;
            Eigen::Vector3f x3Dc(x, y, z);
            x3D = mRwc * x3Dc + mOw;
            return true;
        }
        else
            return false;
    }

    bool Frame::UnprojectStereoLines(const int &i, Eigen::Vector3f &x3D_start, Eigen::Vector3f &x3D_end)
    {
        const pair<float,float> z = mvDepth_Lines[i];
        if (z.first > 0 && z.second > 0) {
            const float us = mvKeysUn_Line[i].startPointX;
            const float vs = mvKeysUn_Line[i].startPointY;
            const float xs = (us - cx) * z.first * invfx;
            const float ys = (vs - cy) * z.first * invfy;
            x3D_start = Eigen::Vector3f(xs, ys, z.first);
            x3D_start = mRwc * x3D_start + mOw;

            const float ue = mvKeysUn_Line[i].endPointX;
            const float ve = mvKeysUn_Line[i].endPointY;
            const float xe = (ue - cx) * z.second * invfx;
            const float ye = (ve - cy) * z.second * invfy;
            x3D_end = Eigen::Vector3f(xe, ye, z.second);
            x3D_end = mRwc * x3D_end + mOw;
            return true;
        }
        else
            return false;
    }

    bool Frame::imuIsPreintegrated()
    {
        unique_lock<std::mutex> lock(*mpMutexImu);
        return mbImuPreintegrated;
    }

    void Frame::setIntegrated()
    {
        unique_lock<std::mutex> lock(*mpMutexImu);
        mbImuPreintegrated = true;
    }

    Frame::Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, ORBextractor *extractorLeft, ORBextractor *extractorRight, ORBVocabulary *voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth, GeometricCamera *pCamera, GeometricCamera *pCamera2, Sophus::SE3f &Tlr, Frame *pPrevF, const IMU::Calib &ImuCalib)
        : mpcpi(nullptr), mbHasPose(false), mbHasVelocity(false), mpORBvocabulary(voc), mpORBextractorLeft(extractorLeft), mpORBextractorRight(extractorRight), mTimeStamp(timeStamp), mK(K.clone()), mK_(Converter::toMatrix3f(K)), mDistCoef(distCoef.clone()),
          mbf(bf), mThDepth(thDepth), mImuCalib(ImuCalib), mpImuPreintegrated(nullptr), mpPrevFrame(pPrevF), mpImuPreintegratedFrame(nullptr), mpReferenceKF(static_cast<KeyFrame *>(nullptr)), mbImuPreintegrated(false),
          mpCamera(pCamera), mpCamera2(pCamera2)

    {
        imgLeft = imLeft.clone();
        imgRight = imRight.clone();

        // Frame ID
        mnId = nNextId++;

        // Scale Level Info
        mnScaleLevels = mpORBextractorLeft->GetLevels();
        mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
        mfLogScaleFactor = log(mfScaleFactor);
        mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
        mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
        mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
        mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

        // ORB extraction
#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_StartExtORB = std::chrono::steady_clock::now();
#endif
        thread threadLeft(&Frame::ExtractORB, this, 0, imLeft, dynamic_cast<KannalaBrandt8 *>(mpCamera)->mvLappingArea[0], dynamic_cast<KannalaBrandt8 *>(mpCamera)->mvLappingArea[1]);
        thread threadRight(&Frame::ExtractORB, this, 1, imRight, dynamic_cast<KannalaBrandt8 *>(mpCamera2)->mvLappingArea[0], dynamic_cast<KannalaBrandt8 *>(mpCamera2)->mvLappingArea[1]);
        threadLeft.join();
        threadRight.join();
#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_EndExtORB = std::chrono::steady_clock::now();

        mTimeORB_Ext = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(time_EndExtORB - time_StartExtORB).count();
#endif

        Nleft = mvKeys.size();
        Nright = mvKeysRight.size();
        N = Nleft + Nright;

        if (N == 0)
            return;

        // This is done only for the first Frame (or after a change in the calibration)
        if (mbInitialComputations)
        {
            ComputeImageBounds(imLeft);

            mfGridElementWidthInv = static_cast<float>(FRAME_GRID_COLS) / (mnMaxX - mnMinX);
            mfGridElementHeightInv = static_cast<float>(FRAME_GRID_ROWS) / (mnMaxY - mnMinY);

            fx = K.at<float>(0, 0);
            fy = K.at<float>(1, 1);
            cx = K.at<float>(0, 2);
            cy = K.at<float>(1, 2);
            invfx = 1.0f / fx;
            invfy = 1.0f / fy;

            mbInitialComputations = false;
        }

        mb = mbf / fx;

        // Sophus/Eigen
        mTlr = Tlr;
        mTrl = mTlr.inverse();
        mRlr = mTlr.rotationMatrix();
        mtlr = mTlr.translation();

#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_StartStereoMatches = std::chrono::steady_clock::now();
#endif
        ComputeStereoFishEyeMatches();
#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_EndStereoMatches = std::chrono::steady_clock::now();

        mTimeStereoMatch = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(time_EndStereoMatches - time_StartStereoMatches).count();
#endif

        // Put all descriptors in the same matrix
        cv::vconcat(mDescriptors, mDescriptorsRight, mDescriptors);

        mvpMapPoints = vector<MapPoint *>(N, static_cast<MapPoint *>(nullptr));
        mvbOutlier = vector<bool>(N, false);

        AssignFeaturesToGrid();

        mpMutexImu = new std::mutex();

        UndistortKeyPoints();
    }

    void Frame::ComputeStereoFishEyeMatches()
    {
        // Speed it up by matching keypoints in the lapping area
        vector<cv::KeyPoint> stereoLeft(mvKeys.begin() + monoLeft, mvKeys.end());
        vector<cv::KeyPoint> stereoRight(mvKeysRight.begin() + monoRight, mvKeysRight.end());

        cv::Mat stereoDescLeft = mDescriptors.rowRange(monoLeft, mDescriptors.rows);
        cv::Mat stereoDescRight = mDescriptorsRight.rowRange(monoRight, mDescriptorsRight.rows);

        mvLeftToRightMatch = vector<int>(Nleft, -1);
        mvRightToLeftMatch = vector<int>(Nright, -1);
        mvDepth = vector<float>(Nleft, -1.0f);
        mvuRight = vector<float>(Nleft, -1);
        mvStereo3Dpoints = vector<Eigen::Vector3f>(Nleft);
        mnCloseMPs = 0;

        // Perform a brute force between Keypoint in the left and right image
        vector<vector<cv::DMatch>> matches;

        BFmatcher.knnMatch(stereoDescLeft, stereoDescRight, matches, 2);

        int nMatches = 0;
        int descMatches = 0;

        // Check matches using Lowe's ratio
        for (auto &matche : matches)
        {
            if (matche.size() >= 2 && matche[0].distance < matche[1].distance * 0.7)
            {
                // For every good match, check parallax and reprojection error to discard spurious matches
                Eigen::Vector3f p3D;
                descMatches++;
                float sigma1 = mvLevelSigma2[mvKeys[matche[0].queryIdx + monoLeft].octave], sigma2 = mvLevelSigma2[mvKeysRight[matche[0].trainIdx + monoRight].octave];
                float depth = dynamic_cast<KannalaBrandt8 *>(mpCamera)->TriangulateMatches(mpCamera2, mvKeys[matche[0].queryIdx + monoLeft], mvKeysRight[matche[0].trainIdx + monoRight], mRlr, mtlr, sigma1, sigma2, p3D);
                if (depth > 0.0001f)
                {
                    mvLeftToRightMatch[matche[0].queryIdx + monoLeft] = matche[0].trainIdx + monoRight;
                    mvRightToLeftMatch[matche[0].trainIdx + monoRight] = matche[0].queryIdx + monoLeft;
                    mvStereo3Dpoints[matche[0].queryIdx + monoLeft] = p3D;
                    mvDepth[matche[0].queryIdx + monoLeft] = depth;
                    nMatches++;
                }
            }
        }
    }

    bool Frame::isInFrustumChecks(MapPoint *pMP, float viewingCosLimit, bool bRight)
    {
        // 3D in absolute coordinates
        Eigen::Vector3f P = pMP->GetWorldPos();

        Eigen::Matrix3f mR;
        Eigen::Vector3f mt, twc;
        if (bRight)
        {
            Eigen::Matrix3f Rrl = mTrl.rotationMatrix();
            Eigen::Vector3f trl = mTrl.translation();
            mR = Rrl * mRcw;
            mt = Rrl * mtcw + trl;
            twc = mRwc * mTlr.translation() + mOw;
        }
        else
        {
            mR = mRcw;
            mt = mtcw;
            twc = mOw;
        }

        // 3D in camera coordinates
        Eigen::Vector3f Pc = mR * P + mt;
        const float Pc_dist = Pc.norm();
        const float &PcZ = Pc(2);

        // Check positive depth
        if (PcZ < 0.0f)
            return false;

        // Project in image and check it is not outside
        Eigen::Vector2f uv;
        if (bRight)
            uv = mpCamera2->project(Pc);
        else
            uv = mpCamera->project(Pc);

        if (uv(0) < mnMinX || uv(0) > mnMaxX)
            return false;
        if (uv(1) < mnMinY || uv(1) > mnMaxY)
            return false;

        // Check distance is in the scale invariance region of the MapPoint
        const float maxDistance = pMP->GetMaxDistanceInvariance();
        const float minDistance = pMP->GetMinDistanceInvariance();
        const Eigen::Vector3f PO = P - twc;
        const float dist = PO.norm();

        if (dist < minDistance || dist > maxDistance)
            return false;

        // Check viewing angle
        Eigen::Vector3f Pn = pMP->GetNormal();

        const float viewCos = PO.dot(Pn) / dist;

        if (viewCos < viewingCosLimit)
            return false;

        // Predict scale in the image
        const int nPredictedLevel = pMP->PredictScale(dist, this);

        if (bRight)
        {
            pMP->mTrackProjXR = uv(0);
            pMP->mTrackProjYR = uv(1);
            pMP->mnTrackScaleLevelR = nPredictedLevel;
            pMP->mTrackViewCosR = viewCos;
            pMP->mTrackDepthR = Pc_dist;
        }
        else
        {
            pMP->mTrackProjX = uv(0);
            pMP->mTrackProjY = uv(1);
            pMP->mnTrackScaleLevel = nPredictedLevel;
            pMP->mTrackViewCos = viewCos;
            pMP->mTrackDepth = Pc_dist;
        }

        return true;
    }

    Eigen::Vector3f Frame::UnprojectStereoFishEye(const int &i)
    {
        return mRwc * mvStereo3Dpoints[i] + mOw;
    }

} // namespace ORB_SLAM

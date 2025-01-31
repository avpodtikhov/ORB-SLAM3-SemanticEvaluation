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

#include "MapDrawer.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include <pangolin/pangolin.h>
#include <mutex>

namespace ORB_SLAM3
{

    MapDrawer::MapDrawer(Atlas *pAtlas, const string &strSettingPath, Settings *settings) : mpAtlas(pAtlas)
    {
        if (settings)
        {
            newParameterLoader(settings);
        }
        else
        {
            cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
            bool is_correct = ParseViewerParamFile(fSettings);

            if (!is_correct)
            {
                std::cerr << "**ERROR in the config file, the format is not correct**" << std::endl;
                try
                {
                    throw -1;
                }
                catch (exception &e)
                {
                }
            }
        }
    }

    void MapDrawer::newParameterLoader(Settings *settings)
    {
        mKeyFrameSize = settings->keyFrameSize();
        mKeyFrameLineWidth = settings->keyFrameLineWidth();
        mGraphLineWidth = settings->graphLineWidth();
        mPointSize = settings->pointSize();
        mCameraSize = settings->cameraSize();
        mCameraLineWidth = settings->cameraLineWidth();
    }

    bool MapDrawer::ParseViewerParamFile(cv::FileStorage &fSettings)
    {
        bool b_miss_params = false;

        cv::FileNode node = fSettings["Viewer.KeyFrameSize"];
        if (!node.empty())
        {
            mKeyFrameSize = node.real();
        }
        else
        {
            std::cerr << "*Viewer.KeyFrameSize parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Viewer.KeyFrameLineWidth"];
        if (!node.empty())
        {
            mKeyFrameLineWidth = node.real();
        }
        else
        {
            std::cerr << "*Viewer.KeyFrameLineWidth parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Viewer.GraphLineWidth"];
        if (!node.empty())
        {
            mGraphLineWidth = node.real();
        }
        else
        {
            std::cerr << "*Viewer.GraphLineWidth parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Viewer.PointSize"];
        if (!node.empty())
        {
            mPointSize = node.real();
        }
        else
        {
            std::cerr << "*Viewer.PointSize parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Viewer.CameraSize"];
        if (!node.empty())
        {
            mCameraSize = node.real();
        }
        else
        {
            std::cerr << "*Viewer.CameraSize parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Viewer.CameraLineWidth"];
        if (!node.empty())
        {
            mCameraLineWidth = node.real();
        }
        else
        {
            std::cerr << "*Viewer.CameraLineWidth parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        return !b_miss_params;
    }

    void MapDrawer::DrawMapPoints()
    {
        cv::Scalar color0(0, 0, 0);
        cv::Scalar color1(70. / 255, 70. / 255, 70. / 255);
        cv::Scalar color2(100. / 255, 40. / 255, 40. / 255);
        cv::Scalar color3(55. / 255, 90. / 255, 80. / 255);
        cv::Scalar color4(220. / 255, 20. / 255, 60. / 255);
        cv::Scalar color5(153. / 255, 153. / 255, 153. / 255);
        cv::Scalar color6(157. / 255, 234. / 255, 50. / 255);
        cv::Scalar color7(128. / 255, 64. / 255, 128. / 255);
        cv::Scalar color8(244. / 255, 35. / 255, 232. / 255);
        cv::Scalar color9(107. / 255, 142. / 255, 35. / 255);
        cv::Scalar color10(0. / 255, 0. / 255, 142. / 255);
        cv::Scalar color11(102. / 255, 102. / 255, 156. / 255);
        cv::Scalar color12(220. / 255, 220. / 255, 0 / 255);
        cv::Scalar color13(70. / 255, 130. / 255, 180. / 255);
        cv::Scalar color14(81. / 255, 0 / 255, 81. / 255);
        cv::Scalar color15(150. / 255, 100. / 255, 100. / 255);
        cv::Scalar color16(230. / 255, 150. / 255, 140. / 255);
        cv::Scalar color17(180. / 255, 165. / 255, 180. / 255);
        cv::Scalar color18(250. / 255, 170. / 255, 30. / 255);
        cv::Scalar color19(110. / 255, 190. / 255, 160. / 255);
        cv::Scalar color20(170. / 255, 120. / 255, 50. / 255);
        cv::Scalar color21(45. / 255, 60. / 255, 150. / 255);
        cv::Scalar color22(145. / 255, 170. / 255, 100. / 255);
        std::vector<cv::Scalar> color_palette = {color0, color1, color2, color3, color4, color5, color6, color7, color8, color9, color10, color11, color12, color13, color14, color15, color16, color17, color18, color19, color20, color21, color22};

        Map *pActiveMap = mpAtlas->GetCurrentMap();
        if (!pActiveMap)
            return;

        const vector<MapPoint *> &vpMPs = pActiveMap->GetAllMapPoints();
        const vector<MapPoint *> &vpRefMPs = pActiveMap->GetReferenceMapPoints();

        set<MapPoint *> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

        if (vpMPs.empty())
            return;

        glPointSize(mPointSize);
        glBegin(GL_POINTS);

        for (auto vpMP : vpMPs)
        {
            if (vpMP->isBad() || spRefMPs.count(vpMP))
                continue;
            glColor3f(color_palette[vpMP->mvSemanticCls][0], color_palette[vpMP->mvSemanticCls][1], color_palette[vpMP->mvSemanticCls][2]);
            Eigen::Matrix<float, 3, 1> pos = vpMP->GetWorldPos();
            glVertex3f(pos(0), pos(1), pos(2));
        }
        glEnd();

        glPointSize(mPointSize);
        glBegin(GL_POINTS);
        glColor3f(1.0, 0.0, 0.0);

        for (auto spRefMP : spRefMPs)
        {
            if (spRefMP->isBad())
                continue;
            Eigen::Matrix<float, 3, 1> pos = spRefMP->GetWorldPos();
            glVertex3f(pos(0), pos(1), pos(2));
        }

        glEnd();
    }

    void MapDrawer::DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph, const bool bDrawInertialGraph, const bool bDrawOptLba)
    {
        const float &w = mKeyFrameSize;
        const float h = w * 0.75;
        const float z = w * 0.6;

        Map *pActiveMap = mpAtlas->GetCurrentMap();
        // DEBUG LBA
        std::set<long unsigned int> sOptKFs = pActiveMap->msOptKFs;
        std::set<long unsigned int> sFixedKFs = pActiveMap->msFixedKFs;

        if (!pActiveMap)
            return;

        const vector<KeyFrame *> vpKFs = pActiveMap->GetAllKeyFrames();

        if (bDrawKF)
        {
            for (auto pKF : vpKFs)
            {
                Eigen::Matrix4f Twc = pKF->GetPoseInverse().matrix();
                unsigned int index_color = pKF->mnOriginMapId;

                glPushMatrix();

                glMultMatrixf((GLfloat *)Twc.data());

                if (!pKF->GetParent()) // It is the first KF in the map
                {
                    glLineWidth(mKeyFrameLineWidth * 5);
                    glColor3f(1.0f, 0.0f, 0.0f);
                    glBegin(GL_LINES);
                }
                else
                {
                    // cout << "Child KF: " << vpKFs[i]->mnId << endl;
                    glLineWidth(mKeyFrameLineWidth);
                    if (bDrawOptLba)
                    {
                        if (sOptKFs.find(pKF->mnId) != sOptKFs.end())
                        {
                            glColor3f(0.0f, 1.0f, 0.0f); // Green -> Opt KFs
                        }
                        else if (sFixedKFs.find(pKF->mnId) != sFixedKFs.end())
                        {
                            glColor3f(1.0f, 0.0f, 0.0f); // Red -> Fixed KFs
                        }
                        else
                        {
                            glColor3f(0.0f, 0.0f, 1.0f); // Basic color
                        }
                    }
                    else
                    {
                        glColor3f(0.0f, 0.0f, 1.0f); // Basic color
                    }
                    glBegin(GL_LINES);
                }

                glVertex3f(0, 0, 0);
                glVertex3f(w, h, z);
                glVertex3f(0, 0, 0);
                glVertex3f(w, -h, z);
                glVertex3f(0, 0, 0);
                glVertex3f(-w, -h, z);
                glVertex3f(0, 0, 0);
                glVertex3f(-w, h, z);

                glVertex3f(w, h, z);
                glVertex3f(w, -h, z);

                glVertex3f(-w, h, z);
                glVertex3f(-w, -h, z);

                glVertex3f(-w, h, z);
                glVertex3f(w, h, z);

                glVertex3f(-w, -h, z);
                glVertex3f(w, -h, z);
                glEnd();

                glPopMatrix();

                glEnd();
            }
        }

        if (bDrawGraph)
        {
            glLineWidth(mGraphLineWidth);
            glColor4f(0.0f, 1.0f, 0.0f, 0.6f);
            glBegin(GL_LINES);

            // cout << "-----------------Draw graph-----------------" << endl;
            for (auto vpKF : vpKFs)
            {
                // Covisibility Graph
                const vector<KeyFrame *> vCovKFs = vpKF->GetCovisiblesByWeight(100);
                Eigen::Vector3f Ow = vpKF->GetCameraCenter();
                if (!vCovKFs.empty())
                {
                    for (auto vCovKF : vCovKFs)
                    {
                        if (vCovKF->mnId < vpKF->mnId)
                            continue;
                        Eigen::Vector3f Ow2 = vCovKF->GetCameraCenter();
                        glVertex3f(Ow(0), Ow(1), Ow(2));
                        glVertex3f(Ow2(0), Ow2(1), Ow2(2));
                    }
                }

                // Spanning tree
                KeyFrame *pParent = vpKF->GetParent();
                if (pParent)
                {
                    Eigen::Vector3f Owp = pParent->GetCameraCenter();
                    glVertex3f(Ow(0), Ow(1), Ow(2));
                    glVertex3f(Owp(0), Owp(1), Owp(2));
                }

                // Loops
                map<unsigned long, KeyFrame *> sLoopKFs = vpKF->GetLoopEdges();
                for (auto &sLoopKF : sLoopKFs)
                {
                    if (sLoopKF.first < vpKF->mnId)
                        continue;
                    Eigen::Vector3f Owl = sLoopKF.second->GetCameraCenter();
                    glVertex3f(Ow(0), Ow(1), Ow(2));
                    glVertex3f(Owl(0), Owl(1), Owl(2));
                }
            }

            glEnd();
        }

        if (bDrawInertialGraph && pActiveMap->isImuInitialized())
        {
            glLineWidth(mGraphLineWidth);
            glColor4f(1.0f, 0.0f, 0.0f, 0.6f);
            glBegin(GL_LINES);

            // Draw inertial links
            for (auto pKFi : vpKFs)
            {
                Eigen::Vector3f Ow = pKFi->GetCameraCenter();
                KeyFrame *pNext = pKFi->mNextKF;
                if (pNext)
                {
                    Eigen::Vector3f Owp = pNext->GetCameraCenter();
                    glVertex3f(Ow(0), Ow(1), Ow(2));
                    glVertex3f(Owp(0), Owp(1), Owp(2));
                }
            }

            glEnd();
        }

        vector<Map *> vpMaps = mpAtlas->GetAllMaps();

        if (bDrawKF)
        {
            for (Map *pMap : vpMaps)
            {
                if (pMap == pActiveMap)
                    continue;

                vector<KeyFrame *> vpKFs = pMap->GetAllKeyFrames();

                for (auto &vpKF : vpKFs)
                {
                    KeyFrame *pKF = vpKF;
                    Eigen::Matrix4f Twc = pKF->GetPoseInverse().matrix();
                    unsigned int index_color = pKF->mnOriginMapId;

                    glPushMatrix();

                    glMultMatrixf((GLfloat *)Twc.data());

                    if (!vpKF->GetParent()) // It is the first KF in the map
                    {
                        glLineWidth(mKeyFrameLineWidth * 5);
                        glColor3f(1.0f, 0.0f, 0.0f);
                        glBegin(GL_LINES);
                    }
                    else
                    {
                        glLineWidth(mKeyFrameLineWidth);
                        glColor3f(mfFrameColors[index_color][0], mfFrameColors[index_color][1], mfFrameColors[index_color][2]);
                        glBegin(GL_LINES);
                    }

                    glVertex3f(0, 0, 0);
                    glVertex3f(w, h, z);
                    glVertex3f(0, 0, 0);
                    glVertex3f(w, -h, z);
                    glVertex3f(0, 0, 0);
                    glVertex3f(-w, -h, z);
                    glVertex3f(0, 0, 0);
                    glVertex3f(-w, h, z);

                    glVertex3f(w, h, z);
                    glVertex3f(w, -h, z);

                    glVertex3f(-w, h, z);
                    glVertex3f(-w, -h, z);

                    glVertex3f(-w, h, z);
                    glVertex3f(w, h, z);

                    glVertex3f(-w, -h, z);
                    glVertex3f(w, -h, z);
                    glEnd();

                    glPopMatrix();
                }
            }
        }
    }

    void MapDrawer::DrawCurrentCamera(pangolin::OpenGlMatrix &Twc)
    {
        const float &w = mCameraSize;
        const float h = w * 0.75;
        const float z = w * 0.6;

        glPushMatrix();

#ifdef HAVE_GLES
        glMultMatrixf(Twc.m);
#else
        glMultMatrixd(Twc.m);
#endif

        glLineWidth(mCameraLineWidth);
        glColor3f(0.0f, 1.0f, 0.0f);
        glBegin(GL_LINES);
        glVertex3f(0, 0, 0);
        glVertex3f(w, h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(w, -h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(-w, -h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(-w, h, z);

        glVertex3f(w, h, z);
        glVertex3f(w, -h, z);

        glVertex3f(-w, h, z);
        glVertex3f(-w, -h, z);

        glVertex3f(-w, h, z);
        glVertex3f(w, h, z);

        glVertex3f(-w, -h, z);
        glVertex3f(w, -h, z);
        glEnd();

        glPopMatrix();
    }

    void MapDrawer::SetCurrentCameraPose(const Sophus::SE3f &Tcw)
    {
        unique_lock<mutex> lock(mMutexCamera);
        mCameraPose = Tcw.inverse();
    }

    void MapDrawer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M, pangolin::OpenGlMatrix &MOw)
    {
        Eigen::Matrix4f Twc;
        {
            unique_lock<mutex> lock(mMutexCamera);
            Twc = mCameraPose.matrix();
        }

        for (int i = 0; i < 4; i++)
        {
            M.m[4 * i] = Twc(0, i);
            M.m[4 * i + 1] = Twc(1, i);
            M.m[4 * i + 2] = Twc(2, i);
            M.m[4 * i + 3] = Twc(3, i);
        }

        MOw.SetIdentity();
        MOw.m[12] = Twc(0, 3);
        MOw.m[13] = Twc(1, 3);
        MOw.m[14] = Twc(2, 3);
    }
} // namespace ORB_SLAM

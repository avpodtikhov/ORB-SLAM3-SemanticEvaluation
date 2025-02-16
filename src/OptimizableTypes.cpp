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

#include "OptimizableTypes.h"

namespace ORB_SLAM3
{
    bool EdgeSE3ProjectXYZOnlyPose::read(std::istream &is)
    {
        for (int i = 0; i < 2; i++)
        {
            is >> _measurement[i];
        }
        for (int i = 0; i < 2; i++)
            for (int j = i; j < 2; j++)
            {
                is >> information()(i, j);
                if (i != j)
                    information()(j, i) = information()(i, j);
            }
        return true;
    }

    bool EdgeSE3ProjectXYZOnlyPose::write(std::ostream &os) const
    {

        for (int i = 0; i < 2; i++)
        {
            os << measurement()[i] << " ";
        }

        for (int i = 0; i < 2; i++)
            for (int j = i; j < 2; j++)
            {
                os << " " << information()(i, j);
            }
        return os.good();
    }

    void EdgeSE3ProjectXYZOnlyPose::linearizeOplus()
    {
        g2o::VertexSE3Expmap *vi = static_cast<g2o::VertexSE3Expmap *>(_vertices[0]);
        Eigen::Vector3d xyz_trans = vi->estimate().map(Xw);

        double x = xyz_trans[0];
        double y = xyz_trans[1];
        double z = xyz_trans[2];

        Eigen::Matrix<double, 3, 6> SE3deriv;
        SE3deriv << 0.f, z, -y, 1.f, 0.f, 0.f,
            -z, 0.f, x, 0.f, 1.f, 0.f,
            y, -x, 0.f, 0.f, 0.f, 1.f;

        _jacobianOplusXi = -pCamera->projectJac(xyz_trans) * SE3deriv;
    }

    bool EdgeSE3ProjectXYZOnlyPoseToBody::read(std::istream &is)
    {
        for (int i = 0; i < 2; i++)
        {
            is >> _measurement[i];
        }
        for (int i = 0; i < 2; i++)
            for (int j = i; j < 2; j++)
            {
                is >> information()(i, j);
                if (i != j)
                    information()(j, i) = information()(i, j);
            }
        return true;
    }

    bool EdgeSE3ProjectXYZOnlyPoseToBody::write(std::ostream &os) const
    {

        for (int i = 0; i < 2; i++)
        {
            os << measurement()[i] << " ";
        }

        for (int i = 0; i < 2; i++)
            for (int j = i; j < 2; j++)
            {
                os << " " << information()(i, j);
            }
        return os.good();
    }

    void EdgeSE3ProjectXYZOnlyPoseToBody::linearizeOplus()
    {
        g2o::VertexSE3Expmap *vi = static_cast<g2o::VertexSE3Expmap *>(_vertices[0]);
        g2o::SE3Quat T_lw(vi->estimate());
        Eigen::Vector3d X_l = T_lw.map(Xw);
        Eigen::Vector3d X_r = mTrl.map(T_lw.map(Xw));

        double x_w = X_l[0];
        double y_w = X_l[1];
        double z_w = X_l[2];

        Eigen::Matrix<double, 3, 6> SE3deriv;
        SE3deriv << 0.f, z_w, -y_w, 1.f, 0.f, 0.f,
            -z_w, 0.f, x_w, 0.f, 1.f, 0.f,
            y_w, -x_w, 0.f, 0.f, 0.f, 1.f;

        _jacobianOplusXi = -pCamera->projectJac(X_r) * mTrl.rotation().toRotationMatrix() * SE3deriv;
    }

    EdgeSE3ProjectXYZ::EdgeSE3ProjectXYZ() : BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexSBAPointXYZ, g2o::VertexSE3Expmap>()
    {
    }

    bool EdgeSE3ProjectXYZ::read(std::istream &is)
    {
        for (int i = 0; i < 2; i++)
        {
            is >> _measurement[i];
        }
        for (int i = 0; i < 2; i++)
            for (int j = i; j < 2; j++)
            {
                is >> information()(i, j);
                if (i != j)
                    information()(j, i) = information()(i, j);
            }
        return true;
    }

    bool EdgeSE3ProjectXYZ::write(std::ostream &os) const
    {

        for (int i = 0; i < 2; i++)
        {
            os << measurement()[i] << " ";
        }

        for (int i = 0; i < 2; i++)
            for (int j = i; j < 2; j++)
            {
                os << " " << information()(i, j);
            }
        return os.good();
    }

    void EdgeSE3ProjectXYZ::linearizeOplus()
    {
        g2o::VertexSE3Expmap *vj = static_cast<g2o::VertexSE3Expmap *>(_vertices[1]);
        g2o::SE3Quat T(vj->estimate());
        g2o::VertexSBAPointXYZ *vi = static_cast<g2o::VertexSBAPointXYZ *>(_vertices[0]);
        Eigen::Vector3d xyz = vi->estimate();
        Eigen::Vector3d xyz_trans = T.map(xyz);

        double x = xyz_trans[0];
        double y = xyz_trans[1];
        double z = xyz_trans[2];

        auto projectJac = -pCamera->projectJac(xyz_trans);

        _jacobianOplusXi = projectJac * T.rotation().toRotationMatrix();

        Eigen::Matrix<double, 3, 6> SE3deriv;
        SE3deriv << 0.f, z, -y, 1.f, 0.f, 0.f,
            -z, 0.f, x, 0.f, 1.f, 0.f,
            y, -x, 0.f, 0.f, 0.f, 1.f;

        _jacobianOplusXj = projectJac * SE3deriv;
    }

    EdgeSE3ProjectXYZToBody::EdgeSE3ProjectXYZToBody() : BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexSBAPointXYZ, g2o::VertexSE3Expmap>()
    {
    }

    bool EdgeSE3ProjectXYZToBody::read(std::istream &is)
    {
        for (int i = 0; i < 2; i++)
        {
            is >> _measurement[i];
        }
        for (int i = 0; i < 2; i++)
            for (int j = i; j < 2; j++)
            {
                is >> information()(i, j);
                if (i != j)
                    information()(j, i) = information()(i, j);
            }
        return true;
    }

    bool EdgeSE3ProjectXYZToBody::write(std::ostream &os) const
    {

        for (int i = 0; i < 2; i++)
        {
            os << measurement()[i] << " ";
        }

        for (int i = 0; i < 2; i++)
            for (int j = i; j < 2; j++)
            {
                os << " " << information()(i, j);
            }
        return os.good();
    }

    void EdgeSE3ProjectXYZToBody::linearizeOplus()
    {
        g2o::VertexSE3Expmap *vj = static_cast<g2o::VertexSE3Expmap *>(_vertices[1]);
        g2o::SE3Quat T_lw(vj->estimate());
        g2o::SE3Quat T_rw = mTrl * T_lw;
        g2o::VertexSBAPointXYZ *vi = static_cast<g2o::VertexSBAPointXYZ *>(_vertices[0]);
        Eigen::Vector3d X_w = vi->estimate();
        Eigen::Vector3d X_l = T_lw.map(X_w);
        Eigen::Vector3d X_r = mTrl.map(T_lw.map(X_w));

        _jacobianOplusXi = -pCamera->projectJac(X_r) * T_rw.rotation().toRotationMatrix();

        double x = X_l[0];
        double y = X_l[1];
        double z = X_l[2];

        Eigen::Matrix<double, 3, 6> SE3deriv;
        SE3deriv << 0.f, z, -y, 1.f, 0.f, 0.f,
            -z, 0.f, x, 0.f, 1.f, 0.f,
            y, -x, 0.f, 0.f, 0.f, 1.f;

        _jacobianOplusXj = -pCamera->projectJac(X_r) * mTrl.rotation().toRotationMatrix() * SE3deriv;
    }

    VertexSim3Expmap::VertexSim3Expmap() : BaseVertex<7, g2o::Sim3>()
    {
        _marginalized = false;
        _fix_scale = false;
    }

    bool VertexSim3Expmap::read(std::istream &is)
    {
        g2o::Vector7d cam2world;
        for (int i = 0; i < 6; i++)
        {
            is >> cam2world[i];
        }
        is >> cam2world[6];

        float nextParam;
        for (size_t i = 0; i < pCamera1->size(); i++)
        {
            is >> nextParam;
            pCamera1->setParameter(nextParam, i);
        }

        for (size_t i = 0; i < pCamera2->size(); i++)
        {
            is >> nextParam;
            pCamera2->setParameter(nextParam, i);
        }

        setEstimate(g2o::Sim3(cam2world).inverse());
        return true;
    }

    bool VertexSim3Expmap::write(std::ostream &os) const
    {
        g2o::Sim3 cam2world(estimate().inverse());
        g2o::Vector7d lv = cam2world.log();
        for (int i = 0; i < 7; i++)
        {
            os << lv[i] << " ";
        }

        for (size_t i = 0; i < pCamera1->size(); i++)
        {
            os << pCamera1->getParameter(i) << " ";
        }

        for (size_t i = 0; i < pCamera2->size(); i++)
        {
            os << pCamera2->getParameter(i) << " ";
        }

        return os.good();
    }

    EdgeSim3ProjectXYZ::EdgeSim3ProjectXYZ() : g2o::BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexSBAPointXYZ, VertexSim3Expmap>()
    {
    }

    bool EdgeSim3ProjectXYZ::read(std::istream &is)
    {
        for (int i = 0; i < 2; i++)
        {
            is >> _measurement[i];
        }

        for (int i = 0; i < 2; i++)
            for (int j = i; j < 2; j++)
            {
                is >> information()(i, j);
                if (i != j)
                    information()(j, i) = information()(i, j);
            }
        return true;
    }

    bool EdgeSim3ProjectXYZ::write(std::ostream &os) const
    {
        for (int i = 0; i < 2; i++)
        {
            os << _measurement[i] << " ";
        }

        for (int i = 0; i < 2; i++)
            for (int j = i; j < 2; j++)
            {
                os << " " << information()(i, j);
            }
        return os.good();
    }

    EdgeInverseSim3ProjectXYZ::EdgeInverseSim3ProjectXYZ() : g2o::BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexSBAPointXYZ, VertexSim3Expmap>()
    {
    }

    bool EdgeInverseSim3ProjectXYZ::read(std::istream &is)
    {
        for (int i = 0; i < 2; i++)
        {
            is >> _measurement[i];
        }

        for (int i = 0; i < 2; i++)
            for (int j = i; j < 2; j++)
            {
                is >> information()(i, j);
                if (i != j)
                    information()(j, i) = information()(i, j);
            }
        return true;
    }

    bool EdgeInverseSim3ProjectXYZ::write(std::ostream &os) const
    {
        for (int i = 0; i < 2; i++)
        {
            os << _measurement[i] << " ";
        }

        for (int i = 0; i < 2; i++)
            for (int j = i; j < 2; j++)
            {
                os << " " << information()(i, j);
            }
        return os.good();
    }


    bool EdgeLineSE3ProjectXYZOnlyPose::read(std::istream& is){
        for (int i=0; i<3; i++){
            is >> _measurement[i];
        }
        for (int i=0; i<2; i++)
            for (int j=i; j<2; j++) {
                is >> information()(i,j);
                if (i!=j)
                    information()(j,i)=information()(i,j);
            }
        return true;
    }

    bool EdgeLineSE3ProjectXYZOnlyPose::write(std::ostream& os) const {
        for (int i=0; i<3; i++){
            os << measurement()[i] << " ";
        }

        for (int i=0; i<2; i++)
            for (int j=i; j<2; j++){
                os << " " <<  information()(i,j);
            }
        return os.good();
    }

    Eigen::Vector2d EdgeLineSE3ProjectXYZOnlyPose::cam_project(const Eigen::Vector3d & trans_xyz) const{
    const float invz = 1.0f/trans_xyz[2];
    Eigen::Vector2d res;
    res[0] = trans_xyz[0]*invz*fx + cx;
    res[1] = trans_xyz[1]*invz*fy + cy;
    return res;
    }


    void EdgeLineSE3ProjectXYZOnlyPose::linearizeOplus() {
        g2o::VertexSE3Expmap * vi = static_cast<g2o::VertexSE3Expmap *>(_vertices[0]);
        Eigen::Vector3d xyz_trans_s = vi->estimate().map(Xw_s);
        Eigen::Vector3d xyz_trans_e = vi->estimate().map(Xw_e);

        double x_s = xyz_trans_s[0];
        double y_s = xyz_trans_s[1];
        double invz_s = 1.0/xyz_trans_s[2];
        double invz_s_2 = invz_s*invz_s;

        double x_e = xyz_trans_e[0];
        double y_e = xyz_trans_e[1];
        double invz_e = 1.0/xyz_trans_e[2];
        double invz_e_2 = invz_e*invz_e;

        double l0 = obs_temp(0);
        double l1 = obs_temp(1);

        _jacobianOplusXi(0,0) = -fx*x_s*y_s*invz_s_2*l0-fy*(1+y_s*y_s*invz_s_2)*l1;
        _jacobianOplusXi(0,1) = fx*(1+x_s*x_s*invz_s_2)*l0+fy*x_s*y_s*invz_s_2*l1;
        _jacobianOplusXi(0,2) = -fx*y_s*invz_s*l0+fy*x_s*invz_s*l1; 
        _jacobianOplusXi(0,3) = fx*invz_s*l0;
        _jacobianOplusXi(0,4) = fy*invz_s*l1;
        _jacobianOplusXi(0,5) = (-fx*x_s*l0-fy*y_s*l1)*invz_s_2;

        _jacobianOplusXi(1,0) = -fx*x_e*y_e*invz_e_2*l0-fy*(1+y_e*y_e*invz_e_2)*l1;
        _jacobianOplusXi(1,1) = fx*(1+x_e*x_e*invz_e_2)*l0+fy*x_e*y_e*invz_e_2*l1;
        _jacobianOplusXi(1,2) = -fx*y_e*invz_e*l0+fy*x_e*invz_e*l1;  
        _jacobianOplusXi(1,3) = fx*invz_e*l0;
        _jacobianOplusXi(1,4) = fy*invz_e*l1;
        _jacobianOplusXi(1,5) = (-fx*x_e*l0-fy*y_e*l1)*invz_e_2;
    }


    EdgeLineSE3ProjectXYZ::EdgeLineSE3ProjectXYZ() : 
    g2o::BaseBinaryEdge<2, Eigen::Vector3d, g2o::VertexSBALineXYZ, g2o::VertexSE3Expmap>() 
    {
    } 

    bool EdgeLineSE3ProjectXYZ::read(std::istream& is){
        for (int i=0; i<3; i++){
            is >> _measurement[i];
        }
        for (int i=0; i<2; i++)
            for (int j=i; j<2; j++) {
                is >> information()(i,j);
                if (i!=j)
                    information()(j,i)=information()(i,j);
            }
        return true;
    }

    bool EdgeLineSE3ProjectXYZ::write(std::ostream& os) const {
        for (int i=0; i<3; i++){
            os << measurement()[i] << " ";
        }

        for (int i=0; i<2; i++)
            for (int j=i; j<2; j++){
                os << " " <<  information()(i,j);
            }
        return os.good();
    }

    Eigen::Vector2d EdgeLineSE3ProjectXYZ::cam_project(const Eigen::Vector3d & trans_xyz) const{
    const float invz = 1.0f/trans_xyz[2];
    Eigen::Vector2d res;
    res[0] = trans_xyz[0]*invz*fx + cx;
    res[1] = trans_xyz[1]*invz*fy + cy;
    return res;
    }

    void EdgeLineSE3ProjectXYZ::linearizeOplus() {
        g2o::VertexSE3Expmap * vj= static_cast<g2o::VertexSE3Expmap *>(_vertices[1]);
        g2o::SE3Quat T(vj->estimate());
        g2o::VertexSBALineXYZ* vi = static_cast<g2o::VertexSBALineXYZ*>(_vertices[0]);
        Eigen::Vector3d xyz_s = vi->estimate().head(3);
        Eigen::Vector3d xyz_trans_s = T.map(xyz_s);
        Eigen::Vector3d xyz_e = vi->estimate().tail(3);
        Eigen::Vector3d xyz_trans_e = T.map(xyz_e); 

        const Eigen::Matrix3d R =  T.rotation().toRotationMatrix();

        double x_s = xyz_trans_s[0];
        double y_s = xyz_trans_s[1];
        double invz_s = 1.0/xyz_trans_s[2];
        double invz_s_2 = invz_s*invz_s;

        double x_e = xyz_trans_e[0];
        double y_e = xyz_trans_e[1];
        double invz_e = 1.0/xyz_trans_e[2];
        double invz_e_2 = invz_e*invz_e;

        double l0 = obs_temp(0);
        double l1 = obs_temp(1);

        _jacobianOplusXi(0,0) = fx*l0*invz_s*R(0,0)+fy*l1*invz_s*R(1,0)-(fx*x_s*l0*invz_s_2+fy*y_s*l1*invz_s_2)*R(2,0);
        _jacobianOplusXi(0,1) = fx*l0*invz_s*R(0,1)+fy*l1*invz_s*R(1,1)-(fx*x_s*l0*invz_s_2+fy*y_s*l1*invz_s_2)*R(2,1);
        _jacobianOplusXi(0,2) = fx*l0*invz_s*R(0,2)+fy*l1*invz_s*R(1,2)-(fx*x_s*l0*invz_s_2+fy*y_s*l1*invz_s_2)*R(2,2);

        _jacobianOplusXi(1,0) = fx*l0*invz_e*R(0,0)+fy*l1*invz_e*R(1,0)-(fx*x_e*l0*invz_e_2+fy*y_e*l1*invz_e_2)*R(2,0);
        _jacobianOplusXi(1,1) = fx*l0*invz_e*R(0,1)+fy*l1*invz_e*R(1,1)-(fx*x_e*l0*invz_e_2+fy*y_e*l1*invz_e_2)*R(2,1);
        _jacobianOplusXi(1,2) = fx*l0*invz_e*R(0,2)+fy*l1*invz_e*R(1,2)-(fx*x_e*l0*invz_e_2+fy*y_e*l1*invz_e_2)*R(2,2);

        _jacobianOplusXj(0,0) = -fx*x_s*y_s*invz_s_2*l0-fy*(1+y_s*y_s*invz_s_2)*l1;
        _jacobianOplusXj(0,1) = fx*(1+x_s*x_s*invz_s_2)*l0+fy*x_s*y_s*invz_s_2*l1;
        _jacobianOplusXj(0,2) = -fx*y_s*invz_s*l0+fy*x_s*invz_s*l1; 
        _jacobianOplusXj(0,3) = fx*invz_s*l0;
        _jacobianOplusXj(0,4) = fy*invz_s*l1;
        _jacobianOplusXj(0,5) = (-fx*x_s*l0-fy*y_s*l1)*invz_s_2;

        _jacobianOplusXj(1,0) = -fx*x_e*y_e*invz_e_2*l0-fy*(1+y_e*y_e*invz_e_2)*l1;
        _jacobianOplusXj(1,1) = fx*(1+x_e*x_e*invz_e_2)*l0+fy*x_e*y_e*invz_e_2*l1;
        _jacobianOplusXj(1,2) = -fx*y_e*invz_e*l0+fy*x_e*invz_e*l1;  
        _jacobianOplusXj(1,3) = fx*invz_e*l0;
        _jacobianOplusXj(1,4) = fy*invz_e*l1;
        _jacobianOplusXj(1,5) = (-fx*x_e*l0-fy*y_e*l1)*invz_e_2; 
    }
}

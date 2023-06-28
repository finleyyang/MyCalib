/******************************************************************************
*  FILE_NAME  : Optimizer.h
*  AUTHER     : finley
*  DATA       : 23-6-28
*  BRIEF      :
*  Email      : finleyyang@163.com
******************************************************************************/

#ifndef MYCALIB_OPTIMIZER_H
#define MYCALIB_OPTIMIZER_H
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Eigen>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#pragma once

class ReprojectionError
{
    ReprojectionError(const Eigen::Vector2d & img_pts_, const Eigen::Vector2d & board_pts_)
    : img_pts(img_pts_), board_pts(board_pts_) {}
    // img_pts : 图像坐标
    // board_pts : 标定板齐次坐标
    template <typename T>

    bool operator() (const T * const intrinsics_, const T * const k_, const T * const rt_, T * residuals)
    {
        //Eigen::Vector3d hom_w(board_pts(0), board_pts(1), T(1.))
        T hom_w_t[3];
        hom_w_t[0] = T(board_pts(0));
        hom_w_t[1] = T(board_pts(1));
        hom_w_t[2] = T(1.);
        T hom_w_trans[3];
        ceres::AngleAxisRotatePoint(rt_, hom_w_t, hom_w_trans);
        //世界坐标系下的标定板坐标转相机坐标系下的标定板坐标
        hom_w_t[0] += rt_[3];
        hom_w_t[1] += rt_[4];
        hom_w_t[2] += rt_[5];

        T c_x = hom_w_trans[0] / hom_w_trans[2];
        T c_y = hom_w_trans[1] / hom_w_trans[2];

        // distortion
        T r2 = c_x * c_x + c_y * c_y;
        T r4 = r2 * r2;
        T r6 = r2 * r2 * r2;
        T a1 = T(2) * c_x * c_y;
        T a2 = r2 * T(2) * c_x * c_x;
        T a3 = r2 * T(2) * c_y * c_y;
        T coeff = (T(1) + k_[0] * r2 +k_[1] * r4 + k_[4] * r6);
        T xd = c_x * coeff + k_[2] * a1 + k_[3] * a2;
        T yd = c_y * coeff + k_[2] * a3 + k_[3] * a1;

        // camera coord => image coord
        T predict_x = intrinsics_[0] * xd + intrinsics_[1] * yd + intrinsics_[2];
        T predict_y = intrinsics_[3] * yd + intrinsics_[4];

        //residuals
        residuals[0] = img_pts(0) - predict_x;
        residuals[1] = img_pts(1) - predict_y;

        return true;
    }

    Eigen::Vector2d img_pts;
    Eigen::Vector2d board_pts;
};

#endif //MYCALIB_OPTIMIZER_H

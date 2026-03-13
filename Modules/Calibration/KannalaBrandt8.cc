/**
* This file is part of Mini-SLAM
*
* Copyright (C) 2021 Juan J. Gómez Rodríguez and Juan D. Tardós, University of Zaragoza.
*
* Mini-SLAM is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Mini-SLAM is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with Mini-SLAM.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include "KannalaBrandt8.h"

#define fx vParameters_[0]
#define fy vParameters_[1]
#define cx vParameters_[2]
#define cy vParameters_[3]
#define k0 vParameters_[4]
#define k1 vParameters_[5]
#define k2 vParameters_[6]
#define k3 vParameters_[7]

void KannalaBrandt8::project(const Eigen::Vector3f& p3D, Eigen::Vector2f& p2D){
    /*
     * Your code for Lab 3 - Task 5 here!
     */
    float r = sqrt(p3D[0] * p3D[0] + p3D[1] * p3D[1]);
    float theta = atan(r / p3D[2]);

    float d = theta + k0 * pow(theta,3) + k1 * pow(theta,5) + k2 * pow(theta,7) + k3 * pow(theta,9);
    p2D[0] = fx * d * (p3D[0] / r) + cx;
    p2D[1] = fy * d * (p3D[1] / r) + cy;
}

void KannalaBrandt8::unproject(const Eigen::Vector2f& p2D, Eigen::Vector3f& p3D) {
    /*
     * Your code for Lab 3 - Task 5 here!
     */
    float mx = (p2D[0] - cx) / fx;
    float my = (p2D[1] - cy) / fy;
    float r_prime = sqrt(mx*mx + my*my);

    float theta = r_prime;

    for(int i = 0; i < 5; i++)
    {
        float t2 = theta*theta;
        float t3 = t2*theta;
        float t5 = t3*t2;
        float t7 = t5*t2;
        float t9 = t7*t2;

        float f = theta + k0*t3 + k1*t5 + k2*t7 + k3*t9 - r_prime;

        float df = 1
                 + 3*k0*t2
                 + 5*k1*t2*t2
                 + 7*k2*t2*t2*t2
                 + 9*k3*t2*t2*t2*t2;

        theta = theta - f/df;
    }
    
    float sin_theta = sin(theta);
    float cos_theta = cos(theta);

    p3D[0] = sin_theta * (mx / r_prime);
    p3D[1] = sin_theta * (my / r_prime);
    p3D[2] = cos_theta;
}

void KannalaBrandt8::projectJac(const Eigen::Vector3f& p3D, Eigen::Matrix<float,2,3>& Jac) {
    /*
     * Your code for Lab 3 - Task 5 here!
     */
    float r = sqrt(p3D[0] * p3D[0] + p3D[1] * p3D[1]);
    float theta = atan(r / p3D[2]);
    float d = theta + k0 * pow(theta,3) + k1 * pow(theta,5) + k2 * pow(theta,7) + k3 * pow(theta,9);


    float dr_dx = p3D[0] / r;
    float dr_dy = p3D[1] / r;

    float dtheta_dr = p3D[2] / (r * r + p3D[2] * p3D[2]);

    float dtheta_dx = dtheta_dr * dr_dx;
    float dtheta_dy = dtheta_dr * dr_dy;
    float dtheta_dz = - (p3D[2] / r * r + p3D[2]*p3D[2]);

    float dd_dx = dtheta_dx + (3 * k0 * dtheta_dx * pow(theta, 2)) + (5 * k1 * dtheta_dx * pow(theta, 4)) + (7 * k2 * dtheta_dx * pow(theta, 6)) + (9 * k3 * dtheta_dx * pow(theta, 8));

    float dd_dy = dtheta_dy + (3 * k0 * dtheta_dy * pow(theta, 2)) + (5 * k1 * dtheta_dy * pow(theta, 4)) + (7 * k2 * dtheta_dy * pow(theta, 6)) + (9 * k3 * dtheta_dy * pow(theta, 8));

    float dd_dz = dtheta_dz + (3 * k0 * dtheta_dz * pow(theta, 2)) + (5 * k1 * dtheta_dz * pow(theta, 4)) + (7 * k2 * dtheta_dz * pow(theta, 6)) + (9 * k3 * dtheta_dz * pow(theta, 8));



    float du_dx = fx * ((dd_dx * p3D[0] / r) + (d / r) - ((dr_dx / (r * r)) * d * p3D[0]));
    float du_dy = fx * p3D[0] * ((dd_dy / r) - ((dr_dy / (r * r)) * d));
    float du_dz = (fx * p3D[0] / r) * dd_dz;

    float dv_dx = fy * p3D[1] * ((dd_dx / r) - ((dr_dx / (r * r)) * d));
    float dv_dy = fy * ((dd_dy * p3D[1] / r) + (d / r) - ((dr_dy / (r * r)) * d * p3D[1]));
    float dv_dz = (fy * p3D[1] / r) * dd_dz;

    Jac(0,0) = du_dx;
    Jac(0,1) = du_dy;
    Jac(0,2) = du_dz;
    
    Jac(1,0) = dv_dx;
    Jac(1,1) = dv_dy;
    Jac(1,2) = dv_dz;


}

// Eigen::Matrix<double, 2, 3> KannalaBrandt8::projectJac(const Eigen::Vector3d &v3D) {
//     double x2 = v3D[0] * v3D[0], y2 = v3D[1] * v3D[1], z2 = v3D[2] * v3D[2];
//     double r2 = x2 + y2;
//     double r = sqrt(r2);
//     double r3 = r2 * r;
//     double theta = atan2(r, v3D[2]);

//     double theta2 = theta * theta, theta3 = theta2 * theta;
//     double theta4 = theta2 * theta2, theta5 = theta4 * theta;
//     double theta6 = theta2 * theta4, theta7 = theta6 * theta;
//     double theta8 = theta4 * theta4, theta9 = theta8 * theta;

//     double f = theta + theta3 * mvParameters[4] + theta5 * mvParameters[5] + theta7 * mvParameters[6] +
//                 theta9 * mvParameters[7];
//     double fd = 1 + 3 * mvParameters[4] * theta2 + 5 * mvParameters[5] * theta4 + 7 * mvParameters[6] * theta6 +
//                 9 * mvParameters[7] * theta8;

//     Eigen::Matrix<double, 2, 3> JacGood;
//     JacGood(0, 0) = mvParameters[0] * (fd * v3D[2] * x2 / (r2 * (r2 + z2)) + f * y2 / r3);
//     JacGood(1, 0) =
//             mvParameters[1] * (fd * v3D[2] * v3D[1] * v3D[0] / (r2 * (r2 + z2)) - f * v3D[1] * v3D[0] / r3);

//     JacGood(0, 1) =
//             mvParameters[0] * (fd * v3D[2] * v3D[1] * v3D[0] / (r2 * (r2 + z2)) - f * v3D[1] * v3D[0] / r3);
//     JacGood(1, 1) = mvParameters[1] * (fd * v3D[2] * y2 / (r2 * (r2 + z2)) + f * x2 / r3);

//     JacGood(0, 2) = -mvParameters[0] * fd * v3D[0] / (r2 + z2);
//     JacGood(1, 2) = -mvParameters[1] * fd * v3D[1] / (r2 + z2);

//     return JacGood;
// }

void KannalaBrandt8::unprojectJac(const Eigen::Vector2f& p2D, Eigen::Matrix<float,3,2>& Jac) {
    throw std::runtime_error("KannalaBrandt8::unprojectJac not implemented yet");
}
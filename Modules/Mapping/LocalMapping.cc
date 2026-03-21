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

#include "Mapping/LocalMapping.h"
#include "Optimization/g2oBundleAdjustment.h"
#include "Matching/DescriptorMatching.h"
#include "Utils/Geometry.h"

using namespace std;

LocalMapping::LocalMapping() {

}

LocalMapping::LocalMapping(Settings& settings, std::shared_ptr<Map> pMap) {
    settings_ = settings;
    pMap_ = pMap;
}

void LocalMapping::doMapping(std::shared_ptr<KeyFrame> &pCurrKeyFrame) {
    //Keep input keyframe
    currKeyFrame_ = pCurrKeyFrame;

    if(!currKeyFrame_)
        return;

    //Remove redundant MapPoints
    mapPointCulling();

    //Triangulate new MapPoints
    triangulateNewMapPoints();

    checkDuplicatedMapPoints();

    //Run a local Bundle Adjustment
    localBundleAdjustment(pMap_.get(),currKeyFrame_->getId());
}

void LocalMapping::mapPointCulling() {
    /*
     * Your code for Lab 4 - Task 4 here!
     */
}

void LocalMapping::triangulateNewMapPoints() {
    //Get a list of the best covisible KeyFrames with the current one
    vector<pair<ID,int>> vKeyFrameCovisible = pMap_->getCovisibleKeyFrames(currKeyFrame_->getId());

    vector<int> vMatches(currKeyFrame_->getMapPoints().size());

    //Get data from the current KeyFrame
    shared_ptr<CameraModel> calibration1 = currKeyFrame_->getCalibration();
    Sophus::SE3f T1w = currKeyFrame_->getPose();

    int nTriangulated = 0;

    for(pair<ID,int> pairKeyFrame_Obs : vKeyFrameCovisible){
        int commonObservations = pairKeyFrame_Obs.second;
        if(commonObservations < 20)
            continue;

        shared_ptr<KeyFrame> pKF = pMap_->getKeyFrame(pairKeyFrame_Obs.first);
        if(pKF->getId() == currKeyFrame_->getId())
            continue;

        //Check that baseline between KeyFrames is not too short
        Eigen::Vector3f vBaseLine = currKeyFrame_->getPose().inverse().translation() - pKF->getPose().inverse().translation();
        float medianDepth = pKF->computeSceneMedianDepth();
        float ratioBaseLineDepth = vBaseLine.norm() / medianDepth;

        if(ratioBaseLineDepth < 0.01){
            continue;
        }

        Sophus::SE3f T2w = pKF->getPose();

        Sophus::SE3f T21 = T2w*T1w.inverse();
        Eigen::Matrix<float,3,3> E = computeEssentialMatrixFromPose(T21);

        //Match features between the current and the covisible KeyFrame
        //TODO: this can be further improved using the orb vocabulary
        int nMatches = searchForTriangulation(currKeyFrame_.get(),pKF.get(),settings_.getMatchingForTriangulationTh(),
                settings_.getEpipolarTh(),E,vMatches);

        vector<cv::KeyPoint> vTriangulated1, vTriangulated2;
        vector<int> vMatches_;
        //Try to triangulate a new MapPoint with each match
        for(size_t i = 0; i < vMatches.size(); i++){
            if(vMatches[i] != -1){
                /*
                 * Your code for Lab 4 - Task 2 here!
                 * Note that the last KeyFrame inserted is stored at this->currKeyFrame_
                 */
                // cv::Point2f p1 = currKeyFrame_->getKeyPoint(i).pt;
                // cv::Point2f p2 = pKF->getKeyPoint(vMatches[i]).pt;

                // // 1. Obtener rayos desproyectados usando la calibración y normalizarlos 
                // Eigen::Vector3f ray_curr = currKeyFrame_->getCalibration()->unproject(p1).transpose();
                // ray_curr.normalize();

                // Eigen::Vector3f ray_covisible = pKF->getCalibration()->unproject(p2).transpose();
                // ray_covisible.normalize();
                
                // // 2. Comprobar el paralaje [cite: 68]
                // float cosParallax = cosRayParallax(ray_curr, ray_covisible);
                // if(cosParallax > settings_.getMinCos()) continue; 
                
                // // 3. Triangulación 
                // Eigen::Vector3f x3D; 
                // triangulate(ray_curr, ray_covisible, T1w, T2w, x3D);
                
                // // 4. Verificación de Profundidad Positiva (delante de ambas cámaras) [cite: 47]
                // Eigen::Vector3f x3D_curr = T1w * x3D;
                // Eigen::Vector3f x3D_covisible = T2w * x3D;

                // if(x3D_curr(2) <= 0) continue; 
                // if(x3D_covisible(2) <= 0) continue; 

                // cv::Point2f proj1 = currKeyFrame_->getCalibration()->project(x3D_curr);
                // cv::Point2f proj2 = pKF->getCalibration()->project(x3D_covisible);

                // // 5. Verificación de Error de Reproyección [cite: 47, 69]
                // float err1 = squaredReprojectionError(p1, proj1);
                // float err2 = squaredReprojectionError(p2, proj2);

                // if(err1 > settings_.getEpipolarTh() || err2 > settings_.getEpipolarTh()) continue;
                
                
                // // Si pasa todos los checks, instanciamos el MapPoint
                // std::shared_ptr<MapPoint> pNewMP = std::make_shared<MapPoint>(x3D);

                // // Vincular el MapPoint a los KeyFrames
                // currKeyFrame_->setMapPoint(i, pNewMP);
                // pKF->setMapPoint(vMatches[i], pNewMP);

                // pMap_->insertMapPoint(pNewMP);


                // pMap_->addObservation(currKeyFrame_->getId(), pNewMP->getId(), i);
                // pMap_->addObservation(pKF->getId(), pNewMP->getId(), vMatches[i]);

                // currKeyFrame_->setMapPoint(i, pNewMP);
                // pKF->setMapPoint(vMatches[i], pNewMP);

                // nTriangulated++;

                // 1. Obtener los índices de los puntos emparejados
                int idx1 = i; 
                int idx2 = vMatches[i];

                // 2. Extraer los KeyPoints 2D de cada KeyFrame
                cv::KeyPoint kp1 = currKeyFrame_->getKeyPoint(idx1);
                cv::KeyPoint kp2 = pKF->getKeyPoint(idx2);

                // 3. Desproyectar los puntos 2D a rayos 3D normalizados en la cámara
                // unproject devuelve una matriz 1x3, la transponemos a Vector3f (3x1)
                Eigen::Vector3f xn1 = currKeyFrame_->getCalibration()->unproject(kp1.pt).transpose();
                Eigen::Vector3f xn2 = pKF->getCalibration()->unproject(kp2.pt).transpose();

                // 4. Triangulación del punto 3D
                Eigen::Vector3f x3D;
                triangulate(xn1, xn2, T1w, T2w, x3D);

                // --- VALIDACIONES GEOMÉTRICAS ---

                // Transformar el punto 3D a las coordenadas locales de cada cámara
                Eigen::Vector3f x3D_c1 = T1w * x3D;
                Eigen::Vector3f x3D_c2 = T2w * x3D;

                // CHECK 1: Profundidad positiva (El punto debe estar delante de ambas cámaras)
                if (x3D_c1.z() <= 0 || x3D_c2.z() <= 0) {
                    continue; 
                }

                // CHECK 2: Paralaje suficiente
                Eigen::Vector3f O1 = T1w.inverse().translation();
                Eigen::Vector3f O2 = T2w.inverse().translation();
                Eigen::Vector3f ray1 = x3D - O1;
                Eigen::Vector3f ray2 = x3D - O2;
                
                float cosParallax = cosRayParallax(ray1, ray2);
                
                // Si el coseno es mayor a ~0.9998 (aprox 1 grado), los rayos son casi paralelos.
                if (cosParallax > 0.9998) {
                    continue; 
                }

                // CHECK 3: Error de reproyección bajo
                cv::Point2f proj1 = currKeyFrame_->getCalibration()->project(x3D_c1);
                cv::Point2f proj2 = pKF->getCalibration()->project(x3D_c2);

                float err1 = squaredReprojectionError(kp1.pt, proj1);
                float err2 = squaredReprojectionError(kp2.pt, proj2);

                // Asumimos un umbral de chi-cuadrado para 2 grados de libertad.
                // Como las coordenadas de la imagen no están estandarizadas por la incertidumbre (sigma) aquí,
                // un valor típico empírico en pixeles suele estar entre 2.0 y 5.99.
                float thError = 5.99; 
                if (err1 > thError || err2 > thError) {
                    continue;
                }

                // --- CREACIÓN E INSERCIÓN DEL MAPPOINT ---
                
                // Si pasa todos los checks, instanciamos el MapPoint
                std::shared_ptr<MapPoint> pNewMP = std::make_shared<MapPoint>(x3D);

                // Vincular el MapPoint a los KeyFrames
                currKeyFrame_->setMapPoint(idx1, pNewMP);
                pKF->setMapPoint(idx2, pNewMP);

                pMap_->insertMapPoint(pNewMP);


                pMap_->addObservation(currKeyFrame_->getId(), pNewMP->getId(), idx1);
                pMap_->addObservation(pKF->getId(), pNewMP->getId(), idx2);

                currKeyFrame_->setMapPoint(idx1, pNewMP);
                pKF->setMapPoint(idx2, pNewMP);

                nTriangulated++;

            }
        }

        cout << "Triangulated: " << nTriangulated << endl;
    }
}

void LocalMapping::checkDuplicatedMapPoints() {
    vector<pair<ID,int>> vKFcovisible = pMap_->getCovisibleKeyFrames(currKeyFrame_->getId());
    vector<shared_ptr<MapPoint>> vCurrMapPoints = currKeyFrame_->getMapPoints();

    for(int i = 0; i < vKFcovisible.size(); i++){
        if(vKFcovisible[i].first == currKeyFrame_->getId())
            continue;
        int nFused = fuse(pMap_->getKeyFrame(vKFcovisible[i].first),settings_.getMatchingFuseTh(),vCurrMapPoints,pMap_.get());
        pMap_->checkKeyFrame(vKFcovisible[i].first);
        pMap_->checkKeyFrame(currKeyFrame_->getId());
    }
}

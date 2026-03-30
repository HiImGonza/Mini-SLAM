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
    ID currKFId = currKeyFrame_->getId();
    std::vector<ID> toRemove;

    // ── Parámetros ajustables ──────────────────────────────────────────
    const long int AGE_THRESHOLD = 5;   // KFs de gracia antes de evaluar
                                         // Prueba: 3, 5, 7
    const int MIN_OBS_THRESHOLD  = 3;   // Observaciones mínimas para sobrevivir
                                         // Prueba: 2, 3
    const float MAX_REMOVAL_RATIO = 0.5f;
    // ──────────────────────────────────────────────────────────────────

    for(auto it = mRecentMapPoints_.begin(); it != mRecentMapPoints_.end(); ){
        ID mID = it->first;
        ID createdAtKF = it->second;

        if(pMap_->getMapPoint(mID) == nullptr){
            it = mRecentMapPoints_.erase(it);
            continue;
        }

        long int age = (long int)currKFId - (long int)createdAtKF;
        int nObs = pMap_->getNumberOfObservations(mID);

        if(age >= AGE_THRESHOLD){
            if(nObs < MIN_OBS_THRESHOLD){
                toRemove.push_back(mID);
            }
            it = mRecentMapPoints_.erase(it);
        } else {
            if(nObs == 0){
                toRemove.push_back(mID);
                it = mRecentMapPoints_.erase(it);
            } else {
                ++it;
            }
        }
    }

    size_t maxToRemove = max((size_t)1, 
                    (size_t)(toRemove.size() * MAX_REMOVAL_RATIO));
    toRemove.resize(std::min(toRemove.size(), maxToRemove));

    for(ID mID : toRemove){
        if(pMap_->getMapPoint(mID) == nullptr)
            continue;
        pMap_->removeMapPoint(mID);
    }
}

void LocalMapping::triangulateNewMapPoints() {
    vector<pair<ID,int>> vKeyFrameCovisible = pMap_->getCovisibleKeyFrames(currKeyFrame_->getId());
    vector<int> vMatches(currKeyFrame_->getMapPoints().size());

    shared_ptr<CameraModel> calibration1 = currKeyFrame_->getCalibration();
    Sophus::SE3f T1w = currKeyFrame_->getPose();

    int nTriangulated = 0;

    for(pair<ID,int> pairKeyFrame_Obs : vKeyFrameCovisible){
        int commonObservations = pairKeyFrame_Obs.second;
        if(commonObservations < 20)
            continue;

        shared_ptr<KeyFrame> pKF = pMap_->getKeyFrame(pairKeyFrame_Obs.first);
        if(!pKF || pKF->getId() == currKeyFrame_->getId())
            continue;

        Eigen::Vector3f vBaseLine = currKeyFrame_->getPose().inverse().translation() - 
                                    pKF->getPose().inverse().translation();
        float medianDepth = pKF->computeSceneMedianDepth();
        float ratioBaseLineDepth = vBaseLine.norm() / medianDepth;

        if(ratioBaseLineDepth < 0.01)
            continue;

        Sophus::SE3f T2w = pKF->getPose();
        Sophus::SE3f T21 = T2w * T1w.inverse();
        Eigen::Matrix<float,3,3> E = computeEssentialMatrixFromPose(T21);

        int nMatches = searchForTriangulation(currKeyFrame_.get(), pKF.get(),
                settings_.getMatchingForTriangulationTh(),
                settings_.getEpipolarTh(), E, vMatches);

        for(size_t i = 0; i < vMatches.size(); i++){
            if(vMatches[i] == -1)
                continue;

            // Guards: no sobreescribir slots ya ocupados
            if(currKeyFrame_->getMapPoints()[i] != nullptr)
                continue;
            if(pKF->getMapPoints()[vMatches[i]] != nullptr)
                continue;

            cv::Point2f p1 = currKeyFrame_->getKeyPoint(i).pt;
            cv::Point2f p2 = pKF->getKeyPoint(vMatches[i]).pt;

            Eigen::Vector3f ray_curr = currKeyFrame_->getCalibration()->unproject(p1).transpose();
            Eigen::Vector3f ray_covisible = pKF->getCalibration()->unproject(p2).transpose();
            ray_curr.normalize(); 
            ray_covisible.normalize(); 

            Eigen::Vector3f x3D;
            triangulate(ray_curr, ray_covisible, T1w, T2w, x3D);

            Eigen::Vector3f x3D_curr = T1w * x3D;
            Eigen::Vector3f x3D_covisible = T2w * x3D;

            if(x3D_curr(2) <= 0 || x3D_covisible(2) <= 0)
                continue;

            Eigen::Vector3f O1 = T1w.inverse().translation();
            Eigen::Vector3f O2 = T2w.inverse().translation();
            Eigen::Vector3f ray1 = x3D - O1;
            Eigen::Vector3f ray2 = x3D - O2;

            float cosParallax = cosRayParallax(ray1, ray2);
            if(cosParallax > settings_.getMinCos())
                continue;

            cv::Point2f proj1 = currKeyFrame_->getCalibration()->project(x3D_curr);
            cv::Point2f proj2 = pKF->getCalibration()->project(x3D_covisible);

            float err1 = squaredReprojectionError(p1, proj1);
            float err2 = squaredReprojectionError(p2, proj2);

            if(err1 > 5.99f || err2 > 5.99f)
                continue;

            auto pNewMP = std::make_shared<MapPoint>(x3D);

            pMap_->insertMapPoint(pNewMP);
            pMap_->addObservation(currKeyFrame_->getId(), pNewMP->getId(), i);
            pMap_->addObservation(pKF->getId(), pNewMP->getId(), vMatches[i]);

            currKeyFrame_->setMapPoint(i, pNewMP);
            pKF->setMapPoint(vMatches[i], pNewMP);

            // Registrar como reciente para culling
            mRecentMapPoints_[pNewMP->getId()] = currKeyFrame_->getId();

            nTriangulated++;
        }
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
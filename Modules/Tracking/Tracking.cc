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


#include "Tracking.h"

#include "Features/FAST.h"
#include "Features/ORB.h"

#include "Map/KeyFrame.h"
#include "Map/MapPoint.h"
#include "Matching/DescriptorMatching.h"

#include "Optimization/g2oBundleAdjustment.h"

using namespace std;

Tracking::Tracking(){}

Tracking::Tracking(Settings& settings, std::shared_ptr<FrameVisualizer>& visualizer,
                    std::shared_ptr<MapVisualizer>& mapVisualizer, std::shared_ptr<Map> map) {
    currFrame_ = Frame(settings.getFeaturesPerImage(),settings.getGridCols(),settings.getGridRows(),
                       settings.getImCols(),settings.getImRows(), settings.getNumberOfScales(), settings.getScaleFactor(),
                       settings.getCalibration(),settings.getDistortionParameters());
    prevFrame_ = Frame(settings.getFeaturesPerImage(),settings.getGridCols(),settings.getGridRows(),
                       settings.getImCols(),settings.getImRows(),settings.getNumberOfScales(), settings.getScaleFactor(),
                       settings.getCalibration(),settings.getDistortionParameters());

    featExtractor_ = shared_ptr<Feature>(new FAST(settings.getNumberOfScales(),settings.getScaleFactor(),settings.getFeaturesPerImage()*2,20,7));
    descExtractor_ = shared_ptr<Descriptor>(new ORB(settings.getNumberOfScales(),settings.getScaleFactor()));

    vMatches_ = vector<int>(settings.getFeaturesPerImage());

    vPrevMatched_ = vector<cv::Point2f>(settings.getFeaturesPerImage());

    status_ = NOT_INITIALIZED;
    bFirstIm_ = true;
    bMotionModel_ = false;

    monoInitializer_ = MonocularMapInitializer(settings.getFeaturesPerImage(),settings.getCalibration(),settings.getEpipolarTh(),settings.getMinCos());

    visualizer_ = visualizer;
    mapVisualizer_ = mapVisualizer;

    pMap_ = map;

    nLastKeyFrameId = 0;
    nFramesFromLastKF_ = 0;

    bInserted = false;

    settings_ = settings;
}

bool Tracking::doTracking(const cv::Mat &im, Sophus::SE3f &Tcw) {
    currIm_ = im.clone();

    //Update previous frame
    if(status_ != NOT_INITIALIZED)
        prevFrame_.assign(currFrame_);

    currFrame_.setIm(currIm_);

    //Extract features in the current image
    extractFeatures(im);

    visualizer_->drawCurrentFeatures(currFrame_.getKeyPointsDistorted(),currIm_);

    //If no map is initialized, perform monocular initialization
    if(status_ == NOT_INITIALIZED){
        if(monocularMapInitialization()){
            status_ = GOOD;
            Tcw = currFrame_.getPose();

            //Update motion model
            updateMotionModel();

            return true;
        }
        else{
            return false;
        }
    }
    //SLAM is initialized and tracking was good, track new frame
    else if(status_ == GOOD){
        //Mapping may has added/deleted MapPoints
        updateLastMapPoints();
        if(cameraTracking()){
            if(trackLocalMap()){
                //Check if we need to insert a new KeyFrame into the system
                if(needNewKeyFrame()){
                    promoteCurrentFrameToKeyFrame();
                }

                //Update motion model
                updateMotionModel();

                Tcw = currFrame_.getPose();

                visualizer_->drawCurrentFrame(currFrame_);

                return true;
            }
            else{
                status_ = LOST;
                return false;
            }
        }
        else{
            status_ = LOST;
            return false;
        }
    }
    //Camera tracking failed last frame, try to rellocalise
    else{
        //Not implemented yet
        return false;
    }
}

void Tracking::updateLastMapPoints() {
    if(bInserted){
        vector<shared_ptr<MapPoint>> vMps = pMap_->getKeyFrame(nLastKeyFrameId)->getMapPoints();
        Sophus::SE3f Tcw = pMap_->getKeyFrame(nLastKeyFrameId)->getPose();
        prevFrame_.setPose(Tcw);

        for(size_t i = 0; i < vMps.size(); i++){
            if(vMps[i]){
                prevFrame_.setMapPoint(i,vMps[i]);
            }
            else{
                prevFrame_.setMapPoint(i,nullptr);
            }
        }

        bInserted = false;
    }
}

void Tracking::extractFeatures(const cv::Mat &im) {
    //Extracf image features
    featExtractor_->extract(im,currFrame_.getKeyPointsDistorted());

    //Compute descriptors to extracted features
    descExtractor_->describe(im,currFrame_.getKeyPointsDistorted(),currFrame_.getDescriptors());

    //Distribute keys and undistort them
    currFrame_.distributeFeatures();
}

bool Tracking::monocularMapInitialization() {
    //Set first frame received as the reference frame
    if(bFirstIm_){
        monoInitializer_.changeReference(currFrame_.getKeyPoints());
        prevFrame_.assign(currFrame_);

        bFirstIm_ = false;

        visualizer_->setReferenceFrame(prevFrame_.getKeyPointsDistorted(),currIm_);

        for(size_t i = 0; i < vPrevMatched_.size(); i++){
            vPrevMatched_[i] = prevFrame_.getKeyPoint(i).pt;
        }

        return false;
    }

    //Find matches between previous and current frame
    int nMatches = searchForInitializaion(prevFrame_,currFrame_,settings_.getMatchingInitTh(),vMatches_,vPrevMatched_);

    visualizer_->drawFrameMatches(currFrame_.getKeyPointsDistorted(),currIm_,vMatches_);

    //If not enough matches found, updtate reference frame
    if(nMatches < 70){
        monoInitializer_.changeReference(currFrame_.getKeyPoints());
        prevFrame_.assign(currFrame_);

        visualizer_->setReferenceFrame(prevFrame_.getKeyPointsDistorted(),currIm_);

        for(size_t i = 0; i < vPrevMatched_.size(); i++){
            vPrevMatched_[i] = prevFrame_.getKeyPoint(i).pt;
        }

        return false;
    }

    //Try to initialize by finding an Essential matrix
    Sophus::SE3f Tcw;
    vector<Eigen::Vector3f> v3DPoints;
    v3DPoints.reserve(vMatches_.capacity());
    vector<bool> vTriangulated(vMatches_.capacity(),false);
    if(!monoInitializer_.initialize(currFrame_.getKeyPoints(), vMatches_, nMatches, Tcw, v3DPoints, vTriangulated)){
        return false;
    }



    //Get map scale
    vector<float> vDepths;
    for(int i = 0; i < vTriangulated.size(); i++){
        if(vTriangulated[i])
            vDepths.push_back(v3DPoints[i](2));
    }

    nth_element(vDepths.begin(),vDepths.begin()+vDepths.size()/2,vDepths.end());
    const float scale = vDepths[vDepths.size()/2];

    //Create map
    Tcw.translation() = Tcw.translation() / scale;

    currFrame_.setPose(Tcw);

    int nTriangulated = 0;

    for(size_t i = 0; i < vTriangulated.size(); i++){
        if(vTriangulated[i]){
            Eigen::Vector3f v = v3DPoints[i] / scale;
            shared_ptr<MapPoint> pMP(new MapPoint(v));

            prevFrame_.setMapPoint(i,pMP);
            currFrame_.setMapPoint(vMatches_[i],pMP);

            pMap_->insertMapPoint(pMP);

            nTriangulated++;
        }
    }

    cout << "Map initialized with " << nTriangulated << " MapPoints" << endl;

    shared_ptr<KeyFrame> kf0(new KeyFrame(prevFrame_));
    shared_ptr<KeyFrame> kf1(new KeyFrame(currFrame_));

    pMap_->insertKeyFrame(kf0);
    pMap_->insertKeyFrame(kf1);

    //Set observations into the map
    vector<shared_ptr<MapPoint>>& vMapPoints = kf0->getMapPoints();
    for(size_t i = 0; i < vMapPoints.size(); i++){
        auto pMP = vMapPoints[i];
        if(pMP){
            //Add observation
            pMap_->addObservation(0,pMP->getId(),i);
            pMap_->addObservation(1,pMP->getId(),vMatches_[i]);
        }
    }

    //Run a Bundle Adjustment to refine the solution
    bundleAdjustment(pMap_.get());

    Tcw = kf1->getPose();
    currFrame_.setPose(Tcw);

    updateMotionModel();

    pLastKeyFrame_ = kf1;
    nLastKeyFrameId = kf1->getId();

    mapVisualizer_->updateCurrentPose(Tcw);

    bInserted = true;

    return true;
}

bool Tracking::cameraTracking() {
    //Set pose estimation for the current frame with the motion model
    Sophus::SE3f currPose;
    if(bMotionModel_){
        currPose = motionModel_ * prevFrame_.getPose();
    }
    else{
        currPose = prevFrame_.getPose();
        bMotionModel_ = true;
    }

    currFrame_.setPose(currPose);

    //Match features between current and previous frame
    int nMatches = guidedMatching(prevFrame_,currFrame_,settings_.getMatchingGuidedTh(),vMatches_,1);

    if(nMatches < 20){
        nMatches = guidedMatching(prevFrame_,currFrame_,settings_.getMatchingGuidedTh(),vMatches_,2);
    }

    //Run a pose optimization
    nFeatTracked_ = poseOnlyOptimization(currFrame_);

    currFrame_.checkAllMapPointsAreGood();

    //Update MapDrawer
    currPose = currFrame_.getPose();
    mapVisualizer_->updateCurrentPose(currPose);

    //We enforce a minimum of 20 MapPoint matches to consider the estimation as good
    return nFeatTracked_ >= 20;
}

bool Tracking::trackLocalMap() {
    //Get local map from the last KeyFrame
    currFrame_.checkAllMapPointsAreGood();

    set<ID> sLocalMapPoints, sLocalKeyFrames, sFixedKeyFrames;
    pMap_->getLocalMapOfKeyFrame(nLastKeyFrameId,sLocalMapPoints,sLocalKeyFrames,sFixedKeyFrames);

    //Keep a record of the already tracked map points
    vector<shared_ptr<MapPoint>>& vTrackedMapPoints = currFrame_.getMapPoints();
    unordered_set<ID> sTrackedMapPoints;
    for(auto pMP : vTrackedMapPoints){
        if(pMP)
            sTrackedMapPoints.insert(pMP->getId());
    }

    //Project local map points into the current Frame
    vector<shared_ptr<MapPoint>> vMapPointsToMatch;
    vMapPointsToMatch.reserve(sLocalKeyFrames.size());
    for(auto nMapPointId : sLocalMapPoints){
        //Check if this local MapPoint is already been tracked
        if(sTrackedMapPoints.count(nMapPointId) != 0){
            continue;
        }

        vMapPointsToMatch.push_back(pMap_->getMapPoint(nMapPointId));
    }


    int nMatches = searchWithProjection(currFrame_,settings_.getMatchingByProjectionTh(),vMapPointsToMatch);


    //Run a pose optimization
    nFeatTracked_ = poseOnlyOptimization(currFrame_);

    currFrame_.checkAllMapPointsAreGood();

    //Update MapDrawer
    Sophus::SE3f currPose = currFrame_.getPose();
    mapVisualizer_->updateCurrentPose(currPose);

    //We enforce a minimum of 20 MapPoint matches to consider the estimation as good
    return nFeatTracked_ >= 20;
}

// bool Tracking::needNewKeyFrame() {
//     /*
//      * Your code for Lab 4 - Task 1 here!
//      */
//     nFramesFromLastKF_++;

//     const size_t TRACKING_POINTS = 20;
//     const size_t LAST_KF_MIN = 2; // 30 frames per second
//     const size_t LAST_KF_MAX = 30; 

//     const int newAreaTh = settings_.getFeaturesPerImage() * 0.7; 

//     bool minFramesHasPassed = (nFramesFromLastKF_ > LAST_KF_MIN);
//     bool notLost = (nFeatTracked_ > TRACKING_POINTS);
//     bool newInformation = (nFeatTracked_ < newAreaTh) || (nFramesFromLastKF_ > LAST_KF_MAX);

//     if (minFramesHasPassed && notLost && newInformation) {
//         nFramesFromLastKF_ = 0;
//         return true;
//     }
    
//     return false;
// }

bool Tracking::needNewKeyFrame() {
    /*
     * Your code for Lab 4 - Task 1 here!
     */
    nFramesFromLastKF_++;

    static int nFeaturesLastKF = settings_.getFeaturesPerImage();

    if (nFramesFromLastKF_ == 1) {
        nFeaturesLastKF = nFeatTracked_;
    }

    const size_t MIN_FRAMES = 2;
    const size_t MAX_FRAMES = 10;
    const int MIN_TRACKED_SAFE = 5;

    const float TRACK_RATIO = 0.60f;
    const int MIN_TRACKED_RATIO = nFeaturesLastKF * TRACK_RATIO;
    const int MIN_TRACKED_ABS = 150;

    cout << "--------------LAST KF: " << nFramesFromLastKF_ << endl;


    // Not enough frames has passed
    if (nFramesFromLastKF_ < MIN_FRAMES){
        return false;
    }
    
    // We are lost dont insert keyframe
    if (nFeatTracked_ < MIN_TRACKED_SAFE) {
        cout << "Tracked safe" << endl;
        return false;
    }

    // We are loosing track need KF
    if (nFeatTracked_ < MIN_TRACKED_RATIO) {
        cout << "Condition: Threshold KF" << endl;
        nFramesFromLastKF_ = 0;
        return true;
    }

    // We are loosing track need KF
    if (nFeatTracked_ < MIN_TRACKED_ABS) {
        cout << "Condition: Absolute KF" << endl;
        nFramesFromLastKF_ = 0;
        return true;
    }

    // Much time without KF
    if (nFramesFromLastKF_ > MAX_FRAMES) {
        cout << "Condition: Much time since last KF" << endl;
        nFramesFromLastKF_ = 0;
        return true;
    }

    return false;
}  

// bool Tracking::needNewKeyFrame() {
//     /*
//      * Your code for Lab 4 - Task 1 here!
//      */

//     nFramesFromLastKF_++;

//     // Guardará cuántos puntos estábamos trackeando la última vez que insertamos un KeyFrame.
//     // Usamos 'static' para que el valor sobreviva entre llamadas a la función.
//     // Inicialmente, le damos el valor máximo para el primer frame.
//     static int nFeaturesLastKF = settings_.getFeaturesPerImage();

//     if (nFramesFromLastKF_ == 1) {
//         nFeaturesLastKF = nFeatTracked_;
//     }
//     // Número mínimo y máximo de frames entre KeyFrames
//     const int minFramesBetweenKFs = 2;
//     const int maxFramesBetweenKFs = 20;

//     /// Ahora comparamos contra los puntos que teníamos en el ÚLTIMO KeyFrame, no contra el máximo.
//     const float minTrackedRatio = 0.70f;
//     const int minTrackedAbs = nFeaturesLastKF * minTrackedRatio;
//     const int minThTracks = 150;

//     // Necesitamos al menos 20 para considerarnos "perdidos" (he corregido el comentario de 10 a 20)
//     const int minFeatTracked = 20;

//     // Condición 0: Seguridad de Tracking
//     // Si estamos trackeando menos del mínimo vital, es que estamos perdidos. 
//     // No metemos NUNCA un KeyFrame, sea cual sea el tiempo que haya pasado.
//     if (nFeatTracked_ < minFeatTracked) {
//         return false;
//     }

//     // Condición 1: No insertar si han pasado muy pocos frames desde el último KF
//     // Evita insertar KFs demasiado seguidos (Añadimos <= para igualar el "> 2" de tu amigo)
//     if(nFramesFromLastKF_ <= minFramesBetweenKFs) {
//         return false;
//     }

//     // Condición 2: Forzar inserción si llevamos demasiados frames sin insertar
//     // (Llegados a este punto, ya sabemos que el tracking es bueno gracias a la Condición 0)
//     if(nFramesFromLastKF_ >= maxFramesBetweenKFs) {
//         cout << "Condition: Much time since last KF" << endl;
//         nFramesFromLastKF_ = 0; 
        
//         // actualizamos memoria
//         // nFeaturesLastKF = nFeatTracked_; 
//         return true;
//     }

//     // Condición 3: Insertar si estamos trackeando pocos puntos respecto al ultimo keyframe
//     if(nFeatTracked_ < minTrackedAbs || nFeatTracked_ < minThTracks) {
//         cout << "Condition: Absolute KF" << endl;
//         nFramesFromLastKF_ = 0; 
//         // nFeaturesLastKF = nFeatTracked_;
//         return true;
//     }

//     return false;
// }

void Tracking::promoteCurrentFrameToKeyFrame() {
    //Promote current frame to KeyFrame
    pLastKeyFrame_ = shared_ptr<KeyFrame>(new KeyFrame(currFrame_));

    //Insert KeyFrame into the map
    pMap_->insertKeyFrame(pLastKeyFrame_);

    //Add all obsevations into the map
    nLastKeyFrameId = pLastKeyFrame_->getId();
    vector<shared_ptr<MapPoint>>& vMapPoints = pLastKeyFrame_->getMapPoints();
    for(int i = 0; i < vMapPoints.size(); i++){
        MapPoint* pMP = vMapPoints[i].get();
        if(pMP)
            pMap_->addObservation(nLastKeyFrameId,pMP->getId(),i);
    }
    pMap_->checkKeyFrame(pLastKeyFrame_->getId());

    bInserted = true;
}

std::shared_ptr<KeyFrame> Tracking::getLastKeyFrame() {
    shared_ptr<KeyFrame> toReturn = pLastKeyFrame_;
    pLastKeyFrame_ = nullptr;

    return toReturn;
}

void Tracking::updateMotionModel() {
    motionModel_ = currFrame_.getPose() * prevFrame_.getPose().inverse();
}

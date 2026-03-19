/**
 * This file is part of Mini-SLAM
 * (Adaptado para la secuencia TUM-VI)
 */

#include "DatasetLoader/TUMVILoader.h" 
#include "System/MiniSLAM.h"

#include <opencv2/opencv.hpp>

using namespace std;

int main(int argc, char **argv){
    //Check program parameters are good
    if(argc != 2){
        cerr << "[Error]: you need to invoke the program with 1 parameter: " << endl;
        cerr << "\t./mono_tumvi <dataset_path>" << endl;
        cerr << "Finishing execution..." << endl;
        return -1;
    }

    //Load dataset sequence
    string datasetPath = argv[1];
    
    // 1. Instanciamos el TUMVILoader con los TRES parámetros que pide
    string timesPath = datasetPath + "/mav0/cam0/times.txt"; // Fichero con los timestamps
    string gtPath = datasetPath + "/groundtruth.txt";       // Fichero de ground truth (o nombre para guardarlo)
    
    TUMVILoader sequence(datasetPath, timesPath, gtPath);

    // 2. Creamos el sistema SLAM apuntando a nuestro NUEVO archivo de configuración
    MiniSLAM SLAM("Data/TUM-VI.yaml");

    // 3. File to store the trajectory (le cambiamos el nombre para no pisar el anterior)
    ofstream trajectoryFile ("trajectory_tumvi.txt");
    if (!trajectoryFile.is_open()){
        cerr << "[Error]: could not open the trajectory file, aborting..." << endl;
        return -2;
    }

    //Process the sequence
    cv::Mat currIm;
    double currTs;

    srand(static_cast<unsigned int>(time(NULL)));
    int totalFrames = sequence.getLenght();
    int startIdx = (totalFrames > 0) ? (rand() % totalFrames) : 0;

    // cout << "Indice de comienzo: " << startIdx << endl;
    
    // Mantenemos el "getLenght()" si así está escrito en el código de tu profesor
    for(int i = 0; i < totalFrames; i++){ 
        
        // 4. Extraemos la imagen
        sequence.getLeftImage(i, currIm);
        sequence.getTimeStamp(i, currTs);

        Sophus::SE3f Tcw;
        if(SLAM.processImage(currIm, Tcw)){
            Sophus::SE3f Twc = Tcw.inverse();
            //Save predicted pose to the file
            trajectoryFile << setprecision(17) << currTs << "," << setprecision(7) << Twc.translation()(0) << ",";
            trajectoryFile << Twc.translation()(1) << "," << Twc.translation()(2) << ",";
            trajectoryFile << Twc.unit_quaternion().x() << "," << Twc.unit_quaternion().y() << ",";
            trajectoryFile << Twc.unit_quaternion().z() << "," << Twc.unit_quaternion().w() << endl;
        }
    }

    trajectoryFile.close();

    return 0;
}
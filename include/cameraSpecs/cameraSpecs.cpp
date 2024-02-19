#include "cameraSpecs.hpp"

cameraSpecs::cameraSpecs()
{
  
}

cameraSpecs::~cameraSpecs()
{

}

void cameraSpecs::create(std::string camera_name,Sensor _Sensor,double _focalLength) {
    sensor.name = _Sensor.name;
    sensor.numberPixelsH = _Sensor.numberPixelsH;
    sensor.numberPixelsW = _Sensor.numberPixelsW;
    sensor.pixelH = _Sensor.pixelH;
    sensor.pixelW = _Sensor.pixelW;
    sensor.sensorH = _Sensor.sensorH;
    sensor.sensorW = _Sensor.sensorW;
    sensor.Cx = _Sensor.numberPixelsW/2;
    sensor.Cy = _Sensor.numberPixelsH/2;

    std::clog << "Sensor: " << _Sensor.name << std::endl;
    focalLength = _focalLength;
    name = camera_name;
    
}
void cameraSpecs::setFov(double _vFov,double _hFov,double _dFov){
    vFov = _vFov != -1 ? _vFov : vFov;
    hFov = _hFov != -1 ? _hFov : hFov;
    dFov = _dFov != -1 ? _dFov : dFov;
    
}
    
void cameraSpecs::cal_fovValues(){
    hFov = (2* atan(sensor.sensorW / (focalLength*2) ) * (180/M_PI));
    vFov = (2* atan(sensor.sensorH / (focalLength*2) ) * (180/M_PI));
    double dim = sqrt(pow(sensor.sensorW,2)+pow(sensor.sensorH,2));
    dFov = (2* atan(dim / (focalLength*2) ) * (180/M_PI));
}

double cameraSpecs::cal_focalLength(double fov){
    double focal_length=0;
    focal_length = sensor.sensorW / (tan( (fov/2) * (M_PI/180) ) * 2 );

    return focal_length;
}

double cameraSpecs::get_hFov() const{
    return hFov;
}
double cameraSpecs::get_vFov() const{
    return vFov;
}

double cameraSpecs::get_dFov() const{
    return dFov;
}


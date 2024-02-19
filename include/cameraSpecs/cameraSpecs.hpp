#ifndef CAMERA_SPECS

#define CAMERA_SPECS


#include <iostream>
#include <vector>
#include <math.h>

struct Sensor
{
    /*
    Sensor yapsı
    -> sensore ait özellikleri içinde bulundurur.
    
    */
    std::string name;
    double pixelH=0; // tek bir piksel'in mm cinsinden yüksekliğidir.
    double pixelW=0; // tek bir piksel'in mm cinsinden genişliğidir.
    double numberPixelsH=0; // dikey eksendeki satır sayısıdır.
    double numberPixelsW=0; // yatay eksende sütün sayısıdır.
    double sensorH=0; // sensorsun yüksekliği
    double sensorW=0; // sensorun genişliği
    double Cx=0; // odak uzaklığı hayalı cizgisinin temas ettiği piksel ICS'deki x konumu
    double Cy=0; // odak uzaklığı hayalı cizgisinin temas ettiği piksel ICS'deki y konumu
};

struct Lenses
{
    /* data */
};


class cameraSpecs
{
private:
    
    std::string name;
    
    double hFov;
    double vFov;
    double dFov;
    
public:
    cameraSpecs();
    ~cameraSpecs();
    void create(std::string camera_name,Sensor Sensor,double _focalLength);
    void setFov(double _vFov=-1,double _hFov=-1,double _dFov=-1);
    double get_hFov() const;
    double get_vFov() const;
    double get_dFov() const;
    void cal_fovValues();
    double cal_focalLength(double fov);
    Sensor sensor;
    double focalLength; // gercek f degeri: mm tipinden
    

};

#endif
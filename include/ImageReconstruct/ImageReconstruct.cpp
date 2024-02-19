#include "ImageReconstruct.hpp"
#define printLine() std::cout  <<"line: " << __LINE__ << std::endl;


void ImageReconstruct::warning(){
    std::cout <<
    "\n------------- Mapping: UYARI -------------\n"

    "\n---------------------------------------------\n"
    << std::endl;

}

ImageReconstruct::ImageReconstruct() {
    warning();
}

ImageReconstruct::~ImageReconstruct(){

}


// double ImageReconstruct::calRange(cv::Point2i cord_2b,double point_altute){
//     // OKS kordinatların alımını
//     int x_oks = cord_2b.x;
//     int y_oks = cord_2b.y;

//     // OKS - MKS donusumu
//     double x_mks = (double)W/2 - (double)x_oks;
//     double y_mks = (double)H/2 - (double)y_oks;

//     // MKS - DKS donusum
//     double x_degree = Fph*(double)x_mks;
//     double y_degree = Fpv*(double)y_mks;

    
//     // degrees * (PI / 180);
//     double world_Z = -1 * ((double)std::abs(global_Zc) - point_altute);
//     double u = std::tan((uav_Roll + x_degree) * (M_PI / 180))*world_Z;
//     double v = std::tan((uav_Pitch + y_degree) * (M_PI / 180))*world_Z;
    
//     // nesneye olan mesafemiz rangefinder
//     double Kp = pow(u,2) + pow(v,2);
//     Kp = sqrt(Kp);

//     double Hp = pow(world_Z,2) + pow(Kp,2);
//     Hp = sqrt(Hp);


//     // range finder !
//     return Hp;
// }

double ImageReconstruct::calRange(cv::Point2i cord_2b,double point_altute){
    // OKS kordinatların alımını
    int x_oks = cord_2b.x;
    int y_oks = cord_2b.y;

    // OKS - MKS donusumu
    double x_mks = (double)x_oks - (double)W/2;
    double y_mks = (double)y_oks - (double)H/2;

    // MKS - DKS donusum
    double x_degree = Fph*(double)x_mks;
    double y_degree = Fpv*(double)y_mks;
 
    // degrees * (PI / 180);
    double world_Z = -1 * ((double)std::abs(translatationMatrix(2)) - point_altute);
    // terrein datası kullanımı
    // ------------------------
    // her üzerinden geçtiği x,y konumun z yükseliğinde kaç metre ustunde geçtiği hesaplanacak
    // ve geçtiği yukseklikte bi yukselti varsa orada geri yansıcak world_z bu olacak

    double u = std::tan((uav_Roll + x_degree) * (M_PI / 180))*world_Z;
    double v = std::tan((uav_Pitch + y_degree) * (M_PI / 180))*world_Z;
    
    // nesneye olan mesafemiz rangefinder
    double Kp = pow(u,2) + pow(v,2);
    Kp = sqrt(Kp);

    double Hp = pow(world_Z,2) + pow(Kp,2);
    Hp = sqrt(Hp);


    // range finder !
    return Hp;
}



// matematik fonkisyonları 

void ImageReconstruct::cal_Fp(){
    Fpv = (double)Camera->get_vFov()/(double)H;
    Fph = (double)Camera->get_hFov()/(double)W;
}


double ImageReconstruct::calZ_W2C(){
    // z ekseni camera kordinat sistemine göre nerede olamalı ne olmalı bi düşün
    // şuanda z degeri kaemranın yerden yükseliği kadardır. bu da CCS'de noktanın 
    // kameradan uzaklıgı

    int flag = get_worldType() == "m" ? METER2MINIMETER:CANTIMETER2MINIMETER;
    return convertUnit(-1.0*global_Zc,flag);
}

double  ImageReconstruct::convertUnit(double value,int flag){
    double time;
    switch (flag)
    {
    case 1:
        time = 100;
        break;
    case 2:
        time = 1000;
        break;
    case 3:
        time = 0.01;
        break;
    case 4:
        time = 10;
        break;
    case 5:
        time = 0.001;
        break;
    case 6:
        time = 0.1;
        break;
    }

    return value*time;
}

void ImageReconstruct::cal_Distortion(cv::Point2i orj_pcs_point,cv::Point2i &dis_pcs_point){
    double k1 = project_distCoeffs.at<double>(0,1);
    double k2 = project_distCoeffs.at<double>(1,1);
    double p1 = project_distCoeffs.at<double>(2,1);
    double p2 = project_distCoeffs.at<double>(3,1);
    // !! k3 bozulmanın yüksek oldugundan emin olana kadar kullanma
    double k3 = 0;//project_distCoeffs.at<double>(4,1);

    double Xp = (double)orj_pcs_point.x;
    double Yp = (double)orj_pcs_point.y;
    double r_2 = pow(Xp,2) + pow(Yp,2);


    double Xdp,Ydp;

    double Xdp_rotation = Xp*(1 + k1*r_2 + k2*pow(r_2,2) + k3*pow(r_2,3));
    double Xdp_tangential = Xp+(2*p1*Xp*Yp + p2*( r_2 + 2*pow(Xp,2) )); 
    Xdp = Xdp_rotation + Xdp_tangential;

    double Ydp_rotation = Yp*(1 + k1*r_2 + k2*pow(r_2,2) + k3*pow(r_2,3));
    double Ydp_tangential = Yp+( p1*(r_2 + 2*pow(Yp,2) ) + 2*p2*Xp*Yp);
    Ydp = Ydp_rotation + Ydp_tangential;

    dis_pcs_point.x = (int)Xdp;
    dis_pcs_point.y = (int)Ydp;

}

void ImageReconstruct::cal_unDistortion_old(cv::Point2i dis_pcs_point,cv::Point2i &orj_pcs_point){
    double k1 = project_distCoeffs.at<double>(0,0);
    double k2 = project_distCoeffs.at<double>(1,0);
    double p1 = project_distCoeffs.at<double>(2,0);
    double p2 = project_distCoeffs.at<double>(3,0);
    // !! k3 bozulmanın yüksek oldugundan emin olana kadar kullanma
    double k3 = 0;//project_distCoeffs.at<double>(4,1);

    std::vector<cv::Point2f> dis_points;
    std::vector<cv::Point2f> orj_points;

    dis_points.push_back(cv::Point2f( (double)dis_pcs_point.x,(double)dis_pcs_point.y ));

    double fx_opencv = Camera->focalLength;//(Camera->focalLength / Camera->sensor.pixelW) / Camera->sensor.pixelW;
    double fy_opencv = Camera->focalLength;//(Camera->focalLength / Camera->sensor.pixelH) / Camera->sensor.pixelH;

    cv::Mat_<double> cam(3,3); cam << fx_opencv,0,Camera->sensor.Cx,0,fy_opencv,Camera->sensor.Cy,0,0,1;
    cv::Mat_<double> dist(1,5); dist << k1,k2,p1,p2,k3;
    std::cout << "cal_unDistortion-> opencv Camera Matris: " << cv::format(cam,cv::Formatter::FMT_NUMPY) << std::endl;
    std::cout << "cal_unDistortion-> opencv dist Matris: " << cv::format(dist,cv::Formatter::FMT_NUMPY) << std::endl;

    //cv::undistortImagePoints(dis_points,orj_points,cam,dist);
    std::cout << "cal_unDistortion-> dis_point: " << dis_points << std::endl;
    std::cout << "cal_unDistortion-> undis_point: " << orj_points << std::endl;
    orj_pcs_point.x = (int)orj_points[0].x;
    orj_pcs_point.y = (int)orj_points[0].y;
}

void ImageReconstruct::cal_unDistortion(cv::Point2d dis_ics_point,cv::Point2d &orj_ics_point){
    double k1 = project_distCoeffs.at<double>(0,1);
    double k2 = project_distCoeffs.at<double>(1,1);
    double p1 = project_distCoeffs.at<double>(2,1);
    double p2 = project_distCoeffs.at<double>(3,1);
    // !! k3 bozulmanın yüksek oldugundan emin olana kadar kullanma
    double k3 = 0;//project_distCoeffs.at<double>(4,1);

    std::vector<cv::Point2f> dis_points;
    std::vector<cv::Point2f> orj_points;

    dis_points.push_back(cv::Point2f( (double)dis_ics_point.x,(double)dis_ics_point.y ));

    double fx_opencv = (Camera->focalLength / Camera->sensor.pixelW) / Camera->sensor.pixelW;
    double fy_opencv = (Camera->focalLength / Camera->sensor.pixelH) / Camera->sensor.pixelH;

    std::cout << "fx_opencv: " << fx_opencv  << "fy_opencv: " << fy_opencv << std::endl;
    cv::Mat_<double> cam(3,3); cam << fx,0,cx,0,fy,cy,0,0,1;
    cv::Mat_<double> real_cam(3,3); cam << Camera->focalLength / Camera->sensor.pixelW,0,Camera->sensor.Cx,0,Camera->focalLength / Camera->sensor.pixelH,Camera->sensor.Cy,0,0,1;
    
    cv::Mat_<double> dist(1,5); dist << k1,k2,p1,p2,k3;

    std::cout << "cal_unDistortion-> opencv Camera Matris: " << cv::format(cam,cv::Formatter::FMT_NUMPY) << std::endl;
    std::cout << "cal_unDistortion-> real Camera Matris: " << cv::format(real_cam,cv::Formatter::FMT_NUMPY) << std::endl; 
    std::cout << "cal_unDistortion-> opencv dist Matris: " << cv::format(cam,cv::Formatter::FMT_NUMPY) << std::endl;

    cv::undistortPoints(dis_points,orj_points,cam,dist);
    std::cout << "cal_unDistortion-> dis_point: " << dis_points << std::endl;
    std::cout << "cal_unDistortion-> undis_point: " << orj_points << std::endl;
    orj_ics_point.x = orj_points[0].x;
    orj_ics_point.y = orj_points[0].y;

    // orj_ics_point.x = dis_ics_point.x;
    // orj_ics_point.y = dis_ics_point.y;

}

void ImageReconstruct::cal_Rotation(double roll,double pitch,double yaw,int flag){
    uav_Pitch = pitch;
    uav_Roll = roll;
    uav_Yaw = yaw;

    switch (flag)
    {
    case 3:
    {
        // eigen ile yapılandır.
        Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitZ());
        Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitX());

        Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;

        Eigen::Matrix3d rotationMatrix = q.matrix();

        break;
    }
    case 2:
    {
        // opencv yontemi yapılandır.
        // cv::Mat_<double> R_x(3,3);
        // R_x << 1, 0, 0,
        //     0, cos(E_x[0]), -sin(E_x[0]),
        //     0, sin(E_x[0]), cos(E_x[0]);

        // cv::Mat_<double> R_y(3,3);

        // R_y << cos(E_y[1]), 0, sin(E_y[1]),
        //     0, 1, 0
        //     -sin(E_y[1]), 0, cos(E_y[1]);

        // cv::Mat_<double> R_z(3,3);

        // R_z << cos(E_z[2]), -sin(E_z[2]), 0,
        //     sin(E_z[2]), cos(E_z[2]), 0,
        //     0, 0, 1;
        // R = R_x * R_y * R_z;
    }

    
    case 1:
    {
        roll = roll * (M_PI/180);
        pitch = pitch * (M_PI/180);
        yaw = yaw * (M_PI/180);
        Eigen::Matrix3d R_x,R_y,R_z;
        R_x << 1,    0,        0,
               0,cos(pitch),-sin(pitch),
               0,sin(pitch),cos(pitch);

        R_y << cos(roll), 0, sin(roll),
               0        ,  1,        0  ,
              -sin(roll),0, cos(roll);
        R_z << cos(yaw),-sin(yaw),0,
               sin(yaw),cos(yaw),0,
               0,        0,     1;

        rotationMatrix = R_x * R_y * R_z;

    }
 }
}



void ImageReconstruct::cvt_Image2World(cv::Point2i ocs_point,cv::Point3d &wcs_point,cv::Point2i &pcs_point,cv::Point2d &ics_point,short verbose){
    //cv::Point2i pcs_point;
    //cv::Point2d ics_point;
    cv::Point3d ccs_point;

    cvt_ocs2pcs(ocs_point,pcs_point);
    if(verbose==1)
    {
        std::clog << "\nocs -> pcs" << std::endl;
        std::clog << "----------" << std::endl;
        std::clog <<"ocs_point: " << ocs_point.x << " , " << ocs_point.y << " piksel "  << std::endl;
        std::clog <<"pcs_point: " << pcs_point.x << " , " << pcs_point.y << " piksel "  << std::endl;

    }

    cvt_pcs2ics(pcs_point,ics_point);
    if(verbose==1){
        std::clog << "\npcs -> ics" << std::endl;
        std::clog << "----------" << std::endl;
        std::clog <<"pcs_point: " << pcs_point.x << " , " << pcs_point.y << " piksel "  << std::endl;
        std::clog <<"ics_point: " << ics_point.x << " , " << ics_point.y << " mm "  << std::endl; 
    }

    // bozulma dikkate alınmamaktadır. !!!

    cvt_ics2ccs(ics_point,ccs_point);
    if(verbose==1){
        std::clog << "\nics -> ccs" << std::endl;
        std::clog << "----------" << std::endl;
        std::clog <<"ics_point: " << ics_point.x << " , " << ics_point.y << " mm "  << std::endl;
        std::clog <<"ccs_point: " << ccs_point.x << " , " << ccs_point.y << " , " << ccs_point.z << " m "  << std::endl;
        
    }

    cvt_ccs2wcs(ccs_point,wcs_point);
    if(verbose==1){
        std::clog << "\ncss -> wcs" << std::endl;
        std::clog << "----------" << std::endl;
        std::clog <<"ccs_point: " << ccs_point.x << " , " << ccs_point.y << " , " << ccs_point.z << " m "  <<  std::endl;
        std::clog <<"wcs_point: " << wcs_point.x << " , " << wcs_point.y << " , " << wcs_point.z << " m "  << std::endl;
    }

}

void ImageReconstruct::cvt_wcs2ccs(cv::Point3d wcs_point,cv::Point3d &ccs_point){

    Eigen::Matrix4d Rt = get_Rt();
        
    Eigen::Vector4d P_world;
    P_world << wcs_point.x, wcs_point.y, wcs_point.z,1;
    Eigen::Vector4d P_camera = Rt.inverse() * P_world;
    ccs_point.x = P_camera(0);
    ccs_point.y = P_camera(1);
    ccs_point.z = P_camera(2);
}

void ImageReconstruct::cvt_ccs2wcs(cv::Point3d ccs_point,cv::Point3d &wcs_point){
    // donusum işlemi için
    // p_world[x,y,z,1] = R * T * c_world[x,y,z,1]
    // 4 boyutlu girer çikar

    // T matrisi birimi m yada cm'dir dünya uzayının tipindedir
    // rotation matrisi ise donusum anlamlarını tasır

    Eigen::Matrix4d Rt = get_Rt();
    
 
    Eigen::Vector4d P_camera;

    P_camera << ccs_point.x, ccs_point.y, ccs_point.z,1;

    Eigen::Vector4d P_world;
    P_world = Rt * P_camera;
    wcs_point.x = P_world(0);
    wcs_point.y = P_world(1);
    wcs_point.z = P_world(2);

}

void ImageReconstruct::cvt_ccs2ics(cv::Point3d ccs_point,cv::Point2d &ics_point){
     int flag = get_worldType() == "m" ? METER2MINIMETER : CANTIMETER2MINIMETER;


    double Xc = convertUnit(ccs_point.x,flag);
    double Yc = convertUnit(ccs_point.y,flag);
    double Zc = convertUnit(ccs_point.z,flag);//calZ_W2C();  

    double Xi;
    double Yi;

    Yi = Camera->focalLength * (Yc/Zc);
    Xi = Camera->focalLength * (Xc/Zc);

    ics_point.x=Xi;
    ics_point.y=Yi;
}

void ImageReconstruct::cvt_ics2ccs(cv::Point2d ics_point,cv::Point3d &ccs_point){
    double Xi = ics_point.x;
    double Yi = ics_point.y;


    // !!!! Zc nedir ? 
    // -> Kamera kordinat sisteminde z ekseni tam ana noktadan geçer (pricipal point).
    // x ve y ekseni orjinleri kamerayı kesecek şekilde geçerler.
    // bu durumda yere dik bakan bir kamerada Zc konumu; 
    double Xc,Yc,Zc;
    

    Zc = calZ_W2C();
    Yc = (Zc*Yi) / Camera->focalLength;
    Xc = (Zc*Xi) / Camera->focalLength;

    // unutma mm cinsinden döner


    // unutma mm cinsinden doner bunu ceviricez
    int flag = get_worldType() == "m" ? MINIMETER2METER : MINIMETER2CANTIMETER;

    ccs_point.x=convertUnit(Xc,flag);
    ccs_point.y=convertUnit(Yc,flag);
    ccs_point.z=convertUnit(Zc,flag);
}

void ImageReconstruct::cvt_ics2pcs(cv::Point2d ics_point,cv::Point2i &pcs_point){

    // ics kordinat sistemi zaten mm oldugu için donusum yapmadık 
    // buna dikkat et
    double Xi = ics_point.x;
    double Yi = ics_point.y;

    int Xp,Yp;
    Xp = (int)(Xi/Camera->sensor.pixelW + Camera->sensor.Cx);
    Yp = (int)(Yi/Camera->sensor.pixelH + Camera->sensor.Cy);

    pcs_point.x = Xp;
    pcs_point.y = Yp;
    
}

void ImageReconstruct::cvt_pcs2ics(cv::Point2i pcs_point,cv::Point2d &ics_point){
    double Xp = (double)pcs_point.x;
    double Yp = (double)pcs_point.y;

    // cevrem sabitleri zaten mm cinsinden oldugundan dolayı donusum sonucu mm cinsinden cikacaktir.
    double Xi,Yi;

    Xi = (Xp - Camera->sensor.Cx) * Camera->sensor.pixelW;
    Yi = (Yp - Camera->sensor.Cy) * Camera->sensor.pixelH;

    ics_point.x = Xi;
    ics_point.y = Yi;
}

void ImageReconstruct::cvt_pcs2ocs(cv::Point2i pcs_point,cv::Point2i &ocs_point){
    double Xp = (double)pcs_point.x; 
    double Yp = (double)pcs_point.y;

    int Xo,Yo;

    Yo = (int)(( (Camera->sensor.numberPixelsH - Yp) / Camera->sensor.numberPixelsH ) * H);
    Xo = (int)(( (Camera->sensor.numberPixelsW - Xp) / Camera->sensor.numberPixelsW ) * W);

    ocs_point.x = Xo;
    ocs_point.y = Yo;

}

void ImageReconstruct::cvt_ocs2pcs(cv::Point2i ocs_point,cv::Point2i &pcs_point){
    double Xo = (double)ocs_point.x; 
    double Yo = (double)ocs_point.y;

    int Xp,Yp;

    Yp = (int)(Camera->sensor.numberPixelsH - ( Yo * Camera->sensor.numberPixelsH) / H);
    Xp = (int)(Camera->sensor.numberPixelsW - ( Xo * Camera->sensor.numberPixelsW) / W);

    pcs_point.x = Xp;
    pcs_point.y = Yp;

}



// setter fonkisyonlar

void ImageReconstruct::set_worldType(int type){
    world_type = type;
}

void ImageReconstruct::set_Zc(double new_z){
    // Z ekseni öncelikle dünya kordinat sistemine göre ifade edilmelidir.
    global_Zc = new_z;
}

void ImageReconstruct::set_uavAnglees(double pitch,double roll){
    uav_Pitch = pitch;
    uav_Roll = roll;
}

void ImageReconstruct::set_rotMat(cv::Mat &new_rot){
    rotMat = new_rot.clone();
};

void ImageReconstruct::set_tvec(cv::Mat &new_tvec){
    tvec = new_tvec.clone();
}

void ImageReconstruct::set_cameraMat(cv::Mat &cam_mat){
    cv::Mat_<double> cam_3f = cam_mat;
    fx = cam_3f.at<double>(0,0);
    fy = cam_3f.at<double>(1,1);
    cx = cam_3f.at<double>(0,2);
    cy = cam_3f.at<double>(1,2);

}

void ImageReconstruct::set_coef(cv::Mat &new_coef) {
    distCoeffs = new_coef.clone();
    project_distCoeffs = distCoeffs;


}

void ImageReconstruct::set_matrix_from_file(cv::String url){
    cv::FileStorage fs(url, cv::FileStorage::READ);
    cv::Mat camera_matris,distCoeffs,Rotation_vector,Translation_vector;
    fs["cameraMatrix"] >> camera_matris;
    fs["distCoeffs"] >> distCoeffs;
    fs["Rotation_vector"] >> Rotation_vector;
    fs["Translation_vector"] >> Translation_vector;
    set_cameraMat(camera_matris);
    set_tvec(Translation_vector);
    set_rotMat(Rotation_vector);
    set_coef(distCoeffs);

    fs.release();
}

void ImageReconstruct::set_camera_from_file(cv::String url,cameraSpecs &_camera){
    Camera = &_camera;
    cv::FileStorage fs(url, cv::FileStorage::READ);
    Sensor camera_sensor;
    fs["sensor_name"] >> camera_sensor.name;
    fs["number_of_pixels_v"] >> camera_sensor.numberPixelsH;
    fs["number_of_pixels_h"] >> camera_sensor.numberPixelsW;
    fs["pixel_size_v"] >> camera_sensor.pixelH;
    fs["pixel_size_h"] >> camera_sensor.pixelW;
    fs["chip_size_v"] >> camera_sensor.sensorH;
    fs["chip_size_h"] >> camera_sensor.sensorW;

    double focal_length;
    std::string camera_name;
    fs["focal_length"] >> focal_length;
    fs["camera_name"] >> camera_name;

    Camera->create(camera_name,camera_sensor,focal_length);
}

void ImageReconstruct::set_camera_Manuel(
            std::string camera_name,
            std::string sensor_name,
            double pixelH,
            double pixelW,
            double numberPixelsH,
            double numberPixelsW,
            double sensorH,
            double sensorW,
            double focalLength,
            cameraSpecs &_camera
            )
{
    Camera = &_camera;
    Sensor camera_sensor;
    camera_sensor.name = sensor_name;
    camera_sensor.numberPixelsH = numberPixelsH;
    camera_sensor.numberPixelsW = numberPixelsW;
    camera_sensor.pixelH = pixelH;
    camera_sensor.pixelW = pixelW;
    camera_sensor.sensorH = sensorH;
    camera_sensor.sensorW = sensorW;
    
    Camera->create(camera_name,camera_sensor,focalLength);
}

void ImageReconstruct::set_Camera(cameraSpecs &_camera){
    Camera = &_camera;
}


void ImageReconstruct::set_frame(short _H, short _W){
    H = _H;
    W = _W;
};

void ImageReconstruct::set_translatationMatrix(double x,double y,double z){
    translatationMatrix(0) = x;
    translatationMatrix(1) = y;
    translatationMatrix(2) = z;
}


// getter fonksiyonlar

std::string ImageReconstruct::get_worldType(){
    std::string ret;
    switch (world_type)
    {
    case WORLDTYPE::METER:
        
        ret= "m";
        break;
    case WORLDTYPE::CENTIMETER:
        
        ret= "cm";
        break;
    case WORLDTYPE::MINIMETER:
        ret = "mm";
        break;
    }

    return ret;
}

bool ImageReconstruct::check_matrix(){
    return true;
}

Eigen::Matrix4d ImageReconstruct::get_Rt(){
    Eigen::Matrix4d Rt,Rt_demo;
    Rt_demo <<  rotationMatrix(0,0), rotationMatrix(0,1), rotationMatrix(0,2),translatationMatrix(0),
                rotationMatrix(1,0), rotationMatrix(1,1), rotationMatrix(1,2), translatationMatrix(1),
                rotationMatrix(2,0), rotationMatrix(2,1), rotationMatrix(2,2), translatationMatrix(2),
                0, 0 ,0 ,1;

    //std::cout << "Rt_demo: " << Rt_demo << std::endl;
    
    // Rt << 1, 0, 0,0,
    //      0, 1, 0, 0,
    //      0, 0, 1,-72,
    //      0, 0 ,0 ,1;
    return Rt_demo;
}




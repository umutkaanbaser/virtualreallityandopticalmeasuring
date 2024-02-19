#include <opencv2/core.hpp> 
#include <opencv2/imgproc.hpp> 
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp> 


#include <iostream> 
#include <vector>
#include <ctype.h>
#include <math.h>
#include <Eigen/Dense>

#include <cameraSpecs.hpp>

#define METER2CANTIMETER 1
#define METER2MINIMETER 2
#define CANTIMETER2METER 3
#define CANTIMETER2MINIMETER 4
#define MINIMETER2METER 5
#define MINIMETER2CANTIMETER 6

#define WITH_EIGEN 1
#define WITH_CV 2

enum WORLDTYPE {
    METER,
    CENTIMETER,
    MINIMETER,
};

enum DISTROTION {
    LENS,
    ROT_TAN,
    INCLINED
};

class ImageReconstruct {
    public:
        cv::Mat rotMat; // R
        cv::Mat tvec; // t
        Eigen::Matrix3d rotationMatrix;
        Eigen::Vector3d translatationMatrix;
        cv::Mat distCoeffs; //k1,k2,p1,p2,k3,k4,k6,s1,s2,s3,tx,ty
        cv::Mat_<double> project_distCoeffs; // distCoeffs yansıması verilerine ulasmak için
        double fx;
        double fy;
        double cx;
        double cy;


        // kamera fiziksel parametreleri.
        cameraSpecs * Camera;

        double uav_Pitch,uav_Roll,uav_Yaw; // !! yav 0 kabul edilir eger gerekirse yaw eksenide dahil edilebilir.
        int global_Zc;
        short int hfov;
        short int vfov;
        double Fpv;
        double Fph;
        short int H;
        short int W;
        int frame_id;
        int world_type=WORLDTYPE::METER;


    public:
        void warning();
        ImageReconstruct();
        ~ImageReconstruct();
        

        // matematiksel işlem fonksiyonları
        
        // -- Zc hesaplanması
        void cal_Fp();
        double calZ_W2C();
        double convertUnit(double value,int flag=1);
        double calRange(cv::Point2i cord_2b,double point_altute=0);

        // -- Bozulma hesaplamaları
        void cal_Distortion(cv::Point2i orj_pcs_point,cv::Point2i &dis_pcs_point);
        void cal_unDistortion(cv::Point2d dis_ics_point,cv::Point2d &orj_ics_point);
        void cal_unDistortion_old(cv::Point2i dis_pcs_point,cv::Point2i &orj_pcs_point);
        void cal_Rotation(double roll,double pitch,double yaw,int flag=1);

        // !!! ana cagiralacak olan fonksiyon budur.!
        void cvt_Image2World(cv::Point2i ocs_point,cv::Point3d &wcs_point,cv::Point2i &pcs_point,cv::Point2d &ics_point,short verbose=0);
        void cvt_wcs2ccs(cv::Point3d wcs_point,cv::Point3d &ccs_point);
        void cvt_ccs2wcs(cv::Point3d ccs_point,cv::Point3d &wcs_point);
        void cvt_ccs2ics(cv::Point3d ccs_point,cv::Point2d &ics_point);
        void cvt_ics2ccs(cv::Point2d ics_point,cv::Point3d &ccs_point);
        void cvt_ics2pcs(cv::Point2d ics_point,cv::Point2i &pcs_point);
        void cvt_pcs2ics(cv::Point2i pcs_point,cv::Point2d &ics_point);
        void cvt_pcs2ocs(cv::Point2i pcs_point,cv::Point2i &ocs_point);
        void cvt_ocs2pcs(cv::Point2i ocs_point,cv::Point2i &pcs_point);


        // setter fonksiyonlar
        void set_worldType(int type=WORLDTYPE::METER);
        void set_Zc(double new_z);
        void set_uavAnglees(double pitch,double roll);
        void set_rotMat(cv::Mat &new_rot);
        void set_tvec(cv::Mat &new_tvec);
        void set_cameraMat(cv::Mat &cam_mat);
        void set_coef(cv::Mat &new_coef);
        void set_matrix_from_file(cv::String url);
        void set_camera_from_file(cv::String url,cameraSpecs &_camera);
        void set_camera_Manuel(
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
        );
        void set_frame(short _H,short _W);
        void set_translatationMatrix(double x,double y,double z);
        void set_Camera(cameraSpecs &_camera);


        // getter fonksiyonlar
        std::string get_worldType();
        bool check_matrix();
        Eigen::Matrix4d get_Rt();
};


#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/video.hpp>


#include <ImageReconstruct.hpp>
#include <cameraSpecs.hpp>


// TODO
// ----
// 1- geriyi ileri alırken, ileriyide geri alırken aynalama yanısıma yapıyo oldugu gibi degil tam tersi x,y'ye gidiyor bi bak!
// 2- yerin düzlüğüne göre hesaplama yapmayı unutma

// genel degiskenler / global variables
// ------------------------------------
bool moveObject = false;
int mousePosition[2] = {0,0};
int startPosition[2] = {0,0};
int endPosition[2]   = {0,0};
int boxSize          = 30;
cameraSpecs Camera; // Kamera'nın algılayıcı ve sensor bilgilerini içerir. | it has got informantions about Camera and Sensor
ImageReconstruct Reconstructer;  // Reconstructer objemiziden kopya oluşturduk | We created a copy from Reconstructer object
cv::Point3d object3dCenter(0,0,10);
std::vector<cv::Point3d> object3dPoints = {
    cv::Point3d(object3dCenter.x-boxSize,     object3dCenter.y-boxSize,    object3dCenter.z-boxSize), 
    cv::Point3d(object3dCenter.x+boxSize,     object3dCenter.y-boxSize,    object3dCenter.z-boxSize),
    cv::Point3d(object3dCenter.x+boxSize,     object3dCenter.y+boxSize,    object3dCenter.z-boxSize),
    cv::Point3d(object3dCenter.x-boxSize,     object3dCenter.y+boxSize,    object3dCenter.z-boxSize),
    cv::Point3d(object3dCenter.x-boxSize,     object3dCenter.y-boxSize,    object3dCenter.z+boxSize),
    cv::Point3d(object3dCenter.x+boxSize,     object3dCenter.y-boxSize,    object3dCenter.z+boxSize),
    cv::Point3d(object3dCenter.x+boxSize,     object3dCenter.y+boxSize,    object3dCenter.z+boxSize),
    cv::Point3d(object3dCenter.x-boxSize,     object3dCenter.y+boxSize,    object3dCenter.z+boxSize)
};
// ------------------------------------


// kullanışlı fonksiyonlar / usefull funcs
// ---------------------------------------
void        update3dPositions();
void        calculateMousePosinWorld(cv::Point2i pos2d);
void        calculate3dto2dPos(std::vector<cv::Point3d> pos3d,std::vector<cv::Point2i> &pos2d);

void        drawInformation(cv::Mat &img);
void        drawLine(cv::Mat &img);
void        drawObject(cv::Mat &img);

void        MouseCallbackControl(int event, int x, int y, int flags, void* userdata);
// ---------------------------------------


int main() {
    // ImageReconstruct Ayarları [adjusting / setup]
    // -------------------------------------

    Camera.setFov(58.2,73.2); // Kamera'nın görüş açısını bildiririz. | We set the fov information about the camera
    Reconstructer.set_camera_Manuel("iphone camera", // hangi kamera | which camera
    "6Splus", // hangi algılayıcı | which sensor
    0.00155, // piksel yüksekliği | pixel heigth
    0.00155, // piksel genişliği | pixel weigth
    3024, // dikeydeki piksel sayısı | count of vertical pixels 
    4032, // yataydaki piksel sayısı | count of horizantical pixels 
    4.68, // algılayıcı yüksekliği | sensor heigth
    6.24, // algılayıcı genişliği | sensor weigth
    4.2, // odak uzaklığı | focal length
    Camera // Camera objesini bu bilgilerin içine yazılması için veririz | we are giving to Camera object for it write the information on the object
    ); 

    Reconstructer.set_worldType(WORLDTYPE::CENTIMETER); // ölçüm sistemini söyleriz | we are setting measurement
    Reconstructer.set_frame(756,1008);  // görüntü boyutunu | image size
    Reconstructer.cal_Rotation(0,-58,0,1);// // kameranın gerçek dünyada bulunduğu euler açıları | Euler angles where the camera is located in the real world
    Reconstructer.set_translatationMatrix(0,-150,-150.0); // kameranın gerçek dünyada ki konumu | the position where the camera is located in the real world
    Reconstructer.set_Zc(-150.0); // zeminden yüksekliği (konumdaki z ekseni ile aynıdır!) | height above ground (same as z-axis at location!)
    Reconstructer.cal_Fp(); // mesafeÖlçer(range Finder) için fovAçı Donusumu hesaplanır | Calculate Fov to Degree tansform for range Finder

    // -------------------------------------



    // opencv ayarları [adjusting]
    // ---------------------------
    cv::Mat image, showImage, prcImage;
    image = cv::imread("../files/example.jpg");
    char key;
    cv::Size showSize(1008, 756); //1008, 756
    cv::namedWindow("3d-reconstruct");
    cv::setMouseCallback("3d-reconstruct", MouseCallbackControl, NULL);
    // ---------------------------

    // ana (main) dongu (loop)
    // -----------------------
    
    for(;;){
        showImage = image.clone();
        cv::resize(showImage,showImage,showSize);

        drawInformation(showImage);
    
        if(moveObject) {
            drawObject(showImage);
        }
        else {
            drawLine(showImage);
        }
        
        cv::imshow("3d-reconstruct",showImage);
        key = cv::waitKey(1);
        if (key == 'q'){
            break;
        }
        else if(key == 'w'){
            boxSize = std::min(100,boxSize+10);
        }
        else if(key == 's'){
             boxSize = std::max(10,boxSize-10);
        }
    }
    // -----------------------
    
    return 0;
}


void update3dPositions(){

    object3dPoints[0] = cv::Point3d(object3dCenter.x-boxSize,     object3dCenter.y-boxSize,    object3dCenter.z-boxSize);
    object3dPoints[1] = cv::Point3d(object3dCenter.x+boxSize,     object3dCenter.y-boxSize,    object3dCenter.z-boxSize);
    object3dPoints[2] = cv::Point3d(object3dCenter.x+boxSize,     object3dCenter.y+boxSize,    object3dCenter.z-boxSize);
    object3dPoints[3] = cv::Point3d(object3dCenter.x-boxSize,     object3dCenter.y+boxSize,    object3dCenter.z-boxSize);
    object3dPoints[4] = cv::Point3d(object3dCenter.x-boxSize,     object3dCenter.y-boxSize,    object3dCenter.z+boxSize);
    object3dPoints[5] = cv::Point3d(object3dCenter.x+boxSize,     object3dCenter.y-boxSize,    object3dCenter.z+boxSize);
    object3dPoints[6] = cv::Point3d(object3dCenter.x+boxSize,     object3dCenter.y+boxSize,    object3dCenter.z+boxSize);
    object3dPoints[7] = cv::Point3d(object3dCenter.x-boxSize,     object3dCenter.y+boxSize,    object3dCenter.z+boxSize);

}

void calculateMousePosinWorld(cv::Point2i pos2d){
    cv::Point2d ics_point;
    cv::Point2i pcs_point;
    double Zc = Reconstructer.calRange(pos2d);
    std::cout << "Zc: " << Zc << std::endl;
    Reconstructer.set_Zc(Zc);
    Reconstructer.cvt_Image2World(pos2d,object3dCenter,pcs_point,ics_point);
    
}

void calculate3dto2dPos(std::vector<cv::Point3d> pos3d,std::vector<cv::Point2i> &pos2d){

    cv::Point3d ccs_point;
    cv::Point2d ics_point;
    cv::Point2i pcs_point;
    cv::Point2i ocs_point;
    //std::cout << "-------" << std::endl;
    for(uint i =0;i<pos3d.size();i++){
        Reconstructer.cvt_wcs2ccs(pos3d[i],ccs_point);
        Reconstructer.cvt_ccs2ics(ccs_point,ics_point);
        Reconstructer.cvt_ics2pcs(ics_point,pcs_point);
        Reconstructer.cvt_pcs2ocs(pcs_point,ocs_point);
        pos2d.push_back(ocs_point);
        //std::cout << "ters donusum: " << pos3d[i] << "----" << ocs_point << std::endl;
    }
}

void drawInformation(cv::Mat &img){}

void drawLine(cv::Mat &img){

    // görselleştirmeler | imaginations
    // ----------------
    cv::line(img,cv::Point2i(startPosition[0],startPosition[1]),cv::Point2i(endPosition[0],endPosition[1]),cv::Scalar(255,0,0),3);
    cv::circle(img,cv::Point2i(startPosition[0],startPosition[1]),10,cv::Scalar(0,255,0),-1);
    cv::circle(img,cv::Point2i(endPosition[0],endPosition[1]),10,cv::Scalar(0,0,255),-1);
    
    // 3d hesaplama işlemleri | 3d measuretions
    // ----------------------
    cv::Point3d start_wcs_point,end_wcs_point;
    cv::Point3d ccs_point;
    cv::Point2d ics_point;
    cv::Point2i pcs_point;

    // baslangic noktası | start point
    double Range = Reconstructer.calRange(cv::Point2i(startPosition[0],startPosition[1]));
    Reconstructer.set_Zc(Range);
    Reconstructer.cvt_Image2World(cv::Point2i(startPosition[0],startPosition[1]),start_wcs_point,pcs_point,ics_point,0);
    
    // bitis noktası | end point
    Range = Reconstructer.calRange(cv::Point2i(endPosition[0],endPosition[1]));
    Reconstructer.set_Zc(Range);
    Reconstructer.cvt_Image2World(cv::Point2i(endPosition[0],endPosition[1]),end_wcs_point,pcs_point,ics_point,0);

    double distance=0.0;
    distance =  sqrt(pow( (start_wcs_point.x - end_wcs_point.x) , 2 ) + pow( (start_wcs_point.y - end_wcs_point.y) , 2 ));

    cv::Point2i center(startPosition[0] - ((startPosition[0] - endPosition[0])/2),startPosition[1] - ((startPosition[1] - endPosition[1])/2));
    cv::putText(img,std::to_string(distance).substr(0,4)+" cm",center,cv::FONT_HERSHEY_COMPLEX,1,cv::Scalar(0,255,255),2);    

}

void drawObject(cv::Mat &img){
    std::vector<cv::Point2i> object2dPoints;
    calculate3dto2dPos(object3dPoints,object2dPoints);

    cv::line(img,object2dPoints[0],object2dPoints[4],cv::Scalar(0,0,255),3);
    cv::line(img,object2dPoints[1],object2dPoints[5],cv::Scalar(0,0,255),3);
    cv::line(img,object2dPoints[2],object2dPoints[6],cv::Scalar(0,0,255),3);
    cv::line(img,object2dPoints[3],object2dPoints[7],cv::Scalar(0,0,255),3);

    cv::line(img,object2dPoints[0],object2dPoints[1],cv::Scalar(0,255,0),3);
    cv::line(img,object2dPoints[1],object2dPoints[2],cv::Scalar(255,0,0),3);
    cv::line(img,object2dPoints[2],object2dPoints[3],cv::Scalar(0,255,0),3);
    cv::line(img,object2dPoints[3],object2dPoints[0],cv::Scalar(255,0,0),3);

    cv::line(img,object2dPoints[4],object2dPoints[5],cv::Scalar(0,255,0),3);
    cv::line(img,object2dPoints[5],object2dPoints[6],cv::Scalar(255,0,0),3);
    cv::line(img,object2dPoints[6],object2dPoints[7],cv::Scalar(0,255,0),3);
    cv::line(img,object2dPoints[7],object2dPoints[4],cv::Scalar(255,0,0),3);



}

void MouseCallbackControl(int event, int x, int y, int flags, void* userdata) {
    if  ( event == cv::EVENT_LBUTTONDOWN && !moveObject ) //
    {

        startPosition[0] = x;
        startPosition[1] = y;
    }
    else if  ( event ==  cv::EVENT_RBUTTONDOWN && !moveObject )
    {
        endPosition[0] = x;
        endPosition[1] = y;

    }
    else if ( event ==  cv::EVENT_MOUSEMOVE && moveObject)
    {
        calculateMousePosinWorld(cv::Point2i(x,y));
        update3dPositions();
    }
    else if  ( event ==  cv::EVENT_MBUTTONDOWN )
    {
        moveObject = !moveObject;
    }

}

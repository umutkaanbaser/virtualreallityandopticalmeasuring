# [TR] 'Image Reconstruct' ile Sanal Gerçeklik ve Optik Ölçekleme
<b><i>([EN] Please check this <a href="https://github.com/umutkaanbaser/virtualreallityandopticalmeasuring/blob/main/BENIOKU.md">README.md</a> file for english document.) </i></b></br>
Okuduğunuz için teşekkür ederim. Bu projede , 'Sanal gerçeklik sistemleri nasıl çalışır ? ' ve 'tek kamera ile gerçek santimetre olarak ölçüm yapabilir miyiz ?' sorularının cevaplarını aradım. ararken, opencv'nin <a href="https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html">Camera Calibration and 3D Reconstruction</a> dökümanını buldum ve bu bir dünyanın kapısnı araladı benim için. (Onu ve okuğun diğer dökümanları   <a href="https://github.com/umutkaanbaser/imagereconstruct">ImageReconstruct</a> repomda bulabilirsiniz.). 


Bu projede gerçek Dünya'ya bir hayali küp çizmeye çalıştım ve gerçek santimetre ile ölçüm yapmayı hedefledim. Edindiğim sonuçlar için aşağıdaki videoları inceleyebilirsiniz.


https://github.com/umutkaanbaser/virtualreallityandopticalmeasuring/assets/59193897/75573805-f98e-45a0-9b15-1b69b0eb84c3

<br/>



https://github.com/umutkaanbaser/virtualreallityandopticalmeasuring/assets/59193897/ffaed67d-afb3-48fb-9a02-a2f5fbf932ce


##  Gerekli Mödüller
1. <a href="https://docs.opencv.org/4.x/d7/d9f/tutorial_linux_install.html">Opencv 4.8.0</a>
2. <a href="https://eigen.tuxfamily.org/index.php?title=Main_Page">Eigen</a>

##  Nasıl Çalıştırılır
CMake build işlemi yaparak kodları derleyebilir ardından çalıştırabilirsiniz.
```
mkdir build && cd build
cmake ..
make
./image_reconstruct
```

## Nasıl Kullanılır ?
* Programı 2 durumu vardır. Bunlar sanal gerçeklik ve optik ölçmedir.
1. İlk durum Optik Ölçme: Seçmiş olduğunuz uzunluğu santimetre olarak ölçer.
-  Farenin sol tuşu ile başlangıcı, sağ tuşu ile bitiş noktasını seçiniz. Program otomatik olarak arasındaki mesafeyi gerçek şekilde ölçeçektir.
2. İkinci Durum Sanal Gerçeklik: Görsele bir küp çizebilirsiniz. İsterseniz bu kübün boyutunu büyültüp küçültebilir, konumu değiştirebilirisiniz.
- daha büyük küp için 'w' tuşuna basın yada daha küçük küp için 's' tuşuna başın ve fareyi istediğiniz noktaya taşıyın. Program kübü otomatik olarak çizer.

Enjoy!

## What are this code doing ?
#### we declared global variables firstly. These are using in functions
```c++

// genel degiskenler / global variables
// ------------------------------------
bool moveObject = false;
int mousePosition[2] = {0,0};
int startPosition[2] = {0,0};
int endPosition[2]   = {0,0};
..
..
..
```
#### after we are setupping Image Reconstruct and Camera Specs. this step is really important. We have to give real and true values about Camera. You can find Camera Specs in the camera's datasheet.
```c++
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
```

#### the next step is Camera positions infos. We have to give real centimetres values and real degree values (euler) know about camera position. 
```C++
Reconstructer.set_worldType(WORLDTYPE::CENTIMETER); // ölçüm sistemini söyleriz | we are setting measurement
Reconstructer.set_frame(756,1008);  // görüntü boyutunu | image size
Reconstructer.cal_Rotation(0,-58,0,1);// // kameranın gerçek dünyada bulunduğu euler açıları | Euler angles where the camera is located in the real world
Reconstructer.set_translatationMatrix(0,-150,-150.0); // kameranın gerçek dünyada ki konumu | the position where the camera is located in the real world
Reconstructer.set_Zc(-150.0); // zeminden yüksekliği (konumdaki z ekseni ile aynıdır!) | height above ground (same as z-axis at location!)
Reconstructer.cal_Fp(); // mesafeÖlçer(range Finder) için fovAçı Donusumu hesaplanır | Calculate Fov to Degree tansform for range Finder
```
#### and last step is loop. this is end of global informations about this code. We will look detail functions next part.
```c++
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

```

## How can we calculate 2d Image cordinate to 3d real world cordinate ? 
Actually , we are already using this calculation in a lot of functions in this code. We are calculating 2d to 3d transform and 3d to 2d transformation.
#### look this 'calculateMousePosinWorld' functions!
we are transforming mouse cordinate in image(2d) to 3d real world cordinate in this function for move our cube!. We are moving our cube like it is real. But actually it is just <i>Virtual reallty</i>. Please look visualreallityObject.mp4 video.
```c++
void calculateMousePosinWorld(cv::Point2i pos2d){
    cv::Point2d ics_point;
    cv::Point2i pcs_point;
    double Zc = Reconstructer.calRange(pos2d);
    std::cout << "Zc: " << Zc << std::endl;
    Reconstructer.set_Zc(Zc);
    Reconstructer.cvt_Image2World(pos2d,object3dCenter,pcs_point,ics_point);
}
```
steps:
1. we are waiting 2d image positions (pos2d).
2. we are declare diffrent cordinate system variables for calculation (please look <a href="https://github.com/umutkaanbaser/imagereconstruct/tree/main/Documentations">Image Reconstruct Documentations</a>)
3. We have to know 'how many centimetres far from camera this object ? '. We are using 'calRange' method for this info.
4. and we are setting this info with set_Zc (Zc is Z axis value in Camera Cordinate System)
5. the last step is we are transform with 'cvt_Image2World' method. Please check 'Image Reconstruct Documentations' for more info.

other functions not diffrent this process.
#### You can use this function when you need to know  any object's real position. Just say the object coordinate in Image to this function and get real world coordinate.

#### So Where is 3D real world point in Image ?
this question's answer is in 'calculate3dto2dPos' function !
we are just doing <i>coordinate System Transformation </i> in this code.
this functions do Virtual Reality ! it is calculating cube 3d edge coordinate in 2d Image. THen we do just drawwing.
```c++
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
```
steps:
1. We need to know 3d positions(pos3d) and we are writing result in pos2d variable.
2. We are declaring pinhole Camera's coordinate systems.
3. Then we are transform between coordinate systems.
#### You can use this function when you need draw 3d object to Image

#### Other functions about drawing and Opencv Highgui. You can get more info about it with this <a href="https://docs.opencv.org/3.4/d7/dfc/group__highgui.html">documentation</a>


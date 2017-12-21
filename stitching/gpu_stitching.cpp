#include "gpu_stitching.hpp"


using namespace cv;
using namespace std;











//Given camera position compute perpendicular bisector paramater k and b
Point2f Stitching::calculateLineParam(const CameraPos c1,const CameraPos c2, float & k, float & b){
    float x1 = c1.x;
    float y1 = c1.y;
    float x2 = c2.x;
    float y2 = c2.y;
    float x0 = (x1+x2)*0.5;
    float y0 = (y1+y2)*0.5;
    
    float k1 = (y2-y1)/(x2-x1);
    k = -1/k1;
    b = (y1+y2)*0.5 - k *(x1+x2)*0.5;
}


void Stitching::makeROI(ROI &roi,Point2f p0, Point2f p1,Point2f p2,Point2f p3,Point2f p4){
    roi.vertices.push_back(p0);
    roi.vertices.push_back(p1);
    roi.vertices.push_back(p2);
    roi.vertices.push_back(p3);
    roi.vertices.push_back(p4);
    
}
void Stitching::makeROI(ROI &roi,Point2f p0, Point2f p1,Point2f p2,Point2f p3){
    roi.vertices.push_back(p0);
    roi.vertices.push_back(p1);
    roi.vertices.push_back(p2);
    roi.vertices.push_back(p3);
    
}
Point2f Stitching::computeROI( ROI * roi,Point2f * cameras_pos,int fusion_width){
    int offset =0;
    
    Point2f p;
    float x,y;
    //左前交点
    Point2f p0;
    float k02,b02;
    calculateLineParam(cameras_pos[FRONT_CAMERA],cameras_pos[LEFT_CAMERA],k02,b02);
    b02-=offset;
    x=0;
    y = k02 * x + b02;
    p0.x = x;
    p0.y = y;
    ////circle(image_stitching,p0,5,Scalar(255,0,255),5);
    //上下
    Point2f p0_f(p0.x,p0.y-fusion_width);
    ////circle(image_stitching,p0_f,5,Scalar(255,0,255),5);
    Point2f p0_b(p0.x,p0.y+fusion_width);
    ////circle(image_stitching,p0_b,5,Scalar(255,0,255),5);
    //右前交点
    Point2f p1;
    float k03,b03;
    calculateLineParam(cameras_pos[FRONT_CAMERA],cameras_pos[RIGHT_CAMERA],k03,b03);
    b03-=offset;
    x=out_width;
    y = k03 * x + b03;
    p1.x = x;
    p1.y = y;
    //circle(image_stitching,p1,5,Scalar(255,0,255),5);
    //上下
    Point2f p1_f(p1.x,p1.y-fusion_width);
    //circle(image_stitching,p1_f,5,Scalar(255,0,255),5);
    Point2f p1_b(p1.x,p1.y+fusion_width);
    //circle(image_stitching,p1_b,5,Scalar(255,0,255),5);
    //上左右交点
    Point2f p2;
    float intersect_x,intersect_y;
    intersect_x =  (b02-b03)/(k03-k02);
    intersect_y = k02 * intersect_x + b02;
    p2.x = intersect_x;
    p2.y = intersect_y;
    //circle(image_stitching,p2,5,Scalar(255,0,255),5);
    //上下
    Point2f p2_f(p2.x,p2.y-fusion_width);
    //circle(image_stitching,p2_f,5,Scalar(255,0,255),5);
    Point2f p2_b(p2.x,p2.y+fusion_width);
    //circle(image_stitching,p2_b,5,Scalar(255,0,255),5);
    //左后交点
    Point2f p3;
    float k12,b12;
    calculateLineParam(cameras_pos[BACK_CAMERA],cameras_pos[LEFT_CAMERA],k12,b12);
    b12+=offset;
    x=0;
    y = k12 * x + b12;
    p3.x = x;
    p3.y = y;
    //circle(image_stitching,p3,5,Scalar(255,0,255),5);
    Point2f p3_f(p3.x,p3.y-fusion_width);
    //circle(image_stitching,p3_f,5,Scalar(255,0,255),5);
    Point2f p3_b(p3.x,p3.y+fusion_width);
    //circle(image_stitching,p3_b,5,Scalar(255,0,255),5);
    //右后交点
    Point2f p4;
    float k13,b13;
    calculateLineParam(cameras_pos[BACK_CAMERA],cameras_pos[RIGHT_CAMERA],k13,b13);
    b13+=offset;
    x=out_width;
    y = k13 * x + b13;
    p4.x = x;
    p4.y = y;
    //circle(image_stitching,p4,5,Scalar(255,0,255),5);
    Point2f p4_f(p4.x,p4.y-fusion_width);
    //circle(image_stitching,p4_f,5,Scalar(255,0,255),5);
    Point2f p4_b(p4.x,p4.y+fusion_width);
    //circle(image_stitching,p4_b,5,Scalar(255,0,255),5);
    //下左右交点
    Point2f p5;
    intersect_x =  (b12-b13)/(k13-k12);
    intersect_y = k12 * intersect_x + b12;
    p5.x = intersect_x;
    p5.y = intersect_y;
    //circle(image_stitching,p5,5,Scalar(255,0,255),5);
    
    Point2f p5_f(p5.x,p5.y-fusion_width);
    //circle(image_stitching,p5_f,5,Scalar(255,0,255),5);
    Point2f p5_b(p5.x,p5.y+fusion_width);
    //circle(image_stitching,p5_b,5,Scalar(255,0,255),5);
    
    
    //Make 8 ROI
    makeROI(roi[0],Point2f(0,0),p0_f,p2_f,p1_f,Point2f(out_width,0));
    makeROI(roi[1],p0_f,p0_b,p2_b,p2_f);
    makeROI(roi[2],p0_b,p3_f,p5_f,p2_b);
    makeROI(roi[3],p3_f,p3_b,p5_b,p5_f);
    makeROI(roi[4],p3_b,Point2f(0,out_height),Point2f(out_width,out_height),p4_b,p5_b);
    makeROI(roi[5],p5_f,p5_b,p4_b,p4_f);
    makeROI(roi[6],p2_b,p5_f,p4_f,p1_b);
    makeROI(roi[7],p2_f,p2_b,p1_b,p1_f);
    
    
    
    
    return p;
    
}

void Stitching::makeMask(ROI &roi,Mat & mask){
    mask = Mat(out_height, out_width, CV_8UC1,Scalar(0));
    
    
    vector<Point> ROI_Poly;
    
    //approximate precision paramater
    approxPolyDP(roi.vertices, ROI_Poly, 1.0, true);
    // Fill polygon white
    fillConvexPoly(mask, &ROI_Poly[0], ROI_Poly.size(), 255, 8, 0);
    //imshow("fff",mask);
    
}
/*Alpha blending*/
void Stitching::make3DMask(ROI &roi,Mat & p_masks_fusion,Mat & n_masks_fusion,int fusion_width){
    //generate positive mask_fusion
    p_masks_fusion = Mat(out_height, out_width, CV_32FC3,Scalar(0,0,0));
    vector<Point> ROI_Poly,vertices;
    //make sure not overflow
    float y_start = (roi.vertices[0].y - 2.0 > 0)?(roi.vertices[0].y-2.0):0;
    float y_end =(roi.vertices[3].y + 2.0 < out_height)?(roi.vertices[3].y + 2.0):out_height;
    vertices.push_back(Point(roi.vertices[0].x,y_start));
    vertices.push_back(Point(roi.vertices[0].x,y_start+4));
    vertices.push_back(Point(roi.vertices[3].x,y_end));
    vertices.push_back(Point(roi.vertices[3].x,y_end-4));
    
    fusion_width =fusion_width*2;
    
    for(int i=0;i<fusion_width;i++){
        vertices[0].y+=1;
        vertices[1].y = vertices[0].y + 1;
        vertices[3].y+=1;
        vertices[2].y = vertices[3].y + 1;
        approxPolyDP(vertices, ROI_Poly, 1.0, true);
        //tricky way to fill with flexible weight
        fillConvexPoly(p_masks_fusion, &ROI_Poly[0], ROI_Poly.size(), Scalar(1-i/(float)fusion_width,1-i/(float)fusion_width,1-i/(float)fusion_width), 8, 0);
        
        //cout<<1-i/(float)fusion_width<<endl;
    }
    //generate negative mask_fusion = 1-positive mask_fusion
    Mat mask = Mat(out_height, out_width, CV_32FC3,Scalar(0,0,0));
    approxPolyDP(roi.vertices, ROI_Poly, 1.0, true);
    // Fill polygon white
    fillConvexPoly(mask, &ROI_Poly[0], ROI_Poly.size(), Scalar(1.0,1.0,1.0), 8, 0);
    
    subtract(mask,p_masks_fusion,n_masks_fusion);
    /*
     Mat scaled,scaled1,scaled2;
     resize(mask,scaled,Size(),0.5,0.5);
     resize(p_masks_fusion,scaled1,Size(),0.5,0.5);
     resize(n_masks_fusion,scaled2,Size(),0.5,0.5);
     
     imshow("mask",scaled);
     imshow("pmask",scaled1);
     imshow("nmask",scaled2);
     */
    
}


void Stitching::makeMasks(Mat * masks,Mat * masks_fusion,CameraPos * cameras_pos,int fusion_width){
    ROI roi[TOTAL_ROI];
    computeROI(roi, cameras_pos,fusion_width);
    
    for(int i=0;i<TOTAL_ROI;i++){
        makeMask(roi[i],masks[i]);
    }
    //fusion ROI
    make3DMask(roi[1],masks_fusion[0],masks_fusion[4],fusion_width);
    make3DMask(roi[3],masks_fusion[1],masks_fusion[5],fusion_width);
    make3DMask(roi[5],masks_fusion[2],masks_fusion[6],fusion_width);
    make3DMask(roi[7],masks_fusion[3],masks_fusion[7],fusion_width);
    
}



//90ms timecost for 1650*1250 resolution stitching
//30ms timecost for 825*625 resolution stitching
//
//10ms for 1650*1250 GPU-upload

//GPU speedup stitching
void Stitching::stitching( gpu::GpuMat * gpu_image, gpu::GpuMat *gpu_masks,gpu::GpuMat *gpu_masks_fusion,gpu::GpuMat &gpu_stitching){
    
    gpu::Stream stream;
   
    //gpu_image.copyTo(gpu_stitching,gpu_masks[0]);
    
    
    //There is no need of fusion for ROI 0,2,4,6
    for(int i=0;i<TOTAL_ROI;i+=2){
        //add(image_stitching,images_32FC3[i/2],image_stitching,masks[i]);
       
        gpu_image[i/2].copyTo(gpu_stitching,gpu_masks[i]);
    }
   
   
    
    //ROI1
    gpu::GpuMat  layer1,layer2,mixed_layer;
    //cout<<"1"<<endl;
    gpu::multiply(gpu_image[0],gpu_masks_fusion[0],layer1,stream);
    //cout<<"2"<<endl;
    gpu::multiply(gpu_image[1],gpu_masks_fusion[4],layer2,stream);
    //cout<<"3"<<endl;
    gpu::add(layer1,layer2,mixed_layer);
    //cout<<"4"<<endl;
    mixed_layer.copyTo(gpu_stitching,gpu_masks[1]);

    
    //ROI3
    
    gpu::multiply(gpu_image[1],gpu_masks_fusion[1],layer1,stream);
    gpu::multiply(gpu_image[2],gpu_masks_fusion[5],layer2,stream);
    gpu::add(layer1,layer2,mixed_layer);
    mixed_layer.copyTo(gpu_stitching,gpu_masks[3]);
    //ROI5
    
    gpu::multiply(gpu_image[3],gpu_masks_fusion[2],layer1,stream);
    gpu::multiply(gpu_image[2],gpu_masks_fusion[6],layer2,stream);
    gpu::add(layer1,layer2,mixed_layer);
    mixed_layer.copyTo(gpu_stitching,gpu_masks[5]);
    //ROI7
    
    gpu::multiply(gpu_image[0],gpu_masks_fusion[3],layer1,stream);
    gpu::multiply(gpu_image[3],gpu_masks_fusion[7],layer2,stream);
    gpu::add(layer1,layer2,mixed_layer);
    mixed_layer.copyTo(gpu_stitching,gpu_masks[7]);
    
    //imshow("mixed",mixed_layer);
    
    
   
    
    
}

void Stitching::saveConfiguration(Mat * masks,Mat * masks_fusion,CameraPos * cameras_pos,int fusion_width){
    
}
void Stitching::loadMap(String filename,Mat & mapx, Mat & mapy){
    //load remap param
    Mat map = imread(filename, CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR);
    //Mat testImage = imread("./map/u0.jpg");
   // Mat mapx, mapy,output;
    vector<Mat> ch3;
    vector<Mat> ch2;//(2);
    
    
    //split channel
    split(map, ch3);
    mapx = ch3[0].clone();
    mapy = ch3[1].clone();
   
    
    //16U to 16S
    mapx.convertTo(mapx, CV_32F);
    mapy.convertTo(mapy, CV_32F);
    
    //cout<<"cpu"<<" xtype:"<<mapx.type()<<" ytype"<<mapy.type()<<" xsize"<<mapx.size()<<endl;

  
    
}
void Stitching::loadConfiguration(gpu::GpuMat * gpu_map_x,gpu::GpuMat * gpu_map_y,Mat * masks,gpu::GpuMat *GPU_masks, Mat * masks_fusion,gpu::GpuMat * GPU_masks_fusion,CameraPos * cameras_pos,int fusion_width){
    //load masks to GPU
    for(int i=0;i<8;i++){
        GPU_masks[i].upload(masks[i]);
    }
    for(int i=0;i<8;i++){
        GPU_masks_fusion[i].upload(masks_fusion[i]);
    }
    
    
    Mat mapx,mapy;
    loadMap("./map/map0.png",mapx,mapy);
   
    gpu_map_x[0].upload(mapx);gpu_map_y[0].upload(mapy);
    loadMap("./map/map1.png",mapx,mapy);
    gpu_map_x[1].upload(mapx);gpu_map_y[1].upload(mapy);
    loadMap("./map/map2.png",mapx,mapy);
    gpu_map_x[2].upload(mapx);gpu_map_y[2].upload(mapy);
    loadMap("./map/map3.png",mapx,mapy);
    gpu_map_x[3].upload(mapx);gpu_map_y[3].upload(mapy);

    
    
}

void Stitching::undistort(gpu::GpuMat * src,gpu::GpuMat * dst,gpu::GpuMat * gpu_mapx,gpu::GpuMat * gpu_mapy,int flags=INTER_LINEAR){
    Mat view0,view1,view2,view3;
   
    for(int i=0;i<4;i++)
        gpu::remap(src[i], dst[i], gpu_mapx[i], gpu_mapy[i], flags);
    
}
void Stitching::initCuda(){
    //init gpu
    cudaSetDevice(0);
    cudaSetDeviceFlags(cudaDeviceMapHost);
    cudaFree(0);
}
Stitching::Stitching(int in_height, int in_width,int out_height,int out_width){
    
    
    this->setInputResolution(in_height,in_width);
    this->setOutputResolution(out_height,out_width);
    
    //fixed camera position
    
    cameras_pos[FRONT_CAMERA] = Point2f(625,400);//front
    cameras_pos[BACK_CAMERA] = Point2f(625,1400);//back
    cameras_pos[LEFT_CAMERA]= Point2f(500,800);//left
    cameras_pos[RIGHT_CAMERA] = Point2f(750,800);//right
    
    //makeMasks
    makeMasks(masks,masks_fusion,cameras_pos,FUSION_WIDTH);
    
    //ToDo
    saveConfiguration(masks,masks_fusion,cameras_pos,FUSION_WIDTH);
    
    
    
    /*----------output initialize*/
    cudaMallocManaged((void**)&p_stichingDataAddress, 3*out_width*out_height*sizeof(float));
    
   
    
    cout<<"load configuration"<<endl;
    
    loadConfiguration(gpu_mapx,gpu_mapy,masks,GPU_masks,masks_fusion,GPU_masks_fusion,cameras_pos,FUSION_WIDTH);
    
    //Wait for GPU Synchronize()
    cudaDeviceSynchronize();
    
}
void Stitching::updateImage(float * Img[4]){
    //Undistorted GPU Image
    
    for(int i=0;i<4;i++){
        
        gpu_image[i] = cv::gpu::GpuMat(in_height,in_width, CV_32FC3, Img[i]);
    }

  
}


void Stitching::loadImages(){
    Mat images_8UC3[4],images_32FC3[4];//currently not use image 8UC3 because of the need of matrix multiply
    

    images_8UC3[0] =  imread("./map/u0.jpg");
    images_8UC3[1] =  imread("./map/u1.jpg");
    images_8UC3[2] =  imread("./map/u2.jpg");
    images_8UC3[3] =  imread("./map/u3.jpg");
    
    
    

    
    
    /*Allocate Zero-Copy memmory, give it to pointer p_DataAddress */
    float * p_DataAddress[4];
    for(int i=0;i<4;i++){
        cudaMallocManaged((void**)&p_DataAddress[i], 3*in_height*in_width*sizeof(float));
        images_32FC3[i] = Mat(in_height,in_width, CV_32FC3, p_DataAddress[i]);
    }

    
    for(int i=0;i<4;i++){
        gpu_image[i] = cv::gpu::GpuMat(in_height,in_width, CV_32FC3, p_DataAddress[i]);
    }
    
    
    Mat scaled[4];
    for(int i=0;i<4;i++)
        resize(images_8UC3[i],scaled[i],Size(),1,1);
    
    scaled[0].convertTo( images_32FC3[0], CV_32FC3, 1.0/255);
    scaled[1].convertTo( images_32FC3[1], CV_32FC3, 1.0/255);
    scaled[2].convertTo( images_32FC3[2], CV_32FC3, 1.0/255);
    scaled[3].convertTo( images_32FC3[3], CV_32FC3, 1.0/255);
    
    
   
}
void Stitching::stitching(Mat & result){
   
    cv::gpu::GpuMat gpu_stitching(out_height,out_width, CV_32FC3, p_stichingDataAddress);
   
    undistort(gpu_image,gpu_undistortImg,gpu_mapx,gpu_mapy,INTER_LINEAR);
    
    stitching(gpu_undistortImg,GPU_masks,GPU_masks_fusion,gpu_stitching);
   
   
   
    result = Mat(out_height,out_width,CV_32FC3,p_stichingDataAddress);

}
void Stitching::setOutputResolution(int out_height, int out_width){
    this->out_height = out_height;
    this->out_width = out_width;
    std::cout<< "Output Size: " << out_width <<"x"<<out_height<<std::endl;
    
}
void Stitching::setInputResolution(int in_height,int in_width){
    this->in_height =  in_height;
    this->in_width = in_width;
    std::cout<< "Input Size: " << in_width <<"x"<<in_height<<std::endl;
}

/*
int main(int argc, char *argv[])
{
    Mat result;
    Stitching stitch;
    for(int i=0;i<100;i++){
        stitch.loadImages();
  
        stitch.stitching(result);
    }
   
}
 */

   
    
    
 

   
    
    
    
    
    
    
    
   
 
    
    
   
   
    
    
    
    
    
    
    
 
    
  
    

   



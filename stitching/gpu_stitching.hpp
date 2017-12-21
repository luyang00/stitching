#ifndef __GPU_STITCHING
#define __GPU_STITCHING
#include <iostream>
#include "opencv2/opencv.hpp"
#include "opencv2/gpu/gpu.hpp"
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/legacy/legacy.hpp"
#include <opencv2/core/cuda_devptrs.hpp>
#include "device_launch_parameters.h"
#include "cuda.h"
#include "cuda_runtime_api.h"
#include <time.h>



using namespace cv;
using namespace std;


#define IMAGE_WIDTH 1250
#define IMAGE_HEIGHT 1650

#define TOTAL_ROI 8
/*camera position*/
#define FRONT_CAMERA 0
#define BACK_CAMERA 1
#define LEFT_CAMERA 2
#define RIGHT_CAMERA 3
#define FUSION_WIDTH 70
struct ROI{
    vector<Point> vertices;
};

typedef Point2f CameraPos;


class Stitching{
private:
    Point2f cameras_pos[4];
    Mat masks[8];
    Mat masks_fusion[8];
    cv::gpu::GpuMat gpu_image[4];
    cv::gpu::GpuMat gpu_undistortImg[4];
    cv::gpu::GpuMat GPU_masks[8],GPU_masks_fusion[8],gpu_mapx[4],gpu_mapy[4];
   
    int out_height,out_width;
    int in_height,in_width;
    
    //temp solution
    float * p_stichingDataAddress;
   
    void saveConfiguration(Mat * masks,Mat * masks_fusion,CameraPos * cameras_pos,int fusion_width);
    Point2f calculateLineParam(const CameraPos c1,const CameraPos c2, float & k, float & b);
    void makeROI(ROI &roi,Point2f p0, Point2f p1,Point2f p2,Point2f p3,Point2f p4);
    void makeROI(ROI &roi,Point2f p0, Point2f p1,Point2f p2,Point2f p3);
    Point2f computeROI( ROI * roi,Point2f * cameras_pos,int fusion_width);
    void makeMask(ROI &roi,Mat & mask);
    void make3DMask(ROI &roi,Mat & p_masks_fusion,Mat & n_masks_fusion,int fusion_width);
    void makeMasks(Mat * masks,Mat * masks_fusion,CameraPos * cameras_pos,int fusion_width);
    void stitching( gpu::GpuMat * gpu_image, gpu::GpuMat *gpu_masks,gpu::GpuMat *gpu_masks_fusion,gpu::GpuMat &gpu_stitching);
    
    void loadMap(String filename,Mat & mapx, Mat & mapy);
    void loadConfiguration(gpu::GpuMat * gpu_map_x,gpu::GpuMat * gpu_map_y,Mat * masks,gpu::GpuMat *GPU_masks, Mat * masks_fusion,gpu::GpuMat * GPU_masks_fusion,CameraPos * cameras_pos,int fusion_width);
    void undistort(gpu::GpuMat * src,gpu::GpuMat * dst,gpu::GpuMat * gpu_mapx,gpu::GpuMat * gpu_mapy,int flags);
    
        
public:
    Stitching(int in_height, int in_width, int out_height,int out_width);
    void setOutputResolution(int out_height, int out_width);
    void setInputResolution(int in_height,int in_width);
    void setCameraPosition(Point2f camera_pos[]);
    void loadImages();
    void updateImage(float * Img[]);
    void stitching(Mat & result);
    void initCuda();
    
};




#endif


#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <assert.h>
#include <linux/videodev2.h>
#include <unistd.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/gpu/gpu.hpp>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <thread>
#include <mutex>
#include <glog/logging.h>
#include <string.h>
#include <vector>
#include <arpa/inet.h>
#include <signal.h>
#include <unistd.h>
//gpu acc
#include "cuda.h"
#include "cuda_runtime_api.h"
#include "opencv2/gpu/gpu.hpp"
#include "gpu_stitching.hpp"

//APT::AptColor *apt_color=NULL;
#define PIXEL_FORMAT "YUV422"
const int width=1280;
const int height=720;
#define DEFAULT_BUFFER_SIZE 4096000

#define SEND_COMMAND 1
#define SEND_DATA 2

using namespace cv;
//cv::Mat rgb;
bool addLine=false;
typedef struct
{
    void *start;
    int length;
}BUFTYPE;
typedef struct
{
    BUFTYPE *user_buffer;
    int n_buffer;
}mmap_buffer;

typedef struct camera_v4l2_core
{
    int fd;
    mmap_buffer user_buffer;
    void* other_data;
    int status;
    int index;
    int cameraID;
}camera_v4l2_core;
int nFrames=0;
const char *dev_name=NULL;
struct calmcar_server;
typedef struct calmcar_client
{
    int socket;
    unsigned char* buffer=NULL;
    unsigned int buffer_len=0;
    bool send_data=false;
    std::thread* thread_handle;
    std::mutex lock;
    calmcar_server *server;
}calmcar_client;
typedef struct calmcar_server
{
    int socket=0;
    int port=-1;
    const char* ip=NULL;
    unsigned char* buffer=NULL;
    unsigned int buffer_len=0;
    //bool send_data=false;
    std::thread *thread_handle;
    
    std::vector<calmcar_client*> client_pool;
    //std::mutex lock;
}calmcar_server;

calmcar_server *server=NULL;
int open_camera(camera_v4l2_core* camera,const char* dev)
{
    if(camera==NULL || dev==NULL)
    {
        return -1;
    }
    int fd;
    if((fd=access(dev,F_OK))!=0)
    {
        return -1;
    }
    if((fd=open(dev,O_RDWR|O_NONBLOCK))<0)
    {
        return -1;
    }
    memset(camera,0,sizeof(camera_v4l2_core));
    camera->fd=fd;
    return 0;
}



int init_mmap(int fd,mmap_buffer *user_buffer)
{
    int i=0;
    struct v4l2_requestbuffers reqbuf;
    bzero(&reqbuf,sizeof(reqbuf));
    reqbuf.count=4;
    reqbuf.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;
    reqbuf.memory=V4L2_MEMORY_MMAP;
    
    //申请视频缓冲区(这个缓冲区位于内核空间，需要通过mmap映射)
    //这一步操作可能会修改reqbuf.count的值，修改为实际成功申请缓冲区个数
    if(-1==ioctl(fd,VIDIOC_REQBUFS,&reqbuf))
    {
        perror("Fail to ioctl 'VIDIOC_REQBUFS'");
        return -1;
    }
    user_buffer->n_buffer=reqbuf.count;
    user_buffer->user_buffer=(BUFTYPE*)calloc(reqbuf.count,sizeof(BUFTYPE));
    if(user_buffer->user_buffer==NULL)
    {
        perror("mmap is NULL\n");
        return -1;
    }
    //将内核缓冲区映射到用户进程空间
    for(i=0;i<reqbuf.count;i++)
    {
        struct v4l2_buffer buf;
        bzero(&buf,sizeof(buf));
        buf.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory=V4L2_MEMORY_MMAP;
        buf.index=i;
        if(-1==ioctl(fd,VIDIOC_QUERYBUF,&buf))
        {
            perror("Fail to ioctl 'VIDIOC_QUERYBUF'");
            return -1;
        }
        user_buffer->user_buffer[i].length=buf.length;
        user_buffer->user_buffer[i].start=
        mmap(
             NULL,
             buf.length,
             PROT_READ | PROT_WRITE,
             MAP_SHARED,
             fd,buf.m.offset
             );
        if(MAP_FAILED==user_buffer->user_buffer[i].start)
        {
            return -1;
        }
    }
    return 0;
}
void print_camera_info(camera_v4l2_core *camera)
{
    if(camera==NULL)
    {
        return;
    }
    struct v4l2_format fmt;
    fmt.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ioctl(camera->fd,VIDIOC_G_FMT,&fmt);
    printf("Current data format:\nwidth:%d\nheight:%d\n",fmt.fmt.pix.width,fmt.fmt.pix.height);
}

int init_camera(camera_v4l2_core* camera)
{
    struct v4l2_fmtdesc fmt;
    struct v4l2_capability cap;
    struct v4l2_format stream_fmt;
    int ret;
    if(!camera)
    {
        return -1;
    }
    memset(&fmt,0,sizeof(fmt));
    fmt.index=0;
    fmt.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;
    while((ret=ioctl(camera->fd,VIDIOC_ENUM_FMT,&fmt))==0)
    {
        fmt.index++;
        printf("{pixelformat = %c%c%c%c},description = '%s'\n",
               fmt.pixelformat & 0xff,(fmt.pixelformat >> 8)&0xff,
               (fmt.pixelformat >> 16) & 0xff,(fmt.pixelformat >> 24)&0xff,
               fmt.description);
    }
    ret=ioctl(camera->fd,VIDIOC_QUERYCAP,&cap);
    if(ret <0)
    {
        perror("FAIL to ioctl VIDIOC_QUERYCAP");
        return -1;
    }
    if(!(cap.capabilities & V4L2_BUF_TYPE_VIDEO_CAPTURE))
    {
        perror("The Current device is not a video capture device\n");
        return -1;
    }
    if(!(cap.capabilities & V4L2_CAP_STREAMING))
    {
        perror("The Current device does not support streaming i/o\n");
        return -1;
    }
    ioctl(camera->fd,VIDIOC_G_FMT,&stream_fmt);
    stream_fmt.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;
    stream_fmt.fmt.pix.width=width;
    stream_fmt.fmt.pix.height=height;
    //stream_fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_SRGGB12;
    //stream_fmt.fmt.pix.pixelformat=V4L2_PIX_FMT_YUYV;
    /*    stream_fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;
     if(-1==ioctl(camera->fd,VIDIOC_S_FMT,&stream_fmt))
     {
     perror("Failed set stream format\n");
     return -1;
     }
     */
    ret=init_mmap(camera->fd,&camera->user_buffer);
    if(ret!=0)
    {
        return ret;
    }
    return 0;
}
int start_capturing(camera_v4l2_core *camera)
{
    unsigned int i;
    enum v4l2_buf_type type;
    if(!camera)
    {
        return -1;
    }
    for(i=0;i<camera->user_buffer.n_buffer;i++)
    {
        struct v4l2_buffer buf;
        bzero(&buf,sizeof(buf));
        buf.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory=V4L2_MEMORY_MMAP;
        buf.index=i;
        if(-1==ioctl(camera->fd,VIDIOC_QBUF,&buf))
        {
            perror("Fail to ioctl 'VIDIOC_QBUF'\n");
            return 0;
        }
    }
    type=V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if(-1==ioctl(camera->fd,VIDIOC_STREAMON,&type))
    {
        perror("Fail to ioctl 'VIDIOC_STREAMON'\n");
        return -1;
    }
    
    return 0;
}
void convert8bit(unsigned char* src,unsigned char* dst,int len)
{
    unsigned short *bit10=(unsigned short *)src;
    for(int i=0;i<len;i++)
    {
        unsigned short pixel=*(bit10++)>>2;
        *(dst++)=pixel;
    }
}

int read_frame(camera_v4l2_core *camera,int cameraID,float * p_Img)
{
    
    int key_pressed;
    static int img_count=0;
    struct v4l2_buffer buf;
    unsigned int i;
    bzero(&buf,sizeof(buf));
    buf.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory=V4L2_MEMORY_MMAP;
    
    
    char windowName[50];
    sprintf(windowName,"camera-%d",cameraID);
     //cv::namedWindow(windowName,CV_WINDOW_NORMAL);
    //std::cout<<windowName << std::endl;
    //从队列中取缓冲区
    if(-1==ioctl(camera->fd,VIDIOC_DQBUF,&buf))
    {
        perror("VIDIOC_DQBUF error\n");
        return -1;
    }
    if(buf.index>camera->user_buffer.n_buffer)
    {
        perror("buffer index error\n");
        return -1;
    }
    
    cv::Mat yuv_8bit(height,width,CV_8UC2,camera->user_buffer.user_buffer[buf.index].start);
    //yuv_8bit=yuv_8bit.clone();
    if(-1==ioctl(camera->fd,VIDIOC_QBUF,&buf))
    {
        perror("VIDIOC_QBUF error\n");
        return -1;
    }
    
    
   
    
    cv::Mat rgb;
    cv::cvtColor(yuv_8bit,rgb,CV_YUV2BGR_YUYV);
    rgb.convertTo(rgb,CV_32FC3,1.0/255);//7ms
    
    
    //cv::gpu::GpuMat gpu_img(height,width, CV_32FC3, p_Img);
    
    memcpy(p_Img,rgb.ptr<float>(0),3*height*width*sizeof(float));
    Mat rrgb(height,width,CV_32FC3,p_Img);
    
   
    
    //cv::imshow(windowName,rrgb);
    int keypressed = cv::waitKey(1);
    if(key_pressed == 32)
    {
        img_count++;
        //std::cout<<"save image"<<std::endl;
        char name_buffer[50];
        
        sprintf(name_buffer, "./save/dev%c-%d.jpg", dev_name[10], img_count);
        //dev_name char* int(count).jpg
        //std::cout<< name_buffer<< std::endl;
        cv::imwrite(name_buffer, rgb);
    }
    //yuv_10bit.convertTo(yuv_8bit,CV_8UC2,0.25);
    return 0;
}

void* process_thread(void* data, float * p_Img)
{
    
    //    apt_color=new APT::AptColor(height,width,APT::APT_COLOR_GRBG2RGB_12BIT);
    camera_v4l2_core* camera=(camera_v4l2_core*)data;
  

    fd_set fds;
    struct timeval tv;
    int r;
    FD_ZERO(&fds);
    FD_SET(camera->fd,&fds);
    
    tv.tv_sec=7;
    tv.tv_usec=0;
    
  
    r=select(camera->fd+1,&fds,NULL,NULL,&tv);
    if(-1==r)
    {
        
         
        perror("Fail to select");
        exit(EXIT_FAILURE);
    }
    if(0==r)
    {
        perror("select time out\n");
        return NULL;
    }
    
    read_frame(camera,camera->cameraID,p_Img);
   
    
    return 0;
}

using namespace std;
void print_help()
{
    printf("camera-test [options] \n"
           "-d device_name such as /dev/video0\n"
           "-n number of frames \n"
           "-h print help\n");
    
}

int main(int argc, char *argv[])
{
    //dev_name="/dev/video0";
    char * dev_name[4]={"/dev/video0","/dev/video1","/dev/video2","/dev/video3"};
    nFrames=20;
    
   
  
   
    
   
    camera_v4l2_core v4l2_core[4]={{0},{0},{0},{0}};
    
    
   
    for(int i=0;i<2;i++){
        
        if(open_camera(&v4l2_core[i],dev_name[i])!=0)
        {
            printf("open camera error\n");
            exit(1);
        }
        if(init_camera(&v4l2_core[i])!=0)
        {
            printf("init camera error\n");
            exit(1);
        }
        v4l2_core[i].status=1;
        print_camera_info(&v4l2_core[i]);
        
        if(start_capturing(&v4l2_core[i])!=0)
        {
            printf("start capturing error\n");
        }
        
    }
    
    float * p_Img[4];
    Mat Img[4];
    for(int i=0;i<4;i++)
    {
        cudaMallocManaged((void**)&p_Img[i], 3*height*width*sizeof(float));
        Img[i] = Mat(height,width,CV_32FC3,p_Img[i]);
    }
    
    //From gpu stitching
   
    Stitching  stitch;
    Mat result,scaled;
   
    for(;;)
    {

         //clock_t begin = clock();
       
        for(int i=0;i<2;i++){
            //read_frame(&v4l2_core[i],i);
            v4l2_core[i].cameraID =i;
            //std::cout<<i<<std::endl;
            
            //25fps
            process_thread(&v4l2_core[i],p_Img[i]);
        }
    
  
        
        for(int i=2;i<4;i++){
            //read_frame(&v4l2_core[i],i);
            v4l2_core[i].cameraID =i;
            //std::cout<<i<<std::endl;
            process_thread(&v4l2_core[i-2],p_Img[i]);
            
        }
            
      
        stitch.updateImage(p_Img);
        stitch.stitching(result);
        
        cv::resize(result,scaled,Size(),0.5,0.5);
        cv::imshow("result",scaled);
        
       
        
       
       
        
       //！ clock_t end = clock();
        //cout<<"Average Running time: "<<(double)((end-begin)/CLOCKS_PER_SEC*1000)<<"ms"<<endl;
        waitKey(1);
    }
    
    
    
        
    for(int i=0;i<4;i++)
        close(v4l2_core[i].fd);
    return 0;
    
}


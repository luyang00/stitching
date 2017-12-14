#版本  
cmake_minimum_required(VERSION 2.8)   
  
#项目名称                                        
project( stitching )       
  
#设置OpenCV所在目录，为后面的find_package提供位置                                                      
#OpenCV_DIR为OpenCV源码编译后得到的文件夹(如上面的release目录)                   
set( OpenCV_DIR /Users/yang/Code/opencv-2.4.13.4/release/)  
  
#会在OpenCV所在目录查找配置文件OpenCVConfig.cmake  
find_package( OpenCV REQUIRED )    
  
#向工程添加源文件                                         
add_executable( mask mask.cpp)
add_executable( stitching stitching.cpp) 

#添加链接库                                   
target_link_libraries( stitching ${OpenCV_LIBS} ) 
target_link_libraries( mask ${OpenCV_LIBS} ) 
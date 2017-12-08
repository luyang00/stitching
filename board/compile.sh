#!/bin/bash
protoc --cpp_out=./ calmcar.prototxt
nvcc -O3 -I/usr/local/cuda/include  camera-test.cpp   -std=c++11 -lstdc++ -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_gpu -L/usr/local/cuda/lib64 -lcuda -lcudart -lnppi -lprotobuf -lglog -lgflags -o camera-test

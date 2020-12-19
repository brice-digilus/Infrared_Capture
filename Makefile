# This makefile is a frankenstein between the MLX makefile and the Dynamixel Makefile and some extra code from me
#
# Code from from robotis is under Apache 2.0 license
#  Copyright 2017 ROBOTIS CO., LTD.
# https://github.com/ROBOTIS-GIT/DynamixelSDK 
#
# Code from melexis is under a BSD 3 clause license
#
# Rest of the code is (C) 2020 Brice Dubost
#
#    Copyright 2020 Brice Dubost
#
#    Licensed under the Apache License, Version 2.0 (the "License");
#    you may not use this file except in compliance with the License.
#    You may obtain a copy of the License at
#
#        http://www.apache.org/licenses/LICENSE-2.0
#
#    Unless required by applicable law or agreed to in writing, software
#    distributed under the License is distributed on an "AS IS" BASIS,
#    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#    See the License for the specific language governing permissions and
#    limitations under the License.
#
# It expect the Dynamixel SDK to be compiled....


all: examples servos

examples: change_addr Th_tiff_timed

servos: Servo_360 Servo_360_DXL

DXL_HEADERS_DIR = headers/dynamixel_sdk
DXL_INCL   += -I$(DXL_HEADERS_DIR)
DXL_LIBRARIES  += -ldxl_x86_cpp
DXL_LIBRARIES  += -lrt


libMLX90640_API.so: functions/MLX90640_API.o functions/MLX90640_LINUX_I2C_Driver.o
	$(CXX) -fPIC -shared $^ -o $@

libMLX90640_API.a: functions/MLX90640_API.o functions/MLX90640_LINUX_I2C_Driver.o
	ar rcs $@ $^
	ranlib $@

functions/MLX90640_API.o functions/MLX90640_RPI_I2C_Driver.o functions/MLX90640_LINUX_I2C_Driver.o : CXXFLAGS+=-fPIC -I headers -shared $(I2C_LIBS)

examples/change_addr.o examples/Th_tiff_timed.o examples/Servo_360.o: CXXFLAGS+=-std=c++11 -I lib -I headers 

examples/Servo_360_DXL.o: CXXFLAGS+=-std=c++11 -O2 -O3 -DLINUX -D_GNU_SOURCE -Wall  -I lib -I headers $(DXL_INCL)


lib/sensor_lib.o : CC=$(CXX) -std=c++11 -I headers -I lib

change_addr Servo_360_DXL : CXXFLAGS+=-I. -std=c++11

lib/nano_tiff_lib.o : CC=$(CXX) -std=c++11


Servo_360_DXL: examples/Servo_360_DXL.o lib/nano_tiff_lib.o lib/sensor_lib.o libMLX90640_API.a
	$(CXX)  $^ -o $@  $(DXL_LIBRARIES)

change_addr: examples/change_addr.o libMLX90640_API.a
	$(CXX) -L/home/pi/mlx90640-library $^ -o $@ 


clean:
	rm -f change_addr Servo_360_DXL
	rm -f examples/*.o
	rm -f functions/*.o
	rm -f *.o
	rm -f *.so
	rm -f test
	rm -f *.a

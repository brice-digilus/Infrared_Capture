/* Data structures for infrared sensor information

    Copyright 2020 Brice Dubost

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/


#ifndef sensor_lib_h
#define sensor_lib_h

#include <MLX90640_API.h>

//dynamic range for the tiff file
#define TIFF_MIN_T -10
#define TIFF_MAX_T 50

#define SEN_NX 32
#define SEN_NY 24

//structure containing sensor data
typedef struct sensor_data_t{
  //Address
  uint8_t sen_addr;
  //frame data
  uint16_t frame[834];
  
  //static float image[768];
  float Image[768];

  //temperature for each pixel
  float mlx90640To[768];

  //sensor mode: 0 interleaved, 1: chess pattern
  int mode;

  // sensor parameters
  paramsMLX90640 mlx90640_p;
  //sensor EEPROM
  uint16_t eeMLX90640[832];

  //last frame received
  int idxframe;

  //ambient temperature
  float ta;
}sensor_data_t;


void init_sensor(sensor_data_t *sen, int fps);

void add_sen_to_tiff(uint16_t *Tiff_image,float *mlx90640To, int sen_id, int n_sensors);

void make_filename_log(char *fbuffer, float *ta, int n_sensors, const char *images_folder);

#endif

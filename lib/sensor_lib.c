/** Sensor library

  This is a layer above the MLX library with convenience functions 
  and inexisting functions in the library like triggered frames
  
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
	
#include <stdint.h>
#include <stdio.h>
#include <string.h> // pulls in declaration for strlen.
#include <errno.h>
#include <sensor_lib.h>
#include <MLX90640_API.h>
#include <time.h>


void init_sensor(sensor_data_t *sen, int fps)
{

	printf("Initialize sensor on address 0x%x . . . \n",sen->sen_addr);
	int refreshrate = 1;
	switch(fps)
	{
		case 0:		//0.5 FPS
			refreshrate = 0x00;
			break;
		case 1: 	//1 FPS
			refreshrate = 0x01;
			break;
		case 2:		//2 FPS
			refreshrate = 0x02;
			break;
		case 4:		//4 FPS
			refreshrate = 0x03;
			break;
		default:
			printf ("Invalid value for FPS %d, acceptables values are 0,1,2,4 reverting to default (1)\n", fps);
	}
	MLX90640_SetRefreshRate(sen->sen_addr, refreshrate);

	//Chess mode, the one for which the sensor is designed to operate
	MLX90640_SetChessMode(sen->sen_addr);
	sen->mode = 1;
	//We get the parameters
	MLX90640_DumpEE(sen->sen_addr, sen->eeMLX90640);
	MLX90640_ExtractParameters(sen->eeMLX90640, &sen->mlx90640_p);
}





//Add the data of a given sensor to the tiff data with normalization
//this function could lie in both librairies but this is more sensor dependant than tiff dependant so its here
void add_sen_to_tiff(uint16_t *Tiff_image,float *mlx90640To, int sen_id, int n_sensors)
{
	int effx;
	
	//here fill image for tiff
	for(int y = 0; y < SEN_NY; y++)
	{
		for(int x = 0; x < SEN_NX; x++)
		{
			float val = mlx90640To[SEN_NX * ((SEN_NY-1)-y) + x];
			float valn;
			valn = val - (TIFF_MIN_T);
			valn = valn /(TIFF_MAX_T - TIFF_MIN_T);
			if(valn <0)
			  valn = 0;
			if(valn>1)
			  valn = 1;
			//We store the sensors next to each other
			effx = x + sen_id * SEN_NX;
			//and the effective nx is SEN_NX*N_SENSORS
			Tiff_image[effx + y*(SEN_NX*n_sensors)] = (uint16_t)(valn * ((1<<16)-1));
		}
	}
}




//make the filename to store the tiff and write the ambiant temperature log
void make_filename_log(char *fbuffer, float *ta, int n_sensors, const char *images_folder)
{
	time_t rawtime;
	struct tm * timeinfo;

	time ( &rawtime );
	timeinfo = localtime ( &rawtime );

	char tbuffer [80];

	strftime (tbuffer,80,"%Y%m%d_%H%M%S",timeinfo);

	snprintf(fbuffer,255,"%s/_Heat_log.txt",images_folder);
	FILE *fptr;
	fptr = fopen (fbuffer,"a");
	if (fptr == NULL)
	{
		printf ("LOG File not created okay, errno = %d\n", errno);
		return;
    }

	fprintf(fptr,"%s",tbuffer);
	for(int i_sen=0;i_sen<n_sensors;i_sen++)
		fprintf(fptr,"; %02.1f",ta[i_sen]);
	fprintf(fptr,"\r\n");
		
	snprintf(fbuffer,255,"%s/_Heat_sen_%s.tiff",images_folder,tbuffer);
	fclose (fptr);

}





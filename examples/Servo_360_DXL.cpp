/** Code to get a 360 images using two dynamixel MX-12W servo and a Melexis infrared sensor
 * 
 * * Basically move, take image, move again, take image .... write combined image with metadata
 * 
 * Yes this is NOT modern C++, it is mostly C code speaking to a C++ library
 * TODO: do some C removal
 * 
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
  
 * */

#include <sstream>
#include <string>
#include <stdint.h>
#include <iostream>
#include <cstring>
#include <fstream>
#include <chrono>
#include <thread>
#include <math.h>
#include <MLX90640_API.h> // we will use the MLX library to speak on i2c
#include <MLX90640_I2C_Driver.h>
#include <stdio.h>  // required for printf statements
#include <string.h>
#include "lib/sensor_lib.h"
#include "lib/nano_tiff_lib.h"
#include "dynamixel_sdk.h"

#include <memory>

#include "DXL_info.h"

#define VERSION "0.04"

//TODO: images folder could be in argc/argv
#define IMAGES_FOLDER "../data/images_servo"

dynamixel::PortHandler *portHandler;
dynamixel::PacketHandler *packetHandler;

//Sensor i2c address
#define MLX_BASE_ADDR 0x33

//A pixel is about a motion tolerance of 20 (1.7degree)
#define MOTION_TOLERANCE 5

//About 2 minutes for 28 images
#define SENSOR_FPS 1
#define SENSOR_DELTA_S 0.6

// //About 1.5 minutespi
// #define SENSOR_FPS 2
// #define SENSOR_DELTA_S 0.3

// //About 40 seconds
// #define SENSOR_FPS 4
// #define SENSOR_DELTA_S 0.1

#define TIFF_METADATA_MAX 8192

//structure containing axis servo data
typedef struct axis_data_t{
    //Address
    int addr;
    //Computation from angle to value is
    //value = int((angle-angle_offset)*angle_slope)%value_limit
    float angle_offset;
    float angle_slope;
    int value_limit;

    //Current angle
    float curr_angle;
    int curr_value;

}axis_data_t;




//Size of memory increment
#define ANGLE_CHUNK 10 
int load_angles(float **list_Y_angles_p,float **list_Xi_angles_p, int *list_n_angles,std::string filename)
{
    
    

    std::cout << " Reading angle file \""<<filename<<"\" \n";

    int n_angles = 0;
    int i_l = 0;
    int arr_size = ANGLE_CHUNK;

    //Here we will read the configuration file for angles !
    //Expect a pair of Y angle, Xi angle per line. eg "-20 12.34"

    float *list_Y_angles = new float[ANGLE_CHUNK];
    float *list_Xi_angles = new float[ANGLE_CHUNK];

    //https://stackoverflow.com/questions/8421170/read-floats-from-a-txt-file

    std::ifstream input(filename);
    if ( !input.is_open() )
        return -1;
    for(std::string line; std::getline(input, line); )   //read stream line by line
    {
        i_l ++;
        std::istringstream in(line);      //make a stream for the line itself
        float angle_Y, angle_Xi;
        in >> angle_Y >> angle_Xi;       //now read the whitespace-separated floats
        if (in.fail()) {
            std::cout << "   Error converting numbers, line "<<i_l<<": \""<<line<<"\" \n";
        } else {
            if(arr_size <= n_angles) //Need to reallocate memory https://stackoverflow.com/questions/3749660/how-to-resize-array-in-c
            {
                float* newlist_Y_angles = new float[arr_size + ANGLE_CHUNK];
                memcpy( newlist_Y_angles, list_Y_angles, arr_size * sizeof(int) );
                delete [] list_Y_angles;
                list_Y_angles = newlist_Y_angles;
                float* newlist_Xi_angles = new float[arr_size + ANGLE_CHUNK];
                memcpy( newlist_Xi_angles, list_Xi_angles, arr_size * sizeof(int) );
                delete [] list_Xi_angles;
                list_Xi_angles = newlist_Xi_angles;
                arr_size += ANGLE_CHUNK;
            }
            list_Y_angles[n_angles] = angle_Y;
            list_Xi_angles[n_angles] = angle_Xi;
            n_angles++;
        }
    }


    //Set "return values"
    *list_Y_angles_p = list_Y_angles;
    *list_Xi_angles_p = list_Xi_angles;
    *list_n_angles = n_angles;
    
    if(n_angles == 0)
    {
        std::cout << "No angles loaded :-(\nExciting\n";
        return -2;
    }
    else
        std::cout << "Loaded " << n_angles <<" angle";
    if(n_angles>1)
        std::cout << "s";
    std::cout <<"\n";

    return 0;

}


void sleep(float seconds)
{
    std::this_thread::sleep_for(std::chrono::microseconds(int(seconds*1000*1000)));
}



//A FUNCTION, WRITE THIS REGISTER OF THIS ADDRESS WITH THIS VALUE OF THIS LENGTH AT THIS PORT NUMBER
int SER_Write_register(int dxl_id,int data_addr, int data_value, int data_len)
{
    int dxl_comm_result;
    uint8_t dxl_error;                          // Dynamixel error

    // 1 byte data
    if (data_len == 1)
    {
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id, data_addr, data_value, &dxl_error);
    }
    else if (data_len == 2) // 2 byte data
    {
        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_id, data_addr, data_value, &dxl_error);
    }
    else
    {
        printf("Unsupported servo data length: %d\n", data_len);
        return -1;
    }

    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("Servo Communication transfer error: %s\n", packetHandler->getTxRxResult(dxl_comm_result));
        return -1;
    }
    else if (dxl_error != 0)
    {
        printf("Servo Communication response error: %s\n", packetHandler->getRxPacketError(dxl_error));
        return -1;
    }
    
    return 0;
}


int SER_Read_register(int dxl_id,int data_addr, int data_len, int *error)
{
    int dxl_comm_result;
    uint8_t dxl_error;                          // Dynamixel error

    uint16_t data_out_16;
    uint8_t data_out_8;
    int data_out; //we are pretty sure not to have a -1 as this will be a 32bit int an we read max 2 Bytes
    *error = 1;

    // 1 byte data
    if (data_len == 1)
    {
        dxl_comm_result = packetHandler->read1ByteTxRx(portHandler, dxl_id, data_addr, &data_out_8, &dxl_error);
        data_out = data_out_8;
    }
    else if (data_len == 2) // 2 byte data
    {
        dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, dxl_id, data_addr, &data_out_16, &dxl_error);
        data_out = data_out_16;
    }
    else
    {
        printf("Unsupported servo data length: %d\n", data_len);
        return 0;
    }

    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("Servo Communication transfer error: %s\n", packetHandler->getTxRxResult(dxl_comm_result));
        return 0;
    }
    else if (dxl_error != 0)
    {
        printf("Servo Communication response error: %s\n", packetHandler->getRxPacketError(dxl_error));
        return 0;
    }

    *error = 0;
    return data_out;
}



int SER_init(axis_data_t *axis_Y,axis_data_t *axis_Xi)
{
    printf("Init Servo. . . \n");

    // Initialize PortHandler instance
    // Set the port path
    // Get methods and members of PortHandlerLinux or PortHandlerWindows
    portHandler = dynamixel::PortHandler::getPortHandler(SER_DEVICENAME);

    // Initialize PacketHandler instance
    // Set the protocol version
    // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
    packetHandler = dynamixel::PacketHandler::getPacketHandler(SER_PROTOCOL_VERSION);

    // Open port
    if (portHandler->openPort())
        printf("Succeeded to open the port!\n");
    else
    {
        printf("Failed to open the port!\n");
        return -1;
    }

    // Set port baudrate
    if (portHandler->setBaudRate(SER_BAUDRATE))
        printf("Succeeded to change the baudrate!\n");
    else
    {
        printf("Failed to change the baudrate!\n");
        return -1;
    }

    //Now we initialize some parameters of the servo
    //Initialize a few values, then finish with torque enable
    int addr_list[] = {SER_ADDR_TORQUE_ENABLE,SER_ADDR_TORQUE_LIM,SER_ADDR_D_GAIN,SER_ADDR_I_GAIN,SER_ADDR_P_GAIN,SER_ADDR_MOVING_SPEED,
                            SER_ADDR_CW_LIM,SER_ADDR_CCW_LIM,SER_ADDR_ACCELERATION,SER_ADDR_PUNCH,SER_ADDR_TORQUE_ENABLE};
    int size_list[] = {SER_SIZE_TORQUE_ENABLE,SER_SIZE_TORQUE_LIM,SER_SIZE_D_GAIN,SER_SIZE_I_GAIN,SER_SIZE_P_GAIN,SER_SIZE_MOVING_SPEED,
                            SER_SIZE_CW_LIM,SER_SIZE_CCW_LIM,SER_SIZE_ACCELERATION,SER_SIZE_PUNCH,SER_SIZE_TORQUE_ENABLE};
    int value_list[] = {          1       ,         SER_VAL_TORQUE_LIM       ,       SER_VAL_D_GAIN       ,        SER_VAL_I_GAIN     ,         SER_VAL_P_GAIN    ,             SER_VAL_MOVING_SPEED     ,
                                    0      ,        4095    ,           SER_VAL_ACCELERATION        ,       SER_VAL_PUNCH    ,   1        };
    for(int i = 0; i < int(sizeof(addr_list)/sizeof(addr_list[0])); i++)
    {
        if(SER_Write_register(axis_Y->addr,addr_list[i], value_list[i], size_list[i]))
        {
            printf("Failed to initialize axis Z address %d with value %d\n",addr_list[i],value_list[i]);
            return -1;
        }
        if(SER_Write_register(axis_Xi->addr,addr_list[i], value_list[i], size_list[i]))
        {
            printf("Failed to initialize axis Xi address %d with value %d\n",addr_list[i],value_list[i]);
            return -1;
        }
    }

    printf("Init servo done !\n");
    return 0;
}


int SER_free(axis_data_t *axis_Y,axis_data_t *axis_Xi)
{
    printf("Free Servo. . . \n");

    //we remove the torque from the servo
    if(SER_Write_register(axis_Y->addr,SER_ADDR_TORQUE_ENABLE, 0, 1))
    {
        printf("Failed to free axis Z \n");
        return -1;
    }
    if(SER_Write_register(axis_Xi->addr,SER_ADDR_TORQUE_ENABLE, 0, 1))
    {
        printf("Failed to free axis Xi\n");
        return -1;
    }
    printf("Free servo done !\n");
    return 0;
}


int cyclic_distance(int a,int b, int lim)
{
    //compute distance when there is a cycle, eg distance between 4090 and 10 is 16, NOT 4080
    int aa,bb;
    if(a<b)
    {
        aa = a;
        bb = b;
    }else{
        aa = b;
        bb = a;
    }

    if((bb-aa)<int(lim/2))
    {
        return bb-aa;
    }else{
        return aa+lim-bb;
    }
}

//FUNCTION WAIT MOTION FINISHED WITH A TIMEOUT
int SER_Wait_for_motion(float timeout,int tolerance, axis_data_t *axis_Y,axis_data_t *axis_Xi)
{

    auto start_pair = std::chrono::system_clock::now();
    float elapsed_f;
    int reached;
    int Y_val, Xi_val;
    int error;
    reached = 0;
    do
    {
        sleep(0.01); //sleep 10ms
        auto end_pair = std::chrono::system_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end_pair - start_pair);
        elapsed_f = float(elapsed.count())/1000000;

        Y_val = SER_Read_register(axis_Y->addr,SER_ADDR_PRESENT_POSITION, SER_SIZE_PRESENT_POSITION, &error);
        if(error)
        {
            printf("Failed to read axis Z angle value\n");
        }
        Xi_val = SER_Read_register(axis_Xi->addr,SER_ADDR_PRESENT_POSITION, SER_SIZE_PRESENT_POSITION, &error);
        if(error)
        {
            printf("Failed to read axis Z angle value\n");
        }

        if ((cyclic_distance(Y_val,axis_Y->curr_value,axis_Y->value_limit) < tolerance) &&
            (cyclic_distance(Xi_val,axis_Xi->curr_value,axis_Xi->value_limit) < tolerance))
            reached++;
        else
        {
            reached = 0;
        }
        
        //we want to avoid overshoots so being in the same spot for 0.1s
    } while ((elapsed_f < timeout) && (reached < 10));

    if(elapsed_f >= timeout )
    {
        printf("\n\t!!!!!!!!!!!!! Warning: value not reached (%d) \n",reached);
        printf("          Y  val %5d Cval %5d cycl d %d\n", Y_val,axis_Y->curr_value,cyclic_distance(Y_val,axis_Y->curr_value,axis_Y->value_limit));
        printf("          Xi val %5d Cval %5d cycl d %d\n", Xi_val,axis_Xi->curr_value,cyclic_distance(Xi_val,axis_Xi->curr_value,axis_Xi->value_limit));
        return -1;
    }
    else
    {
        printf("   took %2.1fs \n",elapsed_f);
    }
    
    return 0;

}

float SER_calc_delta(float Z,float Z0, float Xi, float Xi0)
{
    //Calculate the biggest angle displacement on the two axis
    float d0,d1;
    d0 = Z-Z0;
    d1 = Xi-Xi0;
    if(d0<0)   d0 = -d0;
    if(d1<0)   d1 = -d1;
    if(d1>d0)
        return d1;
    return d0;

}

int SER_goto(float Y_angle,float Xi_angle,axis_data_t *axis_Y,axis_data_t *axis_Xi)
{
    int error = 0;
    int Y_val, Xi_val;
    float sleep_t;
    //0.4 second per 90 deg plus 0.1 stop pause each time is a good timing
    //0.5 second per 90 deg gives a bit more margin
    float sleep_per_90_deg = 2;
    float sleep_after_move = 2;
    float delta_angle;
    //Compute angles
    Y_val = int((Y_angle - axis_Y->angle_offset) * axis_Y->angle_slope);
    Xi_val = int((Xi_angle - axis_Xi->angle_offset) * axis_Xi->angle_slope);
    //positive modulo
    //https://stackoverflow.com/questions/14997165/fastest-way-to-get-a-positive-modulo-in-c-c
    Y_val = (Y_val % axis_Y->value_limit + axis_Y->value_limit) % axis_Y->value_limit;
    Xi_val = (Xi_val % axis_Xi->value_limit + axis_Xi->value_limit) % axis_Xi->value_limit;
    printf("   Moving Y %8.2f/%4d Xi %8.2f/%4d  ",Y_angle,Y_val,Xi_angle,Xi_val);
    //Compute sleeping time
    delta_angle = SER_calc_delta(Y_angle,axis_Y->curr_angle,Xi_angle,axis_Xi->curr_angle);
    sleep_t = sleep_per_90_deg * delta_angle/90 + sleep_after_move;
    //move here Y axis
    axis_Y->curr_angle = Y_angle;
    axis_Y->curr_value = Y_val;
    if(SER_Write_register(axis_Y->addr,SER_ADDR_GOAL_POSITION, Y_val, SER_SIZE_GOAL_POSITION))
    {
        printf("Failed to move axis Y angle %f value %d\n",Y_angle,Y_val);
        return -1;
    }
    //move here Xi axis
    axis_Xi->curr_angle = Xi_angle;
    axis_Xi->curr_value = Xi_val;
    if(SER_Write_register(axis_Xi->addr,SER_ADDR_GOAL_POSITION, Xi_val, SER_SIZE_GOAL_POSITION))
    {
        printf("Failed to move axis Xi angle %f value %d\n",Xi_angle,Xi_val);
        return -1;
    }
    //Turn on LEDs
    SER_Write_register(axis_Y->addr, SER_ADDR_LED, 1, SER_SIZE_LED);
    SER_Write_register(axis_Xi->addr,SER_ADDR_LED, 1, SER_SIZE_LED);
    //tolerance 5 is 0.4deg, about 0.1 pixel
    SER_Wait_for_motion(sleep_t,MOTION_TOLERANCE, axis_Y,axis_Xi);
    //Turn OFF LEDs
    SER_Write_register(axis_Y->addr, SER_ADDR_LED, 0, SER_SIZE_LED);
    SER_Write_register(axis_Xi->addr,SER_ADDR_LED, 0, SER_SIZE_LED);

    return error;
}


float last_elapsed()
{
    static auto start_pair = std::chrono::system_clock::now();
    static auto end_pair = std::chrono::system_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end_pair - start_pair);

    end_pair = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end_pair - start_pair);
    start_pair = end_pair;
    return float(elapsed.count())/1000000;

}


void make_tiff_metadata(char *tiff_descr_buff, int list_n_angles,float *list_Y_angles,float *list_Xi_angles, float *Ta_all, struct tm * starttime, float duration)
{
    //Write metadata buffer for TIFF
    //FIXME: META DATA MUST NOT BE LONGER THAN 8192 bytes
    memset(tiff_descr_buff,0,TIFF_METADATA_MAX*sizeof(char));
    int buff_pos;
    buff_pos = sprintf(tiff_descr_buff,"THDS_V%s",VERSION);
    buff_pos += sprintf(tiff_descr_buff + buff_pos,"//NIM:%d",list_n_angles);
    buff_pos += sprintf(tiff_descr_buff + buff_pos,"//ANH:");
    for(int i=0; i<list_n_angles;i++)
        buff_pos += sprintf(tiff_descr_buff + buff_pos,"%3.2f;", list_Y_angles[i]);
    buff_pos--; //-1 to erase last ;
    buff_pos += sprintf(tiff_descr_buff + buff_pos,"//ANV:");
    for(int i=0; i<list_n_angles;i++)
        buff_pos += sprintf(tiff_descr_buff + buff_pos,"%3.2f;", list_Xi_angles[i]);
    buff_pos--; //-1 to erase last ;
    buff_pos += sprintf(tiff_descr_buff + buff_pos,"//TA_S:"); 
    for(int i=0; i<list_n_angles;i++)
        buff_pos += sprintf(tiff_descr_buff + buff_pos,"%3.2f;", Ta_all[i]);
    buff_pos--; //-1 to erase last ;
    buff_pos += sprintf(tiff_descr_buff + buff_pos,"//SFPS:%d",SENSOR_FPS);
	buff_pos += strftime(tiff_descr_buff + buff_pos,1024,"//TI_SD:%Y%m%d//TI_ST:%H%M%S",starttime);
    buff_pos += sprintf(tiff_descr_buff + buff_pos,"//TI_DU:%.2f", duration);
}




/**
 * 
 * Base structure. This code take one measurement, the surrounding python takes care of the rest
 * By the way this is still an open question the share between C and python
 * I will use the python for the 3D scan and the S3 upload
 * 
 * Intialize,
 *   servo: init parameters and go to home
 *   and sensor
 * 
 * Put sensor in step measurement mode MLX90640_SetStepMode(uint8_t slaveAddr, int step_mode)
 * 
 * For each angle pair
 *      have servo go to angle
 *      Wait for the motion to be finished (150ms within angular tolerance)
 *      for each subframe
 *          Start measurement with MLX90640_StartMeasurement
 *          Wait for frame ready MLX90640_FrameReady with some timing to lessen the CPU (use FPS)
 *          Once ready use MLX90640_GetFrameData
 *      Add the current position to the TIFF data
 *      Save partial tiff under a different name
 *      
 * Servo go home
 * Save Tiff
 * 
 * Exit
 *  
 * */


int main(int argc, char *argv[])
{
    printf("Welcome to servo thermal  Version %s !!\n",VERSION);
    //disable buffering on stdout //https://stackoverflow.com/questions/1716296/why-does-printf-not-flush-after-the-call-unless-a-newline-is-in-the-format-strin
    setbuf(stdout, NULL);

    int error;
    float time_motion = 0;
    float time_clear = 0;
    float time_measure = 0;

    //init Axis data
    axis_data_t axis_Y, axis_Xi;

    //The data for the servos, it is for the DYNAMIXEL MX-12W 
    axis_Y.addr = SER_Z_ADDRESS;
    axis_Y.angle_offset = 180;
    axis_Y.angle_slope = 4096.0/360;
    axis_Y.value_limit = 4096;
    axis_Y.curr_angle = 0;

    axis_Xi.addr = SER_Xi_ADDRESS;
    axis_Xi.angle_offset = -90;
    axis_Xi.angle_slope = 4096.0/360;
    axis_Xi.value_limit = 4096;
    axis_Xi.curr_angle = 0;

	time_t rawtime;
	struct tm * starttime;
	time ( &rawtime );
	starttime = localtime ( &rawtime );

    error = SER_init(&axis_Y, &axis_Xi);
    //TODO: check for error
    
    //Init sensors
    sensor_data_t sen_data;
    //Dynamic data structures the old fashion way (I know that is bad !)
    //https://stackoverflow.com/questions/4029870/how-to-create-a-dynamic-array-of-integers
    //TODO: configuration file default name or on the command line !!!
    //For now just expect the conf file as argc[1]
    //and display we will read from there

    //Here we will read the configuration file !
    int list_n_angles;
    float *list_Y_angles;
    float *list_Xi_angles;
    std::string angles_filename = "angles.txt";

    //Basic argument passing, so far we consider that if there is a single argument it is the 
    //filename for the angles list.
    //In the future we might want to use getopt or similar
    if ( argc > 2 ) // argc should be 2 for correct execution
        // We print argv[0] assuming it is the program name
        std::cout<<"usage: "<< argv[0] <<" <filename>\n";
    else if (argc == 2) {
        // We assume argv[1] is a filename to open
            angles_filename = argv[1];
    }else{
        std::cout<<"Using  "<< angles_filename <<" as input file for angles\n";
    }
    

    //Loading angles, if not successful, return
    if(load_angles(&list_Y_angles,&list_Xi_angles,&list_n_angles, angles_filename))
        return -1;

    // //Very verbose, display all angles
    // std::cout << "Nangles " << list_n_angles <<"\n";
    // for(int i=0;i<list_n_angles;i++)
    //     std::cout << " Angle " << list_Y_angles[i] <<" "<< list_Xi_angles[i] <<"\n";
    // std::cout <<"\n";
    // return 0;

    //Initialize some variables
    uint16_t *Tiff_image = new uint16_t[SEN_NX*SEN_NY*list_n_angles];
    //Image description/metadata
    char tiff_descr_buff[TIFF_METADATA_MAX];
    //Sensor Temperatures
    float *Ta_all = new float[list_n_angles];

    //FIXME: fixed len for path
    char filename[255];
    char filename_partial[255];
    snprintf(filename_partial, 255, "%s/__Heat_latest.tiff", IMAGES_FOLDER);

    //reset the image
    memset(Tiff_image,0,SEN_NX*SEN_NY*list_n_angles*sizeof(uint16_t));
    memset(&sen_data,0,sizeof(sensor_data_t));
    sen_data.sen_addr = MLX_BASE_ADDR;
    init_sensor(&sen_data, SENSOR_FPS);

    float Y_angle, Xi_angle;
	auto start_pair = std::chrono::system_clock::now();
    float elapsed_f;

    //The limit here is a proxy for the TIFF meta data length,
    // starting at 300 angles it will get close to the limit of 8192 bytes
    if(list_n_angles > 300)
    {
        printf("Too many angles (%d) limit is 300\n", list_n_angles);
        return -1;
    }

    //Real start
    printf("Let's go !!!\n\tAnd do %d angles\n", list_n_angles);
    last_elapsed();

    // * For each angle pair
    for(int i_angle=0; i_angle<list_n_angles ; i_angle++)
    {
        int status;
        Y_angle = list_Y_angles[i_angle];
        Xi_angle = list_Xi_angles[i_angle];

        auto end_pair = std::chrono::system_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end_pair - start_pair);
        elapsed_f = float(elapsed.count())/1000000;

        //Moving
        printf("\n=== New Angle === %3d/%d T %4.1fs --", i_angle+1, list_n_angles, elapsed_f);
        error = SER_goto( Y_angle, Xi_angle, &axis_Y, &axis_Xi);
        if(error) return error;

        time_motion += last_elapsed();
        //Now we are at the right position now we take the measurement. Here is thermal.
        //no step measurement, slower, we clear any data left
        //twice, because we still have moving stuff left over. If we get only once we can clear measurement 1 but not measurement 2
        // this is because measurement 2 started while moving but is not done yet.
        //   ******||         moving                                ||***** stopped ******************************************************
        // measurement 1 xxxxxxxxxxxxxxxxxxxxx|| measurement 2 xxxxxxxxxxxxxxxxxxxxxxxxxxxx|| xxxxx correct measurement xxxxxxxxxxxxxxxxxx||
        //                                                          || Data1 || Wait data2 || start waiting for real measurement          ||
        // I have tried triggered measurement but it was not working
        printf("\tSensor Clear");
        for(int i_clear = 0; i_clear <2 ; i_clear++)
        {
            do
            { 
                sleep(SENSOR_DELTA_S/10);
                status =  MLX90640_FrameReady(sen_data.sen_addr);
            }
            while(!(status & 0x0008));
            MLX90640_ClearFrameAvailFlag(sen_data.sen_addr);
            sleep(SENSOR_DELTA_S/10);
            //MLX90640_GetFrameData(sen_data.sen_addr, sen_data.frame);
            //memset(&(sen_data.frame),0,sizeof(sen_data.frame));
            //sleep(0.25);
        }
        time_clear += last_elapsed();
        printf("ed ---  doing measurements   "); //clear....ed 
        //method 2 we store which one we got
        int frames_ready[2];
        frames_ready[0] = 0;
        frames_ready[1] = 0;

        for(int idxframe = 0; idxframe < 2 ; idxframe ++)
        {
            int ready;
            sleep(SENSOR_DELTA_S/2);
            do
            {
                ready = 0;
                //checking if frame is ready
                status =  MLX90640_FrameReady(sen_data.sen_addr);
                //We check if this frame is expected 
                ready = (status>0)&&(status & 0x0008) && !frames_ready[status & 0x0001];
                if(ready)
                {
                    ready = 1;
                    //We just got a frame, we add it
                    MLX90640_GetFrameData(sen_data.sen_addr, sen_data.frame);
                    if(frames_ready[sen_data.frame[833]])
                    {
                        printf(" repeat %d :( ", sen_data.frame[833]);
                        sleep(SENSOR_DELTA_S/2);
                        ready = 0;
                    }
                    else
                    {
                        printf(" frame %d ", sen_data.frame[833]);
                        frames_ready[sen_data.frame[833]] = 1;
                    }
                }
                else if((status>0)&&(status & 0x0008) && frames_ready[status & 0x0001])
                {
                    //discard
                    MLX90640_ClearFrameAvailFlag(sen_data.sen_addr);
                    printf(" x ");
                    sleep(SENSOR_DELTA_S/2);
                }
                else if(!(status & 0x0008))
                {
                    //waiting
                    //printf(".");
                    sleep(SENSOR_DELTA_S/10);
                }
                else
                {
                    printf("\n\t!!!!!!!!!!!!!!!!!!!! Error, reinint sensor and retry\n");
                    init_sensor(&sen_data, SENSOR_FPS);
                    //setting on demand measurement
                    //MLX90640_SetStepMode(sen_data.sen_addr, 1);
                    //MLX90640_StartMeasurement(sen_data.sen_addr);
                    sleep(SENSOR_DELTA_S);
                }
            }while(!ready);
            //get ambiant temperature
            sen_data.ta = MLX90640_GetTa(sen_data.frame, &sen_data.mlx90640_p);
            //Getting the image, if we do not need real temp or for averaging and compute temperature from RAW data
            MLX90640_CalculateTo(sen_data.frame, &sen_data.mlx90640_p, 1., 23.15, sen_data.mlx90640To);
            MLX90640_BadPixelsCorrection((&sen_data.mlx90640_p)->brokenPixels, sen_data.mlx90640To, sen_data.mode, &sen_data.mlx90640_p);
            MLX90640_BadPixelsCorrection((&sen_data.mlx90640_p)->outlierPixels, sen_data.mlx90640To, sen_data.mode, &sen_data.mlx90640_p);
        }

        //We got both frames
        //add the current sensor position to the image
        add_sen_to_tiff(Tiff_image,sen_data.mlx90640To, i_angle, list_n_angles);
        Ta_all[i_angle] = sen_data.ta; 
        time_measure += last_elapsed();
        //We write a partial file here
        make_tiff_metadata(tiff_descr_buff, list_n_angles,list_Y_angles,list_Xi_angles, Ta_all,  starttime, time_motion+time_clear+time_measure);
        write_tiff_descr(Tiff_image,SEN_NX*list_n_angles, SEN_NY, filename_partial, tiff_descr_buff);

    }
    printf("\n ======== All angles done  ==========\n ");
    printf("Writing TIFF file  \n");


    //we got all frames, we write the file
    //make a filename and write the log
    make_filename_log(filename, Ta_all, list_n_angles, IMAGES_FOLDER);//TODO: Do we need to store Ta for each angle ???
    //Filling the image description and write the Tiff image
    make_tiff_metadata(tiff_descr_buff, list_n_angles,list_Y_angles,list_Xi_angles, Ta_all,  starttime, time_motion+time_clear+time_measure);
    write_tiff_descr(Tiff_image,SEN_NX*list_n_angles, SEN_NY, filename, tiff_descr_buff);


    auto end_pair = std::chrono::system_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end_pair - start_pair);
    elapsed_f = float(elapsed.count())/1000000;
    printf("Total Elapsed %5.1fs\n",elapsed_f);
    printf("    motion\t %5.1f \t %2.1f%%\n", time_motion, time_motion*100/elapsed_f);
    printf("    clear\t %5.1f \t %2.1f%%\n", time_clear, time_clear*100/elapsed_f);
    printf("    measure\t %5.1f\t %2.1f%%\n", time_measure, time_measure*100/elapsed_f);


    printf("Done, returning home\n");
    SER_goto(list_Y_angles[0],-60, &axis_Y, &axis_Xi);

    SER_free(&axis_Y, &axis_Xi);


    //Deleting objects (yes I could switch to more modern stuff but this code is mostly C-like)
    delete [] Ta_all;
    delete [] Tiff_image;
    delete [] list_Y_angles;
    delete [] list_Xi_angles;


}




/* Code to write a new i2c address in the EEPROM, Use with caution
    The EEPROM supports only a limitted number of writes, read the datasheet !

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


//#include "headers/MLX90640_API.h"
#include "headers/MLX90640_I2C_Driver.h"
#include <stdio.h>
#include <iostream>
#include <cstring>
#include <fstream>
#include <chrono>
#include <thread>


#define MLX_I2C_ADDR 0x33
#define NEW_MLX_I2C_ADDR 0x38

//at least 5ms per the doc
#define DELAY 10000
  
int MLX90640_ShowEEE(uint8_t slaveAddr)
{
  uint16_t eeData[16];
  MLX90640_I2CRead(slaveAddr, 0x2400, 16, eeData);
  for(int i = 0; i< 16; i++)
    printf("Register 0x%04x  :  0x%04x\n",0x2400+i,eeData[i]);
    
}

//------------------------------------------------------------------------------

int MLX90640_SetEEAddr(uint8_t slaveAddr, uint8_t new_slaveAddr)
{
    uint16_t I2C_Address;
    uint16_t new_I2C_Address;
    int value;
    int error;

    I2C_Address = 0;
    error = MLX90640_I2CRead(slaveAddr, 0x240F, 1, &I2C_Address);
    if(error == 0)
    {
      printf("Register 0x240F  :  %04x\n",I2C_Address);
      new_I2C_Address = (I2C_Address & 0xFF00) | ( new_slaveAddr & 0xFF);
      new_I2C_Address = (0xbe00) | ( new_slaveAddr & 0xFF);

      printf("We will rewrite the EEPROM, be careful can only be done 10 times.\n");
      printf("Will erase Register 0x240F :  %04x\n",new_I2C_Address);
      error = MLX90640_I2CWrite(slaveAddr, 0x240F, 0x0000);
      std::this_thread::sleep_for(std::chrono::microseconds(DELAY));
      printf("Will write Register 0x240F :  %04x\n",new_I2C_Address);
      error = MLX90640_I2CWrite(slaveAddr, 0x240F, new_I2C_Address);
      std::this_thread::sleep_for(std::chrono::microseconds(DELAY));
    }
    else
      printf("!!!!! ERROR   :  %d\n",error);



    if(error == 0)
    {
      printf("Writing done, try to read back address ");
    }
    else
      printf("!!!!! ERROR   :  %d\n",error);


    I2C_Address = 0;
    error = MLX90640_I2CRead(slaveAddr, 0x240F, 1, &I2C_Address);
    if(error == 0)
    {
      printf("Register 0x240F  :  %04x\n",I2C_Address);
      std::this_thread::sleep_for(std::chrono::microseconds(DELAY));
      printf("Will write Register 0x8010 :  %04x\n",new_I2C_Address);
      error = MLX90640_I2CWrite(slaveAddr, 0x8010, new_I2C_Address);
    }
    else
      printf("!!!!! ERROR   :  %d\n",error);

    std::this_thread::sleep_for(std::chrono::microseconds(DELAY));

    I2C_Address = 0;
    error = MLX90640_I2CRead(new_slaveAddr, 0x8010, 1, &I2C_Address);
    if(error == 0)
    {
      printf("Register 0x810  :  %04x\n",I2C_Address);
    }
    else
      printf("!!!!! ERROR   :  %d\n",error);
    if(error == 0)
    {
      printf("The sensor should be at the new address\n");
    }
    else
      printf("!!!!! ERROR   :  %d\n",error);

    if(error == 0)
    {
      printf("Writing done, try to read back address ");
    }
    else
      printf("!!!!! ERROR   :  %d\n",error);


    std::this_thread::sleep_for(std::chrono::microseconds(DELAY));
    I2C_Address = 0;
    error = MLX90640_I2CRead(new_slaveAddr, 0x240F, 1, &I2C_Address);
    if(error == 0)
    {
      printf("Register 0x240F  :  %04x\n",I2C_Address);
    }
    else
      printf("!!!!! ERROR   :  %d\n",error);

    return error;
}

//------------------------------------------------------------------------------

int MLX90640_GetEEAddr(uint8_t slaveAddr)
{
    uint16_t I2C_Address;
    int value;
    int error;
        
    error = MLX90640_I2CRead(slaveAddr, 0x8010, 1, &I2C_Address);
    
    if(error == 0)
    {
      printf("Register 0x8010 & 0xFF :  %02x\n",I2C_Address & 0xFF);
      //value = (controlRegister1 | 0x1000);
        //error = MLX90640_I2CWrite(slaveAddr, 0x800D, value);        
    }    

    error = MLX90640_I2CRead(slaveAddr, 0x240F, 1, &I2C_Address);
    
    if(error == 0)
    {
      printf("Register 0x240F & 0xFF :  %02x\n",I2C_Address & 0xFF);
      //value = (controlRegister1 | 0x1000);
        //error = MLX90640_I2CWrite(slaveAddr, 0x800D, value);        
    }    

    return error;
}

//----------------------------------------------------------------------

int main(){

  MLX90640_ShowEEE(MLX_I2C_ADDR);
  printf("Ready to write from address 0x%2x to 0x%2x, are you sure ?\nDid you read the datasheet and the code ?\nStill time to hit Ctrl-C\nOtherwise press a key....",MLX_I2C_ADDR,NEW_MLX_I2C_ADDR);
  
  getchar();
  printf("OKAY fine !\n");
 
  MLX90640_GetEEAddr(MLX_I2C_ADDR);
  MLX90640_SetEEAddr(MLX_I2C_ADDR,NEW_MLX_I2C_ADDR);
  MLX90640_GetEEAddr(NEW_MLX_I2C_ADDR);
  MLX90640_ShowEEE(NEW_MLX_I2C_ADDR);
    return 0;
}

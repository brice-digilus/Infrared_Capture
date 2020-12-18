/** Header to specify the dynamixel model specific parameters and communication
 * 
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

#define SER_Z_ADDRESS  20
#define SER_Xi_ADDRESS 21
#define SER_DEVICENAME "/dev/ttyUSB0"
#define SER_BAUDRATE   1000000
// Control table address for MX-12W
#define SER_ADDR_TORQUE_ENABLE           24
#define SER_ADDR_GOAL_POSITION           30
#define SER_ADDR_PRESENT_POSITION        36
#define SER_ADDR_CW_LIM                  6
#define SER_ADDR_CCW_LIM                 8
#define SER_ADDR_TORQUE_LIM              34
#define SER_ADDR_MOVING_SPEED            32
#define SER_ADDR_LED                     25
#define SER_ADDR_D_GAIN                  26
#define SER_ADDR_I_GAIN                  27
#define SER_ADDR_P_GAIN                  28
#define SER_ADDR_IS_MOVING               46
#define SER_ADDR_PUNCH                   48
#define SER_ADDR_ACCELERATION            73

#define SER_SIZE_TORQUE_ENABLE           1
#define SER_SIZE_GOAL_POSITION           2
#define SER_SIZE_PRESENT_POSITION        2
#define SER_SIZE_CW_LIM                  2
#define SER_SIZE_CCW_LIM                 2
#define SER_SIZE_TORQUE_LIM              2
#define SER_SIZE_MOVING_SPEED            2
#define SER_SIZE_LED                     1
#define SER_SIZE_D_GAIN                  1
#define SER_SIZE_I_GAIN                  1
#define SER_SIZE_P_GAIN                  1
#define SER_SIZE_IS_MOVING               1
#define SER_SIZE_PUNCH                   2
#define SER_SIZE_ACCELERATION            1
// Protocol version
#define SER_PROTOCOL_VERSION             1.0                // See which protocol version is used in the Dynamixel
#define SER_MOVING_STATUS_THRESHOLD      10                 // Dynamixel moving status threshold

//Motion control constants
#define SER_VAL_TORQUE_LIM              250
#define SER_VAL_D_GAIN                  5
#define SER_VAL_I_GAIN                  12
#define SER_VAL_P_GAIN                  12
#define SER_VAL_MOVING_SPEED            80
#define SER_VAL_ACCELERATION            10
#define SER_VAL_PUNCH                   10


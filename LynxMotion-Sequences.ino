// LynxMotion_2xAL5D_Move_Sequences.ino
// Program for sending move sequences to two LynxMotion AL5D robot arms using
// BotBoarduino (Arduino Duemilanove) and a servo motor controller (SSC-32U)
//    Copyright (C) 2018  Toby Lightheart
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program.  If not, see <https://www.gnu.org/licenses/>.

// Based on https://github.com/Lynxmotion/BotBoarduino/tree/master/Examples/SSC-32U/Basic%20AL5D%20control
//  (v01.00 created by scharette, 2018-03-15)
//
// Last modified:  2018-06-20

// READ CODE COMMENTS FOR STEP INSTRUCTIONS TO CREATE MOTION SEQUENCES

#define ADD_SERIAL_SIZE 50
#define MS_IN_ONE_SECOND 1000
#define MS_IN_ONE_MINUTE 60000

// STEP 1: Connect the BotBoarduino and AL5D arms to the SSC-32U
// See guides: 
//    BotBoarduino http://www.lynxmotion.com/images/html/build185.htm
//    SSC-32U http://www.lynxmotion.com/images/data/lynxmotion_ssc-32u_usb_user_guide.pdf
// Recommended servo motor channel connections:
// Arm 0 connected servo motors:
//    #0 - Base
//    #1 - Shoulder
//    #2 - Elbow
//    #3 - Wrist Up/Down
//    #4 - Gripper Open/Close
//    #5 - Gripper Rotation
// Arm 1 connected servo motors:
//    #10 - Base
//    #11 - Shoulder
//    #12 - Elbow
//    #13 - Wrist Up/Down
//    #14 - Gripper Open/Close
//    #15 - Gripper Rotation

// STEP 2: Set initial servo arm positions
//    #0P1500  -> servo connected to channel #0 (arm 0 base) moves to
//      position for Pulse with 1500us duration (default servo middle position)
//    #11P1800 -> servo connected to channel #11 (arm 1 shoulder) moves to 
//      position for Pulse with 1800us duration
// NOTE: controller channels are used for control are different from recommended, 
//    these channel numbers will need to be adjusted accordingly 
#define ARM0_INIT_POSITION "#0P1500 #1P1500 #2P1500 #3P1500 #4P1500 #5P1500"
#define ARM1_INIT_POSITION "#10P1500 #11P1500 #12P1500 #13P1500 #14P1500 #15P1500"
#define ARM0_TORQUE_OFF "#0P0 #1P0 #2P0 #3P0 #4P0 #5P0"
#define ARM1_TORQUE_OFF "#10P0 #11P0 #12P0 #13P0 #14P0 #15P0"

// STEP 3: Create a list of position commands to send to the arm servo motors
//    An example is given: servo motors can be moved one at a time (e.g., moves 0 through 5)
//    servo motors can also be moved as a group by providing multiple positions at once
//    (e.g., move 7 and the ARM0_INIT_POSITION strings) 
// Position commands for Arm 0 go in this variable:
const char* seq_arm0_positions[] = { "#0P1800",  // sequence 0, move 0 - base
                                     "#1P1800",  // move 1 - shoulder
                                     "#2P1800",  // move 2 - elbow
                                     "#3P1800",  // move 3 - wrist up
                                     "#4P1800",  // move 4 - gripper close
                                     "#5P1800",  // move 5 - gripper rotate
                                     ARM0_INIT_POSITION, // move 6
                                     "#0P1800 #1P1800 #2P1800 #3P1800 #4P1800 #5P1800",  // sequence 1, move 7
                                     ARM0_INIT_POSITION
                                   }; // move 8

// Position commands for Arm 1 go in this variable:
const char* seq_arm1_positions[] = { "#10P1800",  // move 0 - base
                                     "#11P1800",  // move 1 - shoulder
                                     "#12P1800",  // move 2 - elbow
                                     "#13P1800",  // move 3 - wrist up
                                     "#14P1800",  // move 4 - gripper close
                                     "#15P1800",  // move 5 - gripper rotate
                                     ARM1_INIT_POSITION, // move 6
                                     "#10P1800 #11P1800 #12P1800 #13P1800 #14P1800 #15P1800",  // sequence 1, move 7
                                     ARM1_INIT_POSITION
                                   }; // move 8


// STEP 4: Set the number of move sequences and what moves in the list are in each sequence
//    This program will go through each sequence from start to finish.
// WARNING: Counting incorrectly could cause unpredictable operation (end zeros have been added to seq_end for some protection)
const byte seq_end = 1;                // number of move sequences (counting starts at 0)
const byte move_start[] = {0, 7, 0};   // sequence 0 starts at 0, sequence 1 starts at 7
const byte move_end[] = {6, 8, 0};     // sequence 0 ends at move 6, sequence 1 ends at move 8

// STEP 5: Set the move durations and pauses durations between moves
//    Servo move speed is controlled to take a given time. 
//    Commands are given in millisecond (max is 65536ms). The example has each move take 4000ms. 
//    (A 'T' is added to each command sent to the servo controller later in the code.) 
// Move duration times go in this variable:
const unsigned int seq_move_durations[] = { 4000, // move 0 (Numbers for move T commands)
                                            4000, // move 1
                                            4000, // move 2
                                            4000, // move 3
                                            4000, // move 4
                                            4000, // move 5
                                            4000, // move 6
                                            4000, // move 7
                                            4000  // move 8
                                          }; 

// Pause durations go in this variable:
const unsigned int seq_pause_durations[] = { 2000, // move 0 (Numbers provided to delay function)
                                             2000, // move 1
                                             2000, // move 2
                                             2000, // move 3
                                             2000, // move 4
                                             2000, // move 5
                                             1000, // move 6
                                             2000, // move 7
                                             1000  // move 8
                                           }; 

// STEP 6: Set motor cool-down times 
//    Guides recommend servo motors be powered down to cool: operate 1 : cool 3
//    Total cool-down is the number of minutes and seconds
#define ARM_TORQUE_OFF_MINUTES 1  
#define ARM_TORQUE_OFF_SECONDS 10    


// STEP 7: Run this program
//    The remaining code should require no changes to operate the AL5D arms from 
//    the commands provided.


// Init software serial interface to SSC-32U
// Adapted from basic AL5D control with BotBoarduino by scharette (v01.00, 2018-03-15)
#include <SoftwareSerial.h>

SoftwareSerial ssc32u(12, 13);

void setup() {
  // Init serial on USB port (UNO, duemilanove, etc.)
  Serial.begin(115200);
  Serial.println("Starting initialisation >");

  // Init software serial (RX/TX pins are 12/13)
  Serial.println("Software serial interface initialisation >");
  Serial.println("\tssc32u @9600");
  ssc32u.begin(9600);
  ssc32u.listen();

  // Move motors to centered position in a group move
  Serial.println("Set motors to initial positions >");
  Serial.print("\tArm 0: ");
  Serial.println(ARM0_INIT_POSITION);
  Serial.print("\tArm 1: ");
  Serial.println(ARM1_INIT_POSITION);

  ssc32u.write(ARM0_INIT_POSITION);
  ssc32u.write(ARM1_INIT_POSITION);
  ssc32u.write("\r");
  // /r       -> carriage return character necessary to execute move command

  Serial.println("Initialisation completed.");
  delay(MS_IN_ONE_SECOND);
}

byte seq_i = 0;           // sequence index (counter)
byte move_i = 0;          // move index (counter)
byte move_i_temp, seq_i_temp;
unsigned int off_i = 0;   // off cool-down counter
char add_serial[ADD_SERIAL_SIZE];

// Main code loop
void loop() {
  // Loop arm movement sequences
  for (seq_i = 0 ; seq_i <= seq_end; seq_i++)
  {
    snprintf(add_serial, ADD_SERIAL_SIZE, "Starting movement sequence %d >", seq_i);
    Serial.println(add_serial);

    for (move_i = move_start[seq_i]; move_i <= move_end[seq_i]; move_i++)
    {
      // Iterate through moves of the sequence
      // Send move commands to SSC-32U
      ssc32u.write(seq_arm0_positions[move_i]);
      ssc32u.write(seq_arm1_positions[move_i]);
      snprintf(add_serial, ADD_SERIAL_SIZE, "T%d \r", seq_move_durations[move_i]);
      ssc32u.write(add_serial);

      // Repeat command to serial for computer monitoring
      seq_i_temp = seq_i;
      move_i_temp = move_i;
      snprintf(add_serial, ADD_SERIAL_SIZE, "Sequence %d > Move Index %d >", seq_i_temp, move_i_temp);
      Serial.println(add_serial);
      Serial.println(seq_arm0_positions[move_i]);
      Serial.println(seq_arm1_positions[move_i]);
      snprintf(add_serial, ADD_SERIAL_SIZE, "T%d \r", seq_move_durations[move_i]);
      Serial.println(add_serial);

      delay(seq_move_durations[move_i]);
      delay(seq_pause_durations[move_i]);
    }

    // Power down arms servos to allow motors to cool between sequences
    ssc32u.write(ARM0_TORQUE_OFF);  // power off servos to cool
    ssc32u.write(ARM1_TORQUE_OFF);  // power off servos to cool
    ssc32u.write("\r");

    snprintf(add_serial, ADD_SERIAL_SIZE, "Servo cooling. Torque off for %d minutes %d seconds.", 
        ARM_TORQUE_OFF_MINUTES, ARM_TORQUE_OFF_SECONDS);
    Serial.println(add_serial);
    Serial.println(ARM0_TORQUE_OFF);  // power off servos to cool
    Serial.println(ARM1_TORQUE_OFF);  // power off servos to cool

    for(off_i = 0; off_i < ARM_TORQUE_OFF_MINUTES; off_i++)
    {
      delay(MS_IN_ONE_MINUTE);
    }
    for (off_i = 0; off_i < ARM_TORQUE_OFF_SECONDS; off_i++)
    {
      delay(MS_IN_ONE_SECOND);
    }

    seq_i_temp = seq_i;
    move_i_temp = move_end[seq_i];
    snprintf(add_serial, ADD_SERIAL_SIZE, "Sequence %d > Move End Index %d >", seq_i_temp, move_i_temp);
    Serial.println(add_serial);
    Serial.println(seq_arm0_positions[move_end[seq_i]]);
    Serial.println(seq_arm1_positions[move_end[seq_i]]);
    Serial.println("Servo motor torque on.");
    
    // Restart servos in the last position
    ssc32u.write(seq_arm0_positions[move_end[seq_i]]);
    ssc32u.write(seq_arm1_positions[move_end[seq_i]]);
    ssc32u.write("\r");
    delay(MS_IN_ONE_SECOND);
  }
}

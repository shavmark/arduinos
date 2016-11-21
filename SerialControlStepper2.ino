/**
 * \par Copyright (C), 2012-2016, MakeBlock
 * @file    SerialControlStepper.ino
 * @author  MakeBlock
 * @version V1.0.0
 * @date    2015/11/19
 * @brief   Description: this file is sample code for Stepper Driver device.
 *
 * Function List:
 *
 *    1. void MeStepper::moveTo(long absolute); 
 *    2. void MeStepper::move(long relative);
 *    3. boolean MeStepper::run();
 *    4. void MeStepper::setMaxSpeed(float speed);
 *    5. void MeStepper::setAcceleration(float acceleration);
 *
 * \par History:
 * <pre>
 * <Author>     <Time>        <Version>      <Descr>
 * Mark Yan     2015/09/07    1.0.0          rebuild the old lib.
 * forfish      2015/11/19    1.0.0          add some descriptions.
 * </pre>
 */

#include "MeOrion.h"
#include <SoftwareSerial.h>

/* commands
      1. void MeStepper::setpin(uint8_t dir_data, uint8_t step_data);
 *    2. void MeStepper::moveTo(long absolute); 
 *    3. void MeStepper::move(long relative);
 *    4. boolean MeStepper::run();
 *    5. boolean MeStepper::runSpeed();
 *    6. void MeStepper::setMaxSpeed(float speed);
 *    7. void MeStepper::setAcceleration(float acceleration);
 *    8. void MeStepper::setSpeed(float speed);
 *    9. float MeStepper::speed();
 *    10. long MeStepper::distanceToGo();
 *    11. long MeStepper::targetPosition();
 *    12. long MeStepper::currentPosition();  
 *    13. void MeStepper::setCurrentPosition(long position);  
 *    14. void MeStepper::runToPosition();
 *    15. boolean MeStepper::runSpeedToPosition();
 *    16. void MeStepper::runToNewPosition(long position);
 *    17. void MeStepper::disableOutputs();
 *    18. void MeStepper::enableOutputs();
 */


class XYArm
{    
  public:
    enum {SetPin, MoveTo, Move, Run, RunSpeed, SetMaxSpeed, SetAcceleration, SetSpeed, SetCurrentPosition, RunToPosition, RunSpeedToPosition, DisableOutputs, EnableOutputs, GetDistanceToGo, GetTargetPositon, GetCurrentPosition, } Commands;

    
    void begin(int baud, MeStepper& stepper1, MeStepper& stepper2);
    
    int read(MeStepper& stepper1, MeStepper& stepper2);         // must be called regularly to clean out Serial buffer
          
  private:
    unsigned char cmd;
    // internal variables used for reading messages
    int checksum;
    unsigned char status; 
    unsigned char vals[16];  // temporary values, moved after we confirm checksum
    int index;              // -1 = waiting for new packet
};

XYArm xy;
MeStepper stepper1(PORT_1);
MeStepper stepper2(PORT_2);

void setup()
{  
  xy.begin(9600, stepper1, stepper2);
}

void loop()
{
  if(Serial.available())
  {
    char a = Serial.read();
    switch(a)
    {
      case '0':
      stepper1.setSpeed(100.0f);
      stepper1.moveTo(10);
      break;
      case '1':
      stepper1.moveTo(200);
      break;
      case '2':
      stepper1.move(50);
      break;
      case '3':
      stepper1.move(100);
      break;
      case '4':
      stepper1.move(200);
      break;
      case '5':
      stepper1.move(400);
      break;
      case '6':
      stepper1.move(600);
      break;
      case '7':
      stepper1.move(4000);
      break;
      case '8':
      stepper1.move(8000);
      break;
      case '9':
      stepper1.move(3200);
      break;
    }
  }
      if (!stepper1.run()){
        Serial.println("bad!");
      }
}

 void XYArm::begin(int baud, MeStepper& stepper1, MeStepper& stepper2){
      status = 0; 
      index = -1;
      // Change these to suit your stepper if you want
      stepper1.setMaxSpeed(1000);
      stepper1.setAcceleration(20000);
      stepper2.setMaxSpeed(1000);
      stepper2.setAcceleration(20000);
      Serial.begin(baud);
    }

int XYArm::read(MeStepper& stepper1, MeStepper& stepper2){

   while(Serial.available() > 0){


        if(index == -1){         // looking for new packet
            if(Serial.read() == 0xff){
                index = 0;
                checksum = 0;
            }
        }else if(index == 0){
            vals[index] = (unsigned char) Serial.read();
            if(vals[index] != 0xff){            
                checksum += (int) vals[index];
                index++;
            }
        }else{
            vals[index] = (unsigned char) Serial.read();
            checksum += (int) vals[index];
            index++;
            if(index == 16){ // packet complete
                if(checksum%256 != 255){
                    // packet error!
                    index = -1;

                //flush serial
                while(Serial.available())
                {
                  Serial.read();
                }
                    return 0;
                }else{
                  /*
                    Xaxis = ((vals[0]<<8) + vals[1]);
                    Yaxis = (vals[2]<<8) + vals[3];
                    Zaxis = (vals[4]<<8) + vals[5];
                    W_ang = ((vals[6]<<8) + vals[7]);
                    W_rot = (vals[8]<<8) + vals[9];
                    Grip = (vals[10]<<8) + vals[11];
                    dtime = vals[12];
                    buttons = vals[13];
                    ext = vals[14];
                    */
                }
                index = -1;
                //flush serial
                while(Serial.available())
                {
                  Serial.read();
                }
               // Serial.flush();


                return 1;
            }
        }
    }
    return 0;
}

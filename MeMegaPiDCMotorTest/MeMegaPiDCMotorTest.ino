/**
 * \par Copyright (C), 2012-2016, MakeBlock
 * @file    MeMegaPiDCMotorTest.ino
 * @author  MakeBlock
 * @version V1.0.0
 * @date    2016/05/17
 * @brief   Description: this file is sample code for MegaPi DC motor device.
 *
 * Function List:
 *    1. void MeMegaPiDCMotorTest::run(int16_t speed)
 *    2. void MeMegaPiDCMotorTest::stop(void)
 *
 * \par History:
 * <pre>
 * <Author>     <Time>        <Version>      <Descr>
 * Mark Yan     2016/05/17    1.0.0          build the new
 * </pre>
 */

//PORT1B == Right wheel
//PORT2B == Left wheel

#include "MeMegaPi.h"

MeMegaPiDCMotor motor1(PORT1B);

MeMegaPiDCMotor motor2(PORT2B);

MeMegaPiDCMotor motor3(PORT3B);

MeMegaPiDCMotor motor4(PORT4B);

uint8_t motorSpeed = 100;

void setup()
{
}
   
void loop()
{
  //motor1.run(motorSpeed); /* value: between -255 and 255. */
  //motor2.run(motorSpeed); /* value: between -255 and 255. */
  motor1.run(-motorSpeed);
  motor2.run(motorSpeed);
  delay(1000);
  motor1.stop();
  motor2.stop();
  delay(1000);

}


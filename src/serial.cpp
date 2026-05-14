#include <Arduino.h>
#include "serial.h"
#include "imu.h"
#include "motor.h"
#include "legs.h"
#include "ctrl.h"
#include "adc.h"
#include "pid.h"
//串口定时发送任务(调试用)
void Serial_Task(void *pvParameters)
{
	while (1)
	{
		//Serial.printf("%f\r\n",leftLegPos.length);
		//Serial.printf("%f\r\n",rightLegPos.length);
		float lTh = leftLegPos.angle - imuData.pitch - M_PI_2;
		float rTh = rightLegPos.angle - imuData.pitch - M_PI_2;
		int prot = (lTh < -M_PI_4 || lTh > M_PI_4 || rTh < -M_PI_4 || rTh > M_PI_4 || imuData.pitch > M_PI_4 || imuData.pitch < -M_PI_4);
		Serial.printf("lTh:%.2f rTh:%.2f prot:%d j:[%.1f,%.1f,%.1f,%.1f]\n",
				lTh, rTh, prot,
				leftJoint[0].angle, leftJoint[1].angle, rightJoint[0].angle, rightJoint[1].angle);	//调试

		//Serial.printf("%.3f,%.3f\n",leftJoint[0].angle,leftJoint[1].angle);
		//Serial.printf("%.3f,%.3f\n",rightJoint[0].angle,rightJoint[1].angle);
		//Serial.printf("%.3f\n",leftWheel.angle);
		//Serial.printf("%.3f\n",rightWheel.angle);

		//Serial.printf("%.3f,%.3f\r\n",leftJoint[0].voltage,leftJoint[1].voltage);
		//Serial.printf("%.3f,%.3f\r\n",rightJoint[0].voltage,rightJoint[1].voltage);
		//Serial.printf("%.3f,%.3f\r\n",leftWheel.voltage,rightWheel.voltage);


		// Serial.printf("%f,%f,%f,%f,%f,%f\r\n",stateVar.theta,stateVar.dTheta,stateVar.x,stateVar.dx,stateVar.phi,stateVar.dPhi);

		//Serial.printf("%.3f,%.3f,%.3f\r\n",target.yawAngle, imuData.yaw,yawPID.output);
		//Serial.printf("%.3f,%.3f\n",vot,motorOutRatio);
		//Serial.printf("%.3f\n",leftWheel.angle);	//测量2805反电动势
		//Serial.printf("%f\n",leftJoint[0].speed);	//测量4310反电动势
		//Serial.printf("%f,%f,%f\n",imuData.pitchSpd,imuData.rollSpd,imuData.yawSpd); 	//查看角速度 确保IMU方向
		//Serial.printf("%f,%f,%f\n",imuData.pitch,imuData.roll,imuData.yaw); 	//查看角速度 确保IMU方向

		//Serial.printf("Hello\r\n");
		vTaskDelay(50);
	}
}

//串口模块初始化
void Serial_Init(void)
{
    Serial.begin(115200);
    Serial.setTimeout(10);
	xTaskCreate(Serial_Task, "Serial_Task", 4096, NULL, 1, NULL);
}

diff --git a/HEAD:test.c b/HEAD:code.c
index 5a36d5a..8f1b6a3 100644
--- a/HEAD:test.c
+++ b/HEAD:code.c
@@ -1,13 +1,13 @@
 #pragma config(Sensor, in1,    clawLPot,       sensorPotentiometer)
 #pragma config(Sensor, in2,    clawRPot,       sensorPotentiometer)
 #pragma config(Sensor, in3,    armPot,         sensorPotentiometer)
+#pragma config(Sensor, in4,    front,          sensorLineFollower)
+#pragma config(Sensor, in5,    middle,         sensorLineFollower)
+#pragma config(Sensor, in6,    leftback,       sensorLineFollower)
+#pragma config(Sensor, in7,    centerback,     sensorLineFollower)
+#pragma config(Sensor, in8,    rightback,      sensorLineFollower)
 #pragma config(Sensor, dgtl9,  encoderR,       sensorQuadEncoder)
 #pragma config(Sensor, dgtl11, encoderL,       sensorQuadEncoder)
-#pragma config(Sensor, in5,    middle,         sensorReflection)
-#pragma config(Sensor, in7,    centerback,     sensorReflection)
-#pragma config(Sensor, in6,    leftback,       sensorReflection)
-#pragma config(Sensor, in8,    righback,       sensorReflection)
-#pragma config(Sensor, in4,    front,      sensorReflection)
 #pragma config(Motor,  port1,           clawR,         tmotorVex393_HBridge, openLoop)
 #pragma config(Motor,  port2,           lift4,         tmotorVex393_MC29, openLoop)
 #pragma config(Motor,  port3,           lift3,         tmotorVex393_MC29, openLoop, reversed)
@@ -37,9 +37,9 @@ void SetArm(int power);
 // Pot calibration
 
 //Variables
-	//Claw variables
-int ClawLCalibrate = 0; //97 - calibrated value. difference from full closed value: (touching motor)
-int ClawRCalibrate = 0; //4060 - calibrated value. difference form full closed value: + means bigger number
+//Claw variables
+int ClawLCalibrate = 97 - 90; //97 - calibrated value. difference from full closed value: (touching motor)
+int ClawRCalibrate = 40; //4060 - calibrated value. difference form full closed value: + means bigger number
 int ClawState = 9; 						// 0 - open, 1 - semi-closed, 2 - closed
 int CloseLAngle = 820+ClawLCalibrate; 				//the angle the claw is closed
 int CloseRAngle = 3100+ClawRCalibrate; 				//the angle the claw is closed
@@ -52,26 +52,26 @@ int ClawRFullOpen = 1600+ClawRCalibrate;
 int CustomAngleL = 0+ClawLCalibrate; //If we want different/custom angles
 int CustomAngleR = 0+ClawRCalibrate;
 
-		//PID/stalls
+//PID/stalls
 int ClosePower = 70;//127 				// the test value for stall detect
 int ClawStallPower = 70;//60 			//power after stall
 int ClawStallTime = 1500;//1000 			//if the claw is not moving for this time, go to low power
-float ClawP = .2;//.1? 	//P value for claw PID
+float ClawP = .4;//.1? 	//P value for claw PID
 float ClawD = .001;//.001 	// D value for PID
 float ClawI = .005; //.015
-float ClawFineP = .4;//.2 //when error gets close to 0, this takes over
+float ClawFineP = 1;//.2 //when error gets close to 0, this takes over
 int 	ClawFinePLimit = 10; //15 // this is the max power Fine motor can provide
 float ClawFineOffset = (ClawFineP==0?0:ClawFinePLimit-(ClawFinePLimit/ClawFineP*ClawP));
 int 	PILimit = 65;//60 // the limit of the total power P+I can provide
 
-	//Arm Variables##############################################################
+//Arm Variables##############################################################
 int ArmState = 9; 		// disabled
 int ArmCalibrate = 0; //2810 is zero position
-float ArmP = .8;		//.1? 	//P value for claw PID
-float ArmD = .002;	//.006 	// D value for PID
+float ArmP = .12;		//.1? 	//P value for claw PID
+float ArmD = .000;	//.006 	// D value for PID
 float ArmI = .0025; 	//.0025
-float ArmFineP = .2;	//.08 //when error gets close to 0, this takes over
-int ArmFinePLimit = 8; //12 // this is the max power Fine motor can provide
+float ArmFineP = .1;	//.08 //when error gets close to 0, this takes over
+int ArmFinePLimit = 75; //12 // this is the max power Fine motor can provide
 float ArmFineOffset = (ArmFineP==0?0:ArmFinePLimit-(ArmFinePLimit/ArmFineP*ArmP));
 int ArmPILimit = 45;	//60 // the limit of the total power P+I can provide
 int ArmStallPower = 50; //60
@@ -82,31 +82,73 @@ int FireExpireTime = 1500; //1500 if it is firing for longer than this amount of
 int FireOverdriveTime = 20; // 60number of milliseconds it will keep driving arm after it reached fire angle
 bool ArmFired = false;
 // arm angles
-int CustomAngle 	= 1400; 	//Custom arm angle
+int CustomAngle 	= 2770; 	//Custom arm angle
 int ArmDownAngle	= 960+ArmCalibrate;//intake angle
 int ArmPrimeAngle = 1400+ArmCalibrate; //1070 just above the ground
 int ArmHighAngle 	= 1690+ArmCalibrate;//just a fun angle for w/e
 int ArmTopAngle 	= 2940+ArmCalibrate;//after firing it will go here
 
 //functions
+
+//function to make code go a certain distance
+void driveDistance(int target, int power){
+	//settting sensors to zero
+	SensorValue[encoderR] = 0;
+	SensorValue[encoderL] = 0;
+
+
+	//the right encoder will be used to measure distance
+
+	int senL =  SensorValue[encoderL];
+	int senR =  SensorValue[encoderR];
+	int sensor = SensorValue[encoderR];
+	while(sensor < target){
+		motor[baseL1]=power;
+		motor[baseL2]=power;
+		motor[baseR1]=power;
+		motor[baseR2]=power;
+
+		if(senL > senR)
+		{
+			SensorValue[encoderR] = senL;
+			sensor = senL;
+		}
+		if(senL < senR)
+		{
+			SensorValue[encoderL] = senR;
+			sensor = senR;
+		}
+		if(senL == senR)
+		{
+			sensor = senR;
+		}
+
+	}
+	motor[baseL1]=0;
+	motor[baseL2]=0;
+	motor[baseR1]=0;
+	motor[baseR2]=0;
+}
+
+
 void pre_auton()
 {
-  bStopTasksBetweenModes = true;
+	bStopTasksBetweenModes = true;
 }
 void SetBase(int powerL, int powerR)
 {
-		motor[baseL1]=powerL;
-		motor[baseL2]=powerL;
-		motor[baseR1]=powerR;
-		motor[baseR2]=powerR;
-		return;
+	motor[baseL1]=powerL;
+	motor[baseL2]=powerL;
+	motor[baseR1]=powerR;
+	motor[baseR2]=powerR;
+	return;
 }
 void SetArm(int power)
 {
-	  motor[lift1]=power;
-	  motor[lift2]=power;
-	  motor[lift3]=power;
-	  motor[lift4]=power;
+	motor[lift1]=power;
+	motor[lift2]=power;
+	motor[lift3]=power;
+	motor[lift4]=power;
 }
 void SetClawL(int powerL)
 {
@@ -165,34 +207,34 @@ task clawTask() //this just manages the PID on the robot
 		{
 			switch (ClawState) //sets targets
 			{
-				case 1:
-					clawLTarget = CustomAngleL;
-					clawRTarget = CustomAngleR;
-					enablePID = true;
-					break;
-				case 2:
-					clawLTarget = CloseLAngle;
-					clawRTarget = CloseRAngle;
-					enablePID = true;
-					break;
-				case 3:
-					clawLTarget = ClawLSemicloseAngle;
-					clawRTarget = ClawRSemicloseAngle;
-					enablePID = true;
-					break;
-				case 4:
-					clawLTarget = ClawLOpen;
-					clawRTarget = ClawROpen;
-					enablePID = true;
-					break;
-				case 5:
-					clawLTarget = ClawLFullOpen;
-					clawRTarget = ClawRFullOpen;
-					enablePID = true;
-					break;
-				default:
-					enablePID = false;
-					break;
+			case 1:
+				clawLTarget = CustomAngleL;
+				clawRTarget = CustomAngleR;
+				enablePID = true;
+				break;
+			case 2:
+				clawLTarget = CloseLAngle;
+				clawRTarget = CloseRAngle;
+				enablePID = true;
+				break;
+			case 3:
+				clawLTarget = ClawLSemicloseAngle;
+				clawRTarget = ClawRSemicloseAngle;
+				enablePID = true;
+				break;
+			case 4:
+				clawLTarget = ClawLOpen;
+				clawRTarget = ClawROpen;
+				enablePID = true;
+				break;
+			case 5:
+				clawLTarget = ClawLFullOpen;
+				clawRTarget = ClawRFullOpen;
+				enablePID = true;
+				break;
+			default:
+				enablePID = false;
+				break;
 			}
 			ClawState = 0;
 			lastErrorL = errorL;
@@ -274,7 +316,7 @@ task clawTask() //this just manages the PID on the robot
 			stallTimeR = 0;
 		}
 
-//Setting motor power ------------------------------------------------
+		//Setting motor power ------------------------------------------------
 		if(!enablePID)//If PID disbaled
 		{
 			SetClawL(0);
@@ -348,35 +390,35 @@ task armTask()//################################################################
 		{
 			switch (armState) //sets targets
 			{
-				case 1: //custom angle
-					armTarget = CustomAngle;
-					fire = false;
-					enablePID = true;
-					break;
-				case 2:
-					armTarget = ArmDownAngle;
-					fire = false;
-					enablePID = true;
-					break;
-				case 3:
-					armTarget = ArmPrimeAngle;
-					fire = false;
-					enablePID = true;
-					break;
-				case 4:
-					armTarget = ArmHighAngle;
-					fire = false;
-					enablePID = true;
-					break;
-				case 5:
-					armTarget = ArmTopAngle;
-					enablePID = true;
-					fire = true;
-					break;
-				default:
-					enablePID = false;
-					fire = false;
-					break;
+			case 1: //custom angle
+				armTarget = CustomAngle;
+				fire = false;
+				enablePID = true;
+				break;
+			case 2:
+				armTarget = ArmDownAngle;
+				fire = false;
+				enablePID = true;
+				break;
+			case 3:
+				armTarget = ArmPrimeAngle;
+				fire = false;
+				enablePID = true;
+				break;
+			case 4:
+				armTarget = ArmHighAngle;
+				fire = false;
+				enablePID = true;
+				break;
+			case 5:
+				armTarget = ArmTopAngle;
+				enablePID = true;
+				fire = true;
+				break;
+			default:
+				enablePID = false;
+				fire = false;
+				break;
 			}
 			stallTime = 0; //time before stall is true
 			stalled = false; //limits motor power
@@ -394,7 +436,7 @@ task armTask()//################################################################
 		error = armTarget - armPotentiometer;
 		dError = (error - lastError)*(dTime>0?1000/dTime:0);
 
-//PID addition part
+		//PID addition part
 		if ((abs(accError)>abs(accError+error)) ) //if acc error will be reduced
 		{
 			accError+= error;
@@ -404,7 +446,7 @@ task armTask()//################################################################
 			accError+=error;
 		}
 		armPower = (abs(error*ArmFineP)<ArmFinePLimit? error*ArmFineP:(error*ArmP+(error>0?ArmFineOffset:-ArmFineOffset))) + dErrorAvg*ArmD + accError*ArmI; //arm power according to the PIDs
-//Stall Check
+		//Stall Check
 		if(abs(armPower)> ArmStallDetect)//checking whether motors are stalling
 		{
 			stallTime+=dTime;
@@ -418,7 +460,7 @@ task armTask()//################################################################
 			stalled = false;
 			stallTime = 0;
 		}
-//Fire stuffs
+		//Fire stuffs
 		if (fire && fireTargetTime == 0)
 		{
 			fireTargetTime = currentTime+FireExpireTime;
@@ -509,119 +551,337 @@ task driveManager()
 	}
 }
 
-task autonomous()
-{
-  //open claw
+/*----------------To do list------------------*/
+/*calculate distance to fence so we can go back the exact same distance
+fix the drive distance functionn
+fix the parameters in the drive distance function*/
 
-	motor[clawL]=motor[clawR]= 100;
-	wait1Msec(200);
-	motor[clawL]=motor[clawR]=0;
 
 
 
-	//drive forward until line
-	if(SensorValue[middle]<100){
 
-		motor[baseR1]=motor[baseR2]= 127;
-		motor[baseL1]=motor[baseL2]= 127;
-		//motor[left1]=motor[left2]=motor[left3]= 127;
-		//motor[right1]=motor[right2]=motor[right3]= 127;
-	}
 
-	else{
 
-		motor[baseR1]=motor[baseR2]= 0;
-		motor[baseL1]=motor[baseL2]= 0;
-		//motor[left1]=motor[left2]=motor[left3]= 0;
-		//motor[right1]=motor[right2]=motor[right3]= 0;
-	}
+//
 
-	//zero encoders
-	nMotorEncoder(left)=nMotorEncoder(right)=0;
+task autonomous(){
 
-	//turn until front and centerback light sensors are aligned
-	if(SensorValue[centerback] && SensorValue[front] < 100){
+	clearDebugStream();
+	datalogClear();
+	clearTimer(T1);
+	startTask(clawTask);
+	wait1Msec(5);
+	startTask(armTask);
 
-		motor[baseR1]=motor[baseR2]= -100;
-		motor[baseR1]=motor[baseR2]= 100;
+	////claw opening
 
-		//motor[left1]=motor[left2]=motor[left3]= -100;
-		//motor[right1]=motor[right2]=motor[right3]= 100;
-	}
+	ClawState = 5;
+	while(!((SensorValue[clawLPot] < (ClawLFullOpen + 100) && SensorValue[clawLPot] > (ClawLFullOpen - 100)) &&
+		(SensorValue[clawRPot] < (ClawRFullOpen + 100) && SensorValue[clawRPot] > (ClawRFullOpen - 100)))){} // Wait until state
 
-	else{
+	ArmState = 1;  //raise arms up
+	while(!(SensorValue[armPot] < (CustomAngle + 100) && SensorValue[armPot] > (CustomAngle - 100))){} // Wait until state
 
-		motor[baseR1]=motor[baseR2]= 0;
-		motor[baseR1]=motor[baseR2]= 0;
+	ArmState = 9;
 
-		//motor[left1]=motor[left2]=motor[left3]= 0;
-		//motor[right1]=motor[right2]=motor[right3]= 0;
+	////while(SensorValue[middle] > 400 /*adjust value*/){
+	//	SetBase(20,20);
+
+	//	if(SensorValue[middle] < 400)
+	//	{
+	//		SetBase(0,0);
+	//		break;
+	//	}
+	//}
+
+	wait1Msec(100);
+	SensorValue[encoderL] = SensorValue[encoderR] = 0;
+
+	//move foward
+	int firstdrive = 500;
+	int encR = 600;
+
+	while(((Sensorvalue[encoderR] + Sensorvalue[encoderL]) /2 )< firstdrive){
+		SetBase(70,70);
 	}
 
-	turndist = (SensorValue(left)+SensorValue(right))/2;
 
-	//drive towards cube
-	motor[baseR1]=motor[baseR2]= 127;
-	motor[baseL1]=motor[baseL2]= 127;
-	wait1Msec(200);
-	motor[baseR1]=motor[baseR2]= 0;
-	motor[baseL1]=motor[baseL2]= 0;
 
-	//close claw
-	motor[claw1]=motor[claw1]= -100;
+	//while(Sensorvalue[encoderR] < -400 )
+	//	{
+
+	//	}
+
+
+
 	wait1Msec(200);
-	motor[claw1]=motor[claw1]=0;
+	SensorValue[encoderL] = SensorValue[encoderR] = 0;
 
-	// Lift Claw
-	wait1Msec(100);
-	int sensorPot;
-
-	//setting pot value to 0
-	SensorValue[armPot] = 0;
-	senorPot = SensorValue[armPot];
-
-	//claw lifts until arm is high enough that it doesn't hit stars when it turns
-	while( sensorPot < 2000){
-      motor[lift1] = 100;
-	  motor[lift2] = 100;
-	  motor[lift3] = 100;
-	  motor[lift4] = 100;
-	  sensorPot = SensorValue[armPot];
+	//turn horizintal
+	while(Sensorvalue[encoderR] < 470){
+		SetBase(-40,40);
+
+  }
+
+  SensorValue[encoderL] = SensorValue[encoderR] = 0;
+  while(((Sensorvalue[encoderR] + Sensorvalue[encoderL])*0.5) < encR){
+		SetBase(100,100);
 	}
 
-	 motor[lift1] =  motor[lift2]=  motor[lift3] = motor[lift4] = 0;
+	SensorValue[encoderL] = SensorValue[encoderR] = 0;
+	while(Sensorvalue[encoderR] < 470){
+		SetBase(-40,40);
 
+  }
 
 
-	//turn 90 degrees so rear faces wall
-	while (nMotorEncoder[left] < turndist)
-	{
- 		motor[baseR1]=motor[baseR2]= -100;
-		motor[baseR1]=motor[baseR2]= 100;
-	}
-		motor[baseR1]=motor[baseR2]= 0;
-		motor[baseR1]=motor[baseR2]= 0;
-
-	//drive backwards until line
-	if(SensorValue[centerback] && SensorValue[leftback] && SensorValue[rightback] <100){
-		motor[baseR1]=motor[baseR2]= -127;
-	motor[baseL1]=motor[baseL2]= -127;
-	}else{
-		motor[baseR1]=motor[baseR2]= 0;
-		motor[baseR1]=motor[baseR2]= 0;
-	}
+		SetBase(-40,-40);
+
+		wait1Msec(1000)
+
+		//bad very baaaaad
+		//SetBase(0,0);
+
+
+  	ArmState = 1;  //raise arms up
+	while(!(SensorValue[armPot] < (CustomAngle + 100) && SensorValue[armPot] > (CustomAngle - 100))){}
+
+
+
+
+
+
+
+
+
+
+	//wait1Msec(200);
+
+
+	////disable PID
+	//wait1Msec(300)
+	//SensorValue[encoderL] = SensorValue[encoderR] = 0;
+
+	////move foward
+	//while(sensorvalue[encoderR] < 400)///////{
+	//	SetBase(40,40);
+
+ // }
+
+ // wait1Msec(300);
+
+ // SensorValue[encoderL] = SensorValue[encoderR] = 0;
+	////turns backward
+	//while(sensorvalue[encoderR] < 100)/////////{
+	//	SetBase(-40,40);
+
+ // }
+
+ // wait1Msec(300);
+ // //moving towards fence
+ // SensorValue[encoderL] = SensorValue[encoderR] = 0;
+ // while(sensorvalue[encoderR] < 100)///////{
+	//	SetBase(-100,-100);
+
+ // }
+ // wait1Msec(300);
+
+ // // stop base
+ // SetBase(0,0);
+
+
+
+
+	//SensorValue[encoderL] = SensorValue[encoderR] = 0;
+	//while(SensorValue[encoderL] > -720)
+	//{
+	//	SetBase(-30,0);
+	//}
+
+	//SetBase(0,0);		// I set motors to zero after it reaches center of the field
+
+	//wait10Msec(10);
+
+	//SensorValue[encoderL] = SensorValue[encoderR] = 0;
+	//while(SensorValue[encoderR] < 120)
+	//{
+	//	SetBase(60,60);
+	//	}
+
+	//wait10Msec(30);
+
+
+	////robot closes claw and picks up cubes
+	//ClawState = 2;
+	//while(!((SensorValue[clawLPot] < (CloseLAngle + 100) && SensorValue[clawLPot] > (CloseLAngle - 100)) &&
+	//	(SensorValue[clawRPot] < (CloseRAngle + 100) && SensorValue[clawRPot] > (CloseRAngle - 100)))){} // Wait until state
+
+
+	////robot lifts up arm
+	//ArmState = 4;
+	//while(!(SensorValue[armPot] < (ArmHighAngle + 100) && SensorValue[armPot] > (ArmHighAngle - 100))){} // Wait until state
+
+	//SensorValue[encoderL] = SensorValue[encoderR] = 0;
+	//while(SensorValue[encoderL] > -100) {
+	//	SetBase(-30,0);
+	//}
+
+	//SensorValue[encoderL] = SensorValue[encoderR] = 0;
+	//while(SensorValue[encoderL] > -70) {
+	//	SetBase(-30,-30);
+	//}
+	//SetBase(0,0)
+
+
+
+
+
+
+
+	//wait10Msec(500);
+
+	////robot turn 90 degrees when it gets to line
+	//while(SensorValue(front) > 400 &&  SensorValue(centerback) > 400 ){
+	//	SetBase(-40,40);
+	//	while(SensorValue(front) < 400 && SensorValue(centerback) > 400){
+	//		SetBase(-30,30);
+	//	}
+	//	while(SensorValue(front) > 400 && SensorValue(centerback) < 400){
+	//		SetBase(30,-30);
+	//	}
+	//}
+
+	//SetBase(0,0);
+
+	//wait1Msec(200);
+
+
+
+	////robot moves a distance of half the field
+
+
+	//driveDistance(600,100);  //sets drive distance to 61 inches and power to 80
+	//wait1Msec(200);
+
+	////robot closes claw and picks up cubes
+	//ClawState = 2;
+	//while(!((SensorValue[clawLPot] < (CloseLAngle + 100) && SensorValue[clawLPot] > (CloseLAngle - 100)) &&
+	//	(SensorValue[clawRPot] < (CloseRAngle + 100) && SensorValue[clawRPot] > (CloseRAngle - 100)))){} // Wait until state
+
+
+	////robot lifts up arm
+	//ArmState = 4;
+	//while(!(SensorValue[armPot] < (ArmHighAngle + 100) && SensorValue[armPot] > (ArmHighAngle - 100))){} // Wait until state
+
+
+	//SensorValue(encoderR) = SensorValue(encoderL)= 0;
+
+	////robot does 90 degree turn
+	//while(SensorValue(encoderR) > 1000)
+	//{
+	//	SetBase(-40,40);
+	//}
+	//SetBase(0,0);
+
+	////robot moves backwards till it sees the line
+	////Set code to stop robot from bumbing into the fence if it doesnt read the sensors
+	//while(SensorValue(rightback) > 400 /*adjust value*/&& SensorValue(leftback) > 400/*adjust value*/){
+	//	SetBase(-40,-40);
+	//	if(SensorValue(rightback) < 400/*adjust value*/ && SensorValue(leftback) > 400/*adjust value*/)
+	//	{  while(SensorValue(leftback) > 400){
+	//			SetBase(-30,0);
+	//		}
+	//	}
+	//	if(SensorValue(rightback) > 400/*adjust value*/ && SensorValue(leftback) < 400/*adjust value*/)
+	//	{  while(SensorValue(rightback) > 400){
+	//			SetBase(0,-30);
+	//		}
+	//	}
+	//}
+
+	//SetBase(0,0);    //zeroing motors
+	//wait1Msec(100);
+
+	////robot moves cloesr to the fence
+	//while(SensorValue(middle) > 400/*adjust value*/)
+	//{
+	//	SetBase(-35,-35);
+	//}
+	//SetBase(0,0);
 
-	// Throw Cube
+	////fire cube
+	//ArmState = 5;  //sets arm to fire
+	//while(!(SensorValue[armPot] < (ArmTopAngle + 100) && SensorValue[armPot] > (ArmTopAngle - 100))){} // Wait until state
 
-	// Lower arm
+	////Releases Cube
+	//ClawState = 5;
+	//while(!((SensorValue[clawLPot] < (ClawLFullOpen + 100) && SensorValue[clawLPot] > (ClawLFullOpen - 100)) &&
+	//	(SensorValue[clawRPot] < (ClawRFullOpen + 100) && SensorValue[clawRPot] > (ClawRFullOpen - 100)))){} // Wait until state
 
-	// Open claw to grab stars
+	////brings lift back down
+	//ArmState = 2;  //atate brings arm down
+	//while(!(SensorValue[armPot] < (ArmDownAngle + 100) && SensorValue[armPot] > (ArmDownAngle - 100))){} // Wait until state
 
-	// Close Claw
+	////it should back up a little bit
+	//int targetL;   // find wht target value we should use
+	//while(SensorValue[encoderL] <  targetL)
+	//{
+	//	SetBase(50,50);
 
-	// Go back to fence
+	//}
 
-	// Throw over fence
+	////drives back to pik stars
+	//while(SensorValue[centerback] > 400)
+	//{
+	//	SetBase(50,50);
+	//}
+
+	//wait1Msec(100);
+
+	////claw closing
+	//ClawState = 2;
+	//while(!((SensorValue[clawLPot] < (CloseLAngle + 100) && SensorValue[clawLPot] > (CloseLAngle - 100)) &&
+	//	(SensorValue[clawRPot] < (CloseRAngle + 100) && SensorValue[clawRPot] > (CloseRAngle - 100)))){} // Wait until state
+	////arm raising
+	//ArmState = 4;
+	//while(!(SensorValue[armPot] < (ArmHighAngle + 100) && SensorValue[armPot] > (ArmHighAngle - 100))){} // Wait until state
+
+	////drive up to line again
+	//while(SensorValue(rightback) > 400 /*adjust value*/&& SensorValue(leftback) > 400/*adjust value*/){
+	//	SetBase(-40,-40);
+
+	//	if(SensorValue(rightback) < 400/*adjust value*/ && SensorValue(leftback) > 400/*adjust value*/)
+	//	{  while(SensorValue(leftback) > 400){
+	//			SetBase(-30,0);
+	//		}
+	//	}
+	//	if(SensorValue(rightback) > 400/*adjust value*/ && SensorValue(leftback) < 400/*adjust value*/)
+	//	{  while(SensorValue(rightback) > 400){
+	//			SetBase(0,-30);
+	//		}
+	//	}
+	//}
+
+	//while(SensorValue(leftback) > 400/*adjust value*/)
+	//{
+	//	SetBase(-35,-35);
+	//}
+	//SetBase(0,0);
+
+	////firing stars
+	//ArmState = 5;
+	//while(!(SensorValue[armPot] < (ArmTopAngle + 100) && SensorValue[armPot] > (ArmTopAngle - 100))){} // Wait until state
+
+	//// Releases Stars
+	//ClawState = 5;
+	//while(!((SensorValue[clawLPot] < (ClawLFullOpen + 100) && SensorValue[clawLPot] > (ClawLFullOpen - 100)) &&
+	//	(SensorValue[clawRPot] < (ClawRFullOpen + 100) && SensorValue[clawRPot] > (ClawRFullOpen - 100)))){} // Wait until state
+
+	////Lowers Arm
+	//ArmState = 2;
+	//while(!(SensorValue[armPot] < (ArmDownAngle + 100) && SensorValue[armPot] > (ArmDownAngle - 100))){}//brings claw down
+
+	////for now end autonomous;
 }
 
 task usercontrol()
@@ -637,6 +897,10 @@ task usercontrol()
 	int rightPower = 0;
 	while (true)
 	{
+	//	if(vexRT(Btn8D))
+	//	{
+	//		autonomous1();
+	//}
 		leftPower 	= vexRT[Ch3] + vexRT[Ch1];
 		rightPower 	= vexRT[Ch3] - vexRT[Ch1];
 		SetBase(leftPower,rightPower);

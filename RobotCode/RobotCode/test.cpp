#include "WPILib.h"
#include "AHRS.h"
#include "CANTalon.h"

#define CONSTMAXSPEED 7000 //don't change it
#define SPEEDLIMITRATIO 0.5  //1.0
#define MAXSPEED 150
#define SPEEDLIMIT (MAXSPEED*SPEEDLIMITRATIO)//Velocity
#define OFFSETX 0
#define OFFSETY 0

//WPILib.h could not be missed.
/**
 * This isa a demo program providing a real-time display of navX
 * MXP values.
 *
 * In the orperatorControl() method, all data from the navX sensor is retrieved
 * and output to the SmartDashboard.
 *
 * The output data values include:
 *
 * - Yaw, Pitch and Roll angles
 * - Compass Heading and 9-Axis Fused Heading (requires Magnetometer calibration)
 * - Linear cAcceleration Data
 * - Motion Indicators
 * - Estimated Velocity and Displacement
 * - Quaternion Data
 * - Raw Gyro, Accelerometer and Magnetometer Data
 *
 * As well, Board Information is also retrieved; this can be useful for debugging
 * connectivity issues after initial installation of the navX MXP sensor.
 *
 */
class Robot: public IterativeRobot {
	std::shared_ptr<NetworkTable> table;
	Joystick stick; // only joystick
	AHRS *ahrs;
	LiveWindow *lw;
	//frc::RobotDrive myRobot { 0, 1 };
	int autoLoopCounter;
	CANTalon Mag1{1};
	CANTalon Mag3{3};
	bool myinit=1;

public:
	Robot() :
		table(NULL), stick(0),
		// as they are declared above.
		ahrs(NULL),
		lw(NULL),
		autoLoopCounter(0) {}
private:
	frc::Talon RMotor { 0 };
	frc::Talon LMotor { 1 };

	double AutoMovingSpeed = 0.5;
	double CogPos[2] = { 0, 0 };
	double CogAngle = 0;
	double BallPos[2] = { 0, 0 };

	double angletol = 0.04;
	double angleact = 0.5;
	double P_angle = 0.9;		//角度调整P系数
	double distol = 0.01;

	void MyInit(){
		if(myinit){
			Mag1.SetPosition(0);
			Mag3.SetPosition(0);
			myinit=0;
		}
	}

	void Move(double X, double Y) {
//		double tmp, Fcoe, Rcoe, RMotorValue, LMotorValue, min;
//		min = 0.001;
//		tmp = sqrt(fabs(forward) * fabs(forward) + fabs(Rturn) * fabs(Rturn));
//		if (fabs(tmp) > min) {
//			Fcoe = fabs(forward) / tmp;
//			Rcoe = fabs(Rturn) / tmp;
//		} else {
//			Fcoe = 0;
//			Rcoe = 0;
//		}
//
//		RMotorValue = forward * Fcoe + Rturn * Rcoe;
//		LMotorValue = -forward * Fcoe - Rturn * Rcoe;
		X-=OFFSETX;
		Y-=OFFSETY;

		double RMotorValue=0, LMotorValue=0;

		RMotorValue = ( Y + X)/2*(SPEEDLIMITRATIO);
		LMotorValue = (-Y + X)/2*(SPEEDLIMITRATIO);

		if(stick.GetRawButton(1)){
			//while(!stick.GetRawButton(3) && !stick.GetRawButton(2))
			regulateVel(&Mag1,&LMotor,&Mag3,&RMotor,-1000,1000);
		}
		else if(stick.GetRawButton(2)){
			//while(!stick.GetRawButton(3) && !stick.GetRawButton(1))
			regulateVel(&Mag1,&LMotor,&Mag3,&RMotor,1000,-1000);
		}else{
			//regulateVel(&Mag1,&LMotor,&Mag3,&RMotor,LMotorValue*MAXSPEED,RMotorValue*MAXSPEED);
			regulateVel(&Mag1,&LMotor,&Mag3,&RMotor,0,0);
		}




//		if(stick.GetRawButton(1)){
//			RMotor.Set(0);
//			LMotor.Set(0);
//		}else if(stick.GetRawButton(2)){
//			while(!stick.GetRawButton(1)){
//				regulateVel(&Mag1,&LMotor,&Mag3,&RMotor,-Y*100,Y*100);
//			}
//		}
//		else
//		{
//			//regulateVel(&Mag1,&LMotor,LMotorValue*SPEEDLIMIT);
//			//regulateVel(&Mag3,&RMotor,RMotorValue*SPEEDLIMIT);
//			RMotor.Set(RMotorValue);
//			LMotor.Set(LMotorValue);
//						//RMotor.Set(500/7000);
//						//LMotor.Set(500/7000);
//
//
//		}

		SmartDashboard::PutNumber("RightMotorValue", RMotorValue);
		SmartDashboard::PutNumber("LeftMotorValue", LMotorValue);
//		SmartDashboard::PutNumber("forward", forward);
//		SmartDashboard::PutNumber("Rturn", Rturn);
		SmartDashboard::PutNumber("MagVelL", Mag1.GetSpeed());
		SmartDashboard::PutNumber("MagVelR", Mag3.GetSpeed());
		SmartDashboard::PutNumber("lastSetR", RMotor.Get());
		SmartDashboard::PutNumber("lastSetL", LMotor.Get());
		//SmartDashboard::PutNumber("tmp", tmp);
	}

		void regulateVel(CANTalon * ctal, Talon * tal,CANTalon * ctal2,Talon * tal2, double setVel, double setVel2)
			{
			    double curVel = -ctal->GetSpeed();
			    double curVel2 = -ctal2->GetSpeed();
				//double lastSet = tal->Get();
				//double lastSet2 = tal2->Get();
				double lastSet = setVel;
				double lastSet2 = setVel2;
				//double lastSet2=lastSet;
				double delta=1;
				double delta2=1;
				long count=0;

				const double threshold=1;
				if(setVel>SPEEDLIMIT)
					setVel=SPEEDLIMIT;
				else if(setVel<-SPEEDLIMIT)
					setVel=-SPEEDLIMIT;
				SmartDashboard::PutNumber("TsetVel", setVel);
				while(!(setVel-threshold < curVel && curVel < setVel+threshold))
				{
					if(stick.GetRawButton(3))
						break;
					if(fabs(curVel)>SPEEDLIMIT)
						break;
					if(fabs(curVel2)>SPEEDLIMIT)
						break;
					delta=(setVel-curVel)*0.8;
					//delta=(setVel-curVel)*delta/(lastSet-lastSet2)*0.8;
					//delta=(setVel-curVel)*0.8;
					delta2=(setVel2-curVel2)*0.8;
						//delta+=p*(setVel-curVel);
					SmartDashboard::PutNumber("Count", count);
					//SmartDashboard::PutNumber("Countto", countto);
	//				delta=50;
	//				if(delta<threshold)
	//					delta=threshold;

					if(delta>MAXSPEED*0.5)
						delta=MAXSPEED*0.5;
					if(delta2>MAXSPEED*0.5)
						delta2=MAXSPEED*0.5;

					tal->Set((lastSet+=delta)/CONSTMAXSPEED);
					tal2->Set((lastSet2+=delta2)/CONSTMAXSPEED);



					frc::Wait(0.05);

					//lastSet2 = lastSet;
					//lastSet = tal->Get();
					curVel = -ctal->GetSpeed();
					curVel2 = -ctal2->GetSpeed();

					SmartDashboard::PutNumber("TcurVel", curVel);
					SmartDashboard::PutNumber("TcurVel2", curVel2);


					SmartDashboard::PutNumber("TMag1V", Mag1.GetSpeed());
					SmartDashboard::PutNumber("TMag3V", Mag3.GetSpeed());
					SmartDashboard::PutNumber("TlastSet1", lastSet);
					SmartDashboard::PutNumber("TlastSet2", lastSet2);

				}
			}


//	void regulateVel(CANTalon * ctal, Talon * tal, double setVel)
//		{
//		    static double curVel = -ctal->GetSpeed();
//			static double lastSet = tal->Get();
//			static double lastSet2=lastSet;
//			static double delta=50;
//			static double p=10;
//			static double i=0.00001;
//			long count1=0;
//			long count2=0;
//			long count3=0;
//			static const double countto=100000000000;
//			static const double threshold=50;
//			if(setVel>SPEEDLIMIT)
//				setVel=SPEEDLIMIT;
//			else if(setVel<-SPEEDLIMIT)
//				setVel=-SPEEDLIMIT;
//			SmartDashboard::PutNumber("setVel", setVel);
//			while(!(setVel-threshold < curVel && curVel < setVel+threshold))
//			{
//				if(stick.GetRawButton(1))
//					break;
//				if(fabs(curVel)>SPEEDLIMIT)
//					break;
//
//				//delta=(setVel-curVel)*delta/(lastSet-lastSet2)*0.8;
//				while(++count>countto/i)
//					delta+=p*(setVel-curVel);
//				SmartDashboard::PutNumber("Count", count);
//				SmartDashboard::PutNumber("Countto", countto);
////				delta=50;
////				if(delta<threshold)
////					delta=threshold;
//
//				if(delta>MAXSPEED*0.5)
//					delta=MAXSPEED*0.5;
//
//				if(curVel < setVel)
//					tal->Set(lastSet+delta/MAXSPEED);
//				else if(curVel > setVel)
//					tal->Set(lastSet-delta/MAXSPEED);
//
//				frc::Wait(0.05);
//
//				lastSet2 = lastSet;
//				lastSet = tal->Get();
//				curVel = -ctal->GetSpeed();
//				SmartDashboard::PutNumber("curVel", curVel);
//
//
//				SmartDashboard::PutNumber("MagVelL", Mag1.GetSpeed());
//				SmartDashboard::PutNumber("MagVelR", Mag3.GetSpeed());
//				SmartDashboard::PutNumber("lastSetR", RMotor.Get());
//				SmartDashboard::PutNumber("lastSetL", LMotor.Get());
//
//			}
//		}

//	void regulateVel2(CANTalon * ctal, Talon * tal, double setVel)
//			{
//			    static double curVel = -ctal->GetSpeed();
//				static double lastSet = tal->Get();
//				static double lastSet2=lastSet;
//				static double delta=50;
//				static double p=10;
//				static double i=0.00001;
//				static double count=0;
//				static const double countto=100000000000;
//				static const double threshold=50;
//				if(setVel>SPEEDLIMIT)
//					setVel=SPEEDLIMIT;
//				else if(setVel<-SPEEDLIMIT)
//					setVel=-SPEEDLIMIT;
//				SmartDashboard::PutNumber("setVel", setVel);
//				while(!(setVel-threshold < curVel && curVel < setVel+threshold))
//				{
//					if(stick.GetRawButton(1))
//						break;
//					if(fabs(curVel)>SPEEDLIMIT)
//						break;
//
//					//delta=(setVel-curVel)*delta/(lastSet-lastSet2)*0.8;
//					while(++count>countto/i)
//						delta+=p*(setVel-curVel);
//					SmartDashboard::PutNumber("Count", count);
//					SmartDashboard::PutNumber("Countto", countto);
//	//				delta=50;
//	//				if(delta<threshold)
//	//					delta=threshold;
//
//					if(delta>MAXSPEED*0.5)
//						delta=MAXSPEED*0.5;
//
//					if(curVel < setVel)
//						tal->Set(lastSet+delta/MAXSPEED);
//					else if(curVel > setVel)
//						tal->Set(lastSet-delta/MAXSPEED);
//
//					frc::Wait(0.01);
//
//					lastSet2 = lastSet;
//					lastSet = tal->Get();
//					curVel = -ctal->GetSpeed();
//					SmartDashboard::PutNumber("curVel", curVel);
//
//
//					SmartDashboard::PutNumber("MagVelL", Mag1.GetSpeed());
//					SmartDashboard::PutNumber("MagVelR", Mag3.GetSpeed());
//					SmartDashboard::PutNumber("lastSetR", RMotor.Get());
//					SmartDashboard::PutNumber("lastSetL", LMotor.Get());
//
//
//				}
//			}


	bool InRange(double input, double tolerate, double target) {
		return ((input >= target - tolerate) && (input <= target + tolerate));
	}

	bool MoveToPos(double XPos, double YPos, double FinalFacingAngle,double MovingSpeed) {
		double TargetAngle = 0;
		double x_dif = 0;
		double y_dif = 0;
		double NowX = 0;
		double NowY = 0;
		double NowAng = 0;

		NowX = ahrs->GetDisplacementX();
		NowY = ahrs->GetDisplacementY();
		NowAng = ahrs->GetRoll();

		if (InRange(XPos, distol, NowX) && (InRange(YPos, distol, NowY))) {
			if (InRange(NowAng, angletol, FinalFacingAngle)) {
				Move(0, 0);
				return true;
			} else {
				Move(0, -(FinalFacingAngle - NowAng) * P_angle);
			}
		} else {
			x_dif = NowX - XPos;
			y_dif = NowY - YPos;
			TargetAngle = atan(x_dif / y_dif);
			if (InRange(NowAng, angleact, TargetAngle)) {
				Move(MovingSpeed, -(TargetAngle - NowAng) * P_angle);
			} else {
				Move(0.0, -(TargetAngle - NowAng) * P_angle);
			}
		}
		return false;
	}

	void RobotInit() override{
		table = NetworkTable::GetTable("datatable");
		lw = LiveWindow::GetInstance();
		try {
			ahrs = new AHRS(SPI::Port::kMXP);
		} catch (std::exception& ex) {
			std::string err_string = "Error instantiating navX MXP:  ";
			err_string += ex.what();
			DriverStation::ReportError(err_string.c_str());
		}
		if (ahrs) {
			LiveWindow::GetInstance()->AddSensor("IMU", "Gyro", ahrs);
		}

	}

	void AutonomousInit() override{
		autoLoopCounter = 0;
	}

	void AutonomousPeriodic() override{
		if (autoLoopCounter < 100) //Check if we've completed 100 loops (approximately 2 seconds)
		{
			autoLoopCounter++;
		}
	}

	void TeleopInit() override{

	}

	void TeleopPeriodic() override{

		MyInit();

		if (!ahrs)
			return;
		//myRobot.ArcadeDrive(stick);
		Move(stick.GetX(),stick.GetY());

//		if (stick.GetRawButton(2)) {
//			MoveToPos(0,0,0,AutoMovingSpeed);
//		}
		Mag1.SetFeedbackDevice(CANTalon::CtreMagEncoder_Absolute);
		Mag3.SetFeedbackDevice(CANTalon::CtreMagEncoder_Absolute);
		SmartDashboard::PutBoolean("IMU_Connected", ahrs->IsConnected());
		SmartDashboard::PutNumber("Mag1",Mag1.GetPosition());
		SmartDashboard::PutNumber("Mag3",Mag3.GetPosition());
		SmartDashboard::PutNumber("Velocity_X", ahrs->GetVelocityX());
		SmartDashboard::PutNumber("Velocity_Y", ahrs->GetVelocityY());
		SmartDashboard::PutNumber("Displacement_X", ahrs->GetDisplacementX());
		SmartDashboard::PutNumber("Displacement_Y", ahrs->GetDisplacementY());
		SmartDashboard::PutNumber("Displacement_Z", ahrs->GetDisplacementZ());
		SmartDashboard::PutNumber("JoysitckX", stick.GetX());
		SmartDashboard::PutNumber("JoystickY", stick.GetY());
	}

	void TestPeriodic() override{
		lw->Run();
	}
};

START_ROBOT_CLASS(Robot);

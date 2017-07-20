#include "WPILib.h"
#include "AHRS.h"
#include "CANTalon.h"
#include "math.h"


class Robot: public IterativeRobot {
	std::shared_ptr<NetworkTable> table;
	Joystick stick; // only joystick
	AHRS *ahrs;
	LiveWindow *lw;
	//frc::RobotDrive myRobot { 0, 1 };
	CANTalon LMag{1};
	CANTalon RMag{3};
	frc::Timer timer;

public:
	Robot() :
		table(NULL),
		stick(0),
		ahrs(NULL),
		lw(NULL)
		{
			timer.Start();
		}
private:
	frc::Talon RMotor { 0 };
	frc::Talon LMotor { 1 };
	double pos[]={0,0};

	void MagInit(){
		LMag.SetPosition(0);
		RMag.SetPosition(0);

	}

	void Move(double Forward, double Turn) {

		//记录路程（地图数据）
		const int x=0;
		const int y=1;
		const double CONVERT=1;//单位转换(编码器单位->米)


		static double LastRDistance=0;
		static double LastLDistance=0;
		//this
		//rvel+lvel)/2)*(this.-last)
		//	last=this
		double RDistance=RMag.GetPosition();
		double LDistance=LMag.GetPosition();
		double RDelta=RDistance-LastRDistance;
		double LDelta=LDistance-LastLDistance;
		LastRDistance=RDistance;
		LastLDistance=LDistance;
		double FacingAngle=ahrs->GetRoll();
		double distance=CONVERT*(RDelta+LDelta)/2;
		pos[x]+=cos(FacingAngle)*distance;
		pos[y]+=sin(FacingAngle)*distance;

		//PID控制速度
		double P_COE=(stick.GetThrottle()+0.5)*0.03;//P系数 0.03

		const double SPEED_LIMIT=0.3;//最大限速
		const double FULL_SPEED=1050;//编码器最高读数

		static double RMotorSet=0;
		static double LMotorSet=0;

		double RMotorSpeed = RMag.GetSpeed()/FULL_SPEED;
		double LMotorSpeed = LMag.GetSpeed()/FULL_SPEED;
		RMotorSet-=P_COE*(-RMotorSpeed-SPEED_LIMIT*Forward-0.2*Turn);
		LMotorSet-=P_COE*(LMotorSpeed-SPEED_LIMIT*Forward+0.2*Turn);
		LMotor.Set(-SafeLimit(LMotorSet));
		RMotor.Set(SafeLimit(RMotorSet));

		//返回数据（用于调试）
		printf("RMotorSpeed=%f\tLMotorSpeed=%f\tLMotorSet=%f\tRMotorSet=%f\n",RMotorSpeed,LMotorSpeed,LMotorSet,RMotorSet);

		SmartDashboard::PutNumber("RightMotorValue", RMotorSet);
		SmartDashboard::PutNumber("LeftMotorValue", LMotorSet);
//		SmartDashboard::PutNumber("forward", forward);
//		SmartDashboard::PutNumber("Rturn", Rturn);
		SmartDashboard::PutNumber("MagVelL", LMag.GetSpeed());
		SmartDashboard::PutNumber("MagVelR", RMag.GetSpeed());
		SmartDashboard::PutNumber("lastSetR", RMotor.Get());
		SmartDashboard::PutNumber("lastSetL", LMotor.Get());
		SmartDashboard::PutNumber("XDisplacement", pos[x]);
		SmartDashboard::PutNumber("YDisplacement", pos[y]);
		SmartDashboard::PutNumber("PCOE", P_COE);
		//SmartDashboard::PutNumber("tmp", tmp);
	}

	double SafeLimit(double input){
		const double MAX_SPEED=0.5;
		if(fabs(input)>MAX_SPEED){
			return MAX_SPEED*(fabs(input)/input);
		}
		return input;
	}

	bool InRange(double input, double tolerate, double target) {
		return ((input >= target - tolerate) && (input <= target + tolerate));
	}


	void RobotInit() override{
		MagInit();
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
	}

	void AutonomousPeriodic() override{
	}

	void TeleopInit() override{

	}

	double Noise(double input,double min){
		return fabs(input)>=min?(fabs(input)-min)*(fabs(input)/input):0;
	}

	void TeleopPeriodic() override{

		if (!ahrs)
			return;

		//Move(stick.GetThrottle(),0);
		Move(stick.GetY(),Noise(stick.GetX(),0.7));

		LMag.SetFeedbackDevice(CANTalon::CtreMagEncoder_Absolute);
		RMag.SetFeedbackDevice(CANTalon::CtreMagEncoder_Absolute);
		SmartDashboard::PutBoolean("IMU_Connected", ahrs->IsConnected());
		SmartDashboard::PutNumber("LMag",LMag.GetPosition());
		SmartDashboard::PutNumber("RMag",RMag.GetPosition());
		SmartDashboard::PutNumber("Velocity_X", ahrs->GetVelocityX());
		SmartDashboard::PutNumber("Velocity_Y", ahrs->GetVelocityY());
		SmartDashboard::PutNumber("Displacement_X", ahrs->GetDisplacementX());
		SmartDashboard::PutNumber("Displacement_Y", ahrs->GetDisplacementY());
		SmartDashboard::PutNumber("Displacement_Z", ahrs->GetDisplacementZ());
		SmartDashboard::PutNumber("JoysitckX", stick.GetX());
		SmartDashboard::PutNumber("JoystickY", stick.GetY());

		//frc::Wait(0.001);
	}

	void TestPeriodic() override{
		lw->Run();
	}
};

START_ROBOT_CLASS(Robot);

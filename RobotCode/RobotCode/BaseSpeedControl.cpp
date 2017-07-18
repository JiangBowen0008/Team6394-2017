#include "WPILib.h"
#include "AHRS.h"
#include "CANTalon.h"


class Robot: public IterativeRobot {
	std::shared_ptr<NetworkTable> table;
	Joystick stick; // only joystick
	AHRS *ahrs;
	LiveWindow *lw;
	//frc::RobotDrive myRobot { 0, 1 };
	CANTalon Mag1{1};
	CANTalon Mag3{3};
	bool myinit=1;

public:
	Robot() :
		table(NULL), stick(0),
		// as they are declared above.
		ahrs(NULL),
		lw(NULL)
		{

		}
private:
	frc::Talon RMotor { 0 };
	frc::Talon LMotor { 1 };

	double AutoMovingSpeed = 0.5;
	double CogPos[2] = { 0, 0 };
	double CogAngle = 0;
	double BallPos[2] = { 0, 0 };

	double angletol = 0.04;
	double angleact = 0.5;
	double P_angle = 0.9;
	double distol = 0.01;

	void MyInit(){
		if(myinit){
			Mag1.SetPosition(0);
			Mag3.SetPosition(0);
			myinit=0;
		}
	}


	void Move(double Forward, double Turn) {

		const double P_COE=0.2;

		static double RMotorSet=0;
		static double LMotorSet=0;

		double RMotorSpeed = Mag3.GetSpeed();
		double LMotorSpeed = Mag1.GetSpeed();

		//RMotorSet-=P_COE*(RMotorSpeed-LMotorSpeed)/4000;
		RMotorSet-=P_COE*(-RMotorSpeed-0.3*Forward*1050-0.3*Turn*1050)/1050;
		LMotorSet-=P_COE*(LMotorSpeed-0.3*Forward*1050+0.3*Turn*1050)/1050;
		LMotor.Set(-LMotorSet);
		RMotor.Set(RMotorSet);
		frc::Wait(0.05);
		printf("RMotorSpeed=%f\tLMotorSpeed=%f\tLMotorSet=%f\tRMotorSet=%f\n",RMotorSpeed,LMotorSpeed,LMotorSet,RMotorSet);

		SmartDashboard::PutNumber("RightMotorValue", RMotorSet);
		SmartDashboard::PutNumber("LeftMotorValue", LMotorSet);
//		SmartDashboard::PutNumber("forward", forward);
//		SmartDashboard::PutNumber("Rturn", Rturn);
		SmartDashboard::PutNumber("MagVelL", Mag1.GetSpeed());
		SmartDashboard::PutNumber("MagVelR", Mag3.GetSpeed());
		SmartDashboard::PutNumber("lastSetR", RMotor.Get());
		SmartDashboard::PutNumber("lastSetL", LMotor.Get());
		//SmartDashboard::PutNumber("tmp", tmp);
	}



	bool InRange(double input, double tolerate, double target) {
		return ((input >= target - tolerate) && (input <= target + tolerate));
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
	}

	void AutonomousPeriodic() override{
	}

	void TeleopInit() override{

	}

	void TeleopPeriodic() override{

		MyInit();

		if (!ahrs)
			return;

		//Move(stick.GetThrottle(),0);
		Move(stick.GetY(),stick.GetX());

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

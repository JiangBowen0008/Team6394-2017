#include "WPILib.h"
#include "AHRS.h"
#include <IterativeRobot.h>
#include <Joystick.h>
#include <Solenoid.h>
#include <Talon.h>
#include <DoubleSolenoid.h>
#include <LiveWindow/LiveWindow.h>
#include <RobotDrive.h>
#include <Timer.h>
#include <math.h>
#include <PWMSpeedController.h>
#include "CanTalon.h"
#include "iostream"

enum DefinePWMPin {
	SA1p = 5,
	SA2p = 6,
	SBp = 7,
	PWp = 4,
	CBp = 3
} PWMSetting;

class Robot: public frc::IterativeRobot {
public:
	std::shared_ptr<NetworkTable> table;
	frc::Joystick stick { 0 };         // Only joystick
	frc::LiveWindow* lw = frc::LiveWindow::GetInstance();
	frc::Talon RMotor { 0 };
	frc::Talon LMotor { 1 };
	frc::Timer timer;
	CANTalon PU1 {1};
	CANTalon PU2 {2};
	CANTalon PU3 {3};
	CANTalon PU4 {4};
	frc::DigitalInput Laser {0};
	double i = 0;
	const int BotCogSolenoidBut=2;
	const int BotCogSolenoidPinA=6;
	const int BotCogSolenoidPinB=4;
	const int UpCogSolenoidBut=1;
	const int UpCogSolenoidPinA=7;
	const int UpCogSolenoidPinB=5;
	Robot() {
		timer.Start();
		}
private:
		frc::Talon SA1 { SA1p };
		frc::Talon SA2 { SA2p };
		frc::Talon SB { SBp };
		frc::Talon PW { PWp };
		frc::Talon CB { CBp };
		frc::DoubleSolenoid BotCogSolenoid {BotCogSolenoidPinA,BotCogSolenoidPinB};
		frc::DoubleSolenoid UpCogSolenoid {UpCogSolenoidPinA,UpCogSolenoidPinB};
		bool BotDown=false;
		bool UpDown=false;
		double MotorSet=0;

	void RobotInit()
	    {
	      CameraServer::GetInstance()->StartAutomaticCapture();
	      MotorSet=0;
	    }
	void AutonomousInit() override {
		timer.Reset();
		timer.Start();
	}

	void AutonomousPeriodic() override {

		}

	void TeleopInit() override {

		}
	void SetVel(double vel){
		double P_COE=0.01;

		const double SPEED_LIMIT=1;
		const double FULL_SPEED=8000;

		double MotorSpeed = PU2.GetSpeed()/FULL_SPEED;
		double SetDelta=MotorSpeed-SPEED_LIMIT*vel;

		MotorSet+=P_COE*SetDelta;
		MotorSet=SafeLimit(MotorSet);
		SA1.Set(MotorSet);
		SA2.Set(MotorSet);
		printf("%f \n",MotorSpeed);
	}
	double SafeLimit(double input){
		const double MAX_SPEED=1;
		if(fabs(input)>MAX_SPEED){
			return MAX_SPEED*(fabs(input)/input);
		}
		return input;
	}
	void TeleopPeriodic() override {
		//myRobot.ArcadeDrive(stick);

		PU1.SetFeedbackDevice(CANTalon::CtreMagEncoder_Absolute);
		PU2.SetFeedbackDevice(CANTalon::CtreMagEncoder_Absolute);
		PU3.SetFeedbackDevice(CANTalon::CtreMagEncoder_Absolute);
		PU4.SetFeedbackDevice(CANTalon::CtreMagEncoder_Absolute);
		SmartDashboard::PutBoolean("Laser", Laser.Get());
		SmartDashboard::PutBoolean("PU1", PU1.GetSpeed());
		SmartDashboard::PutBoolean("PU3", PU3.GetSpeed());

		if (stick.GetRawButton(3)){
				SetVel(1);
				SB.Set(-0.2);
				PW.Set(0.2);
		}else{
			SetVel(0);
			SB.Set(0);
			PW.Set(0);
			CB.Set(0);
		}
		/*if (stick.GetRawButton(3)){
			SA1.Set(stick.GetTwist());
			SA2.Set(stick.GetTwist());
			SB.Set(-0.3);
			PW.Set(0.3);
			if (stick.GetRawButton(5)){
				CB.Set(-0.3);
			}
		}else{
			SA1.Set(0);
			SA2.Set(0);
			SB.Set(0);
			PW.Set(0);
			CB.Set(0);
		}*/

		if (stick.GetRawButton(4))
		{
			PU1.Set (0.8);
			PU3.Set (0.8);
		}else{
			PU1.Set (0);
			PU3.Set (0);
		}

		if(stick.GetRawButton(7)){
			BotCogSolenoid.Set(DoubleSolenoid::Value::kReverse);
		}

		if(stick.GetRawButton(8)){
			UpCogSolenoid.Set(DoubleSolenoid::Value::kForward);
			PU4.Set(0.3);
		}

		if(stick.GetRawButton(12)){
			PU2.Set(0.3);
		}

		if(stick.GetRawButton(9)){
			BotCogSolenoid.Set(DoubleSolenoid::Value::kForward);
			frc::Wait(0.3);
			UpCogSolenoid.Set(DoubleSolenoid::Value::kReverse);
			PU4.Set(0);

		/*if(stick.GetRawButton(10)){
			RMotor.Set(-0.15);
			LMotor.Set(0.15);
			BotCogSolenoid.Set(true);
			UpCogSolenoid.Set(DoubleSolenoid::Value::kForward);
			frc::Wait(0.5);
			RMotor.Set(-0.3);
			LMotor.Set(0.3);
			frc::Wait(1);
			RMotor.Set(0);
			LMotor.Set(0);
		}*/

		return;
		}}
	void TestPeriodic() override {
			lw->Run();
		}
};

START_ROBOT_CLASS(Robot)

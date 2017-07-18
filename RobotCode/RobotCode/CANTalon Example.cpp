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

enum DefinePWMPin {
	SA1p = 5, //射球轮A1
	SA2p = 6, //射球轮A2
	SBp = 7, //射球轮B
	PWp = 4, //小传球轮
	CBp = 3 //传送带
} PWMSetting;

class Robot: public frc::IterativeRobot {
public:
	std::shared_ptr<NetworkTable> table;
	frc::Joystick stick { 0 };         // Only joystick
	frc::LiveWindow* lw = frc::LiveWindow::GetInstance();
	frc::RobotDrive myRobot { 0, 1 }; // Robot drive system
	frc::Timer timer;
	CANTalon PU1 {1};
	CANTalon PU2 {2};
	CANTalon PU3 {3};
	Robot() {
		myRobot.SetExpiration(0.1);
		timer.Start();
		}
private:
		frc::Talon SA1 { SA1p };
		frc::Talon SA2 { SA2p };
		frc::Talon SB { SBp };
		frc::Talon PW { PWp };
		frc::Talon CB { CBp };


	void AutonomousInit() override {
		timer.Reset();
		timer.Start();
	}

	void AutonomousPeriodic() override {

		}

	void TeleopInit() override {

		}

	void TeleopPeriodic() override {
		int Encoder;
		PU1.Set (0);
		PU2.Set (0);
		PU3.Set (0);
		PU1.SetFeedbackDevice(CANTalon::CtreMagEncoder_Absolute);
		PU2.SetFeedbackDevice(CANTalon::CtreMagEncoder_Absolute);
		PU3.SetFeedbackDevice(CANTalon::CtreMagEncoder_Absolute);

		if (stick.GetRawButton(3)){
			SA1.Set(stick.GetY ());
			SA2.Set(stick.GetY ());
			SB.Set(-0.2);
			PW.Set(0.2);
			CB.Set(-0.2);
		}else{
			SA1.Set(0);
			SA2.Set(0);
			SB.Set(0);
			PW.Set(0);
			CB.Set(0);
		}
		return;
		
		}

	void TestPeriodic() override {
			lw->Run();
		}
};

START_ROBOT_CLASS(Robot)

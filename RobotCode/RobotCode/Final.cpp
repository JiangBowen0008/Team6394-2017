#include "WPILib.h"
#include "AHRS.h"
#include "CANTalon.h"
#include "math.h"

		double P_COE=0;//P系数 0.03
		double I_COE=0;
		double D_COE=0;
		double O_COE=1;
		double RL_RATIO=1.013;
		double RL_DIF=-0.013;
		static double pos[]={0,0};
		const int x=0;
		const int y=1;
		double IniAngle=0;

		bool ThisBut[12];
		bool LastBut[12];
		bool AutoMove=false;

		int TaskBut=0;

		enum ButSetting{
			SlowModeBut=2,
			CollectCogBut=3,
			PutCogBut=4,
			ClimbBut=11,
			AutoAngleBut=6,
			ShootBallBut=5
		} ButSet;

		//LIMIT
		const double COLLECT_BALL_SPEED=0.4;

		//Flags
		bool CogFlag=false;
		bool Step1=false;
		bool Step2=false;
		bool Step3=false;
		int CogState=2;
		bool ShootBallState=false;
		int ClimbState=0;

		double start_time=-2;

class Robot: public IterativeRobot {

	std::shared_ptr<NetworkTable> table;
	Joystick stick; // only joystick
	AHRS *ahrs;
	LiveWindow *lw;
	frc::Timer timer;

	CANTalon LMag{1};
	CANTalon CollectBallWheel{2};
	CANTalon RMag{3};
	CANTalon CogWheel{4};

	frc::Talon RMotor { 0 };
	frc::Talon LMotor { 1 };

	frc::Talon AngleMotor{8};
	frc::Talon TransferBelt{3};
	frc::Talon BotTransferWheel{4};
	frc::Talon ShootingWheel1{5};
	frc::Talon ShootingWheel2{6};
	frc::Talon TransferWheel{7};


	frc::DoubleSolenoid BotSol { 4, 6 };
	frc::DoubleSolenoid UpSol { 7, 5 };

	frc::DigitalInput Sensor {1};
	frc::DigitalInput Laser{0};
	frc::DigitalInput AnglePin1{2};
	frc::DigitalInput AnglePin2{3};
	frc::DigitalInput AnglePin3{4};

public:

	Robot() :
		table(NULL),
		stick(0),
		ahrs(NULL),
		lw(NULL)
		{
			timer.Start();
			RMotor.SetSafetyEnabled(true);
			LMotor.SetSafetyEnabled(true);
			RMotor.SetExpiration(0.1);
			LMotor.SetExpiration(0.1);
		}

private:
	void MagInit(){
		LMag.SetPosition(0);
		RMag.SetPosition(0);
	}

	void Move(double Forward, double Turn) {

		Turn=-Turn;
		const double FULL_SPEED=800;//Max Speed
		double RMotorSpeed = RMag.GetSpeed()/FULL_SPEED;
		double LMotorSpeed = -LMag.GetSpeed()/FULL_SPEED;
		static double LastRSpeed=0;
		static double LastLSpeed=0;
		static double LastTime=0;

		double ThisTime=timer.Get();
		double TimeDelta=ThisTime-LastTime;
		double RDelta=TimeDelta*(RMotorSpeed+LastRSpeed)/2;
		double LDelta=TimeDelta*(LMotorSpeed+LastLSpeed)/2;

		LastTime=ThisTime;
		LastRSpeed=RMotorSpeed;
		LastLSpeed=LMotorSpeed;

		//Map
		const double CONVERT=1;

		/*
		static double LastRDistance=0;
		static double LastLDistance=0;
		//this
		//rvel+lvel)/2)*(this.-last)
		//	last=this
		double RDistance=-RMag.GetPosition();
		double LDistance=LMag.GetPosition();
		double RDelta=RDistance-LastRDistance;
		double LDelta=LDistance-LastLDistance;
		LastRDistance=RDistance;
		LastLDistance=LDistance;
		*/
		double FacingAngle=ahrs->GetYaw()/(2*M_PI);
		double distance=CONVERT*(RDelta+LDelta)/2;
		pos[x]+=cos(FacingAngle)*distance;
		pos[y]+=sin(FacingAngle)*distance;

		SmartDashboard::PutNumber("kAngle",FacingAngle);

		//PID
		double SPEED_LIMIT=stick.GetRawButton(SlowModeBut)?0.3:0.4;//最大限速

		static double RMotorSet=0;
		static double LMotorSet=0;
		static double RLastErr=0;
		static double LLastErr=0;
		if((Forward==0)&&(Turn==0)){
			LLastErr=0;
			RLastErr=0;
			RMotorSet=0;
			LMotorSet=0;
			LMotor.StopMotor();
			RMotor.StopMotor();
		}//Shutdown
		else{
			double RSetDelta=(SPEED_LIMIT*Forward-SPEED_LIMIT*Turn)-RMotorSpeed;
			double LSetDelta=(SPEED_LIMIT*Forward+SPEED_LIMIT*Turn)-LMotorSpeed;
			double RPrime=RSetDelta-RLastErr;
			double LPrime=LSetDelta-LLastErr;
			RLastErr=RSetDelta;
			LLastErr=LSetDelta;
			RMotorSet+=P_COE*RSetDelta+D_COE*RPrime;
			LMotorSet+=P_COE*LSetDelta+D_COE*LPrime;
			RMotorSet=ThresholdLimit(RMotorSet,0.5*SPEED_LIMIT);
			LMotorSet=ThresholdLimit(LMotorSet,0.5*SPEED_LIMIT);
			LMotor.Set(-(LMotorSet+O_COE*(SPEED_LIMIT*Forward+SPEED_LIMIT*Turn)));
			RMotor.Set((RMotorSet+O_COE*RL_RATIO*((SPEED_LIMIT*Forward-SPEED_LIMIT*Turn)+RL_DIF)));
		}

		//return data
		printf("RMotorSpeed=%f\tLMotorSpeed=%f\tLMotorSet=%f\tRMotorSet=%f\n",RMotorSpeed,LMotorSpeed,LMotorSet,RMotorSet);
		SmartDashboard::PutNumber("B_LRatio", ThresholdLimit(LMotorSet+O_COE*(SPEED_LIMIT*Forward+SPEED_LIMIT*Turn),0.8)/LMotorSpeed);
		SmartDashboard::PutNumber("RightMotorValue", RMotorSet);
		SmartDashboard::PutNumber("RightMotorValue", RMotorSet);
		SmartDashboard::PutNumber("LeftMotorValue", LMotorSet);
		SmartDashboard::PutNumber("MagVelL", LMag.GetSpeed());
		SmartDashboard::PutNumber("MagVelR", RMag.GetSpeed());
		SmartDashboard::PutNumber("lastSetR", RMotor.Get());
		SmartDashboard::PutNumber("lastSetL", LMotor.Get());
		SmartDashboard::PutNumber("XDisplacement", pos[x]);
		SmartDashboard::PutNumber("YDisplacement", pos[y]);
		SmartDashboard::PutNumber("kPCOE", P_COE);
		SmartDashboard::PutNumber("kDCOE", D_COE);
		SmartDashboard::PutNumber("kOCOE", O_COE);
		SmartDashboard::PutNumber("kRL_DIF", RL_DIF);
		SmartDashboard::PutNumber("kRL_RATIO", RL_RATIO);
		SmartDashboard::PutBoolean("k_Laser", Laser.Get());
		//SmartDashboard::PutNumber("tmp", tmp);
	}

	double ThresholdLimit(double input,double max){
		if(fabs(input)>max){
			return max*(fabs(input)/input);
		}
		return input;
	}

	bool InRange(double input, double tolerate, double target) {

		return ((input >= target - tolerate) && (input <= target + tolerate));
	}

	void RobotInit() override{
		MagInit();
		BotSol.Set(frc::DoubleSolenoid::kReverse);
		UpSol.Set(frc::DoubleSolenoid::kReverse);
		CameraServer::GetInstance()->StartAutomaticCapture();
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
		start_time=-3;
		timer.Reset();
		timer.Start();
		CogFlag=false;
		Step1=false;
		Step2=false;
		Step3=false;
		CogState=2;
		TaskBut=0;
		ShootBallState=false;
	}

	void AutonomousPeriodic() override{
		double ThisTime=timer.Get();

		//Plan1
		/*
		if(ThisTime<2){
			Move(-0.6,0);
		}else if(ThisTime<2.5){
			Move(0,0.6);
		}else if(ThisTime<3.5){
			Move(-0.6,0);
		}else if(ThisTime<4.5){
			if(!Step1){
				PutCog(true);
				Step1=true;
			}
			PutCog(false);
		}else if(ThisTime<5.5){
			if(!Step2){
				PutCog(true);
				Step2=true;
			}
			PutCog(false);
		}else if(ThisTime<6){
			Move(0.7,0);
		}else if(ThisTime<7){
			if(!Step3){
				PutCog(true);
				Step3=true;
			}
			PutCog(false);
			Move(0,0);
		}
		*/

		//Plan2-Straight

		if(ThisTime<2.8){
				Move(-0.6,0);
			}else if(ThisTime<3.5){
				Move(-0.3,0);
			}else if(ThisTime<4){
				if(!Step1){
				PutCog(true);
				Step1=true;
			}
				PutCog(false);
			}else if(ThisTime<4.5){
				if(!Step2){
					PutCog(true);
					Step2=true;
				}
				PutCog(false);
			}else if(ThisTime<5){
				Move(0.7,0);
			}else if(ThisTime<6){
				if(!Step3){
					PutCog(true);
					Step3=true;
				}
				PutCog(false);
				Move(0,0);
		}

		//Test
		/*
		if(ThisTime<3){
			Move(-0.8,0);
		}else{
			Move(0,0);
		}*/
	}

	void TeleopInit() override{
		ClimbState=0;
		start_time=-3;
		timer.Reset();
		timer.Start();
		CogFlag=false;
		CogState=2;
		TaskBut=0;
		ShootBallState=false;
	}
	//
	double Noise(double input,double min){
		return fabs(input)>=min?(fabs(input)-min)*(fabs(input)/input):0;
	}


	void AdjustCoe(){
		if(stick.GetRawButton(8)){
			P_COE=stick.GetThrottle()*0.1;
		}
		if(stick.GetRawButton(12)){
			D_COE=stick.GetThrottle()*0.1;
		}
		if(stick.GetRawButton(7)){
			O_COE=(stick.GetThrottle()+1)*0.5+1;
		}
		if(stick.GetRawButton(9)){
			RL_DIF=(stick.GetThrottle())*0.1;
		}
		if(stick.GetRawButton(11)){
			RL_RATIO=(stick.GetThrottle())*0.1+1;
		}
	}
	void PutCog(bool AutoActivate){

		if((TaskBut==0)||(TaskBut==PutCogBut)){
			if(NewBut(PutCogBut)||AutoActivate){
				CogState=(CogState+1)%3;
			}
			switch(CogState){
			case 0:
				TaskBut=PutCogBut;
				CogFlag=false;
				BotSol.Set(frc::DoubleSolenoid::kForward);
				break;
			case 1:
				UpSol.Set(frc::DoubleSolenoid::kForward);
				break;
			case 2:
				if(!CogFlag){
					UpSol.Set(frc::DoubleSolenoid::kReverse);
					BotSol.Set(frc::DoubleSolenoid::kReverse);
					Move(0,0);
					CogFlag=true;
					TaskBut=0;
					AutoMove=false;
				}
			}
		}
	}
	void ControlArm(){

		static double LeftTime=0;
		static int ArmState=2;
		static bool BotState=false;

		if(TaskBut==0||TaskBut==CollectCogBut){
			if(NewBut(CollectCogBut)) {
				ArmState=(ArmState+1)%3;
			}
			switch(ArmState){
				case 0:
					TaskBut=CollectCogBut;
					CogWheel.Set(0);
					BotSol.Set(frc::DoubleSolenoid::kForward);
					if(Sensor.Get()){
						BotState=true;
					}
					if(BotState){
						UpSol.Set(frc::DoubleSolenoid::kOff);
					}else{
						UpSol.Set(frc::DoubleSolenoid::kForward);
					}
					break;
				case 1:
					CogWheel.Set(0.45);
					BotSol.Set(frc::DoubleSolenoid::kForward);
					UpSol.Set(frc::DoubleSolenoid::kForward);
					BotState=false;
					LeftTime=timer.Get();
					break;
				case 2:
					BotSol.Set(frc::DoubleSolenoid::kReverse);
					BotState=false;
					if(timer.Get()>=(LeftTime+0.5)){
						UpSol.Set(frc::DoubleSolenoid::kReverse);
						CogWheel.Set(0);
						TaskBut=0;
					}
					break;
			}
		}

	}

	void Climb(){
		//LMag is a climbing motor here
		if(NewBut(ClimbBut)){
			ClimbState=(ClimbState+1)%3;
		}
		switch(ClimbState){
		case 0:
			LMag.Set(0);
			RMag.Set(0);
			break;
		case 1:
			LMag.Set(0.3);
			RMag.Set(0.3);
			break;
		case 2:
			LMag.Set(0.8);
			RMag.Set(0.8);
			break;
		}
	}
	bool NewBut(int but_num){
		return ThisBut[but_num]&&!LastBut[but_num];
	}
	void CollectBall(){

		if(stick.GetRawButton(1)){
			CollectBallWheel.Set(COLLECT_BALL_SPEED);
		}else{
			if(!ShootBallState){
				CollectBallWheel.Set(0);
			}
		}
	}
	void ShootBall(){
		const double ACC_TIME=2;
		const double ShootingSpeed=0.3;


		double SetValue=0;

		if(NewBut(ShootBallBut)){
			ShootBallState=!ShootBallState;
			start_time=timer.Get();
		}
		double PastTime=timer.Get()-start_time;

		if(ShootBallState){
			if(PastTime>=ACC_TIME){
				SetValue=ShootingSpeed;
				TransferBelt.Set(-0.3);
				BotTransferWheel.Set(0.2);
				TransferWheel.Set(-0.3);
				CollectBallWheel.Set(COLLECT_BALL_SPEED);
			}else{
				SetValue=(PastTime/ACC_TIME)*ShootingSpeed;
			}
		}else{
			if(PastTime>=ACC_TIME){
				SetValue=0;
			}else{
				SetValue=(1-PastTime/ACC_TIME)*ShootingSpeed;
				TransferWheel.Set(0);
				TransferBelt.Set(0);
				BotTransferWheel.Set(0);
			}
		}
		ShootingWheel1.Set(-SetValue);
		ShootingWheel2.Set(-SetValue);

	}
	void RotateAngle(){
		const double ROTATING_SPEED=0.2;
		double SetSpeed=0;
		int SetValue=-1;

		SetValue=stick.GetPOV();
		if(SetValue<180&&SetValue>0){
			SetSpeed=-ROTATING_SPEED;
		}else if(SetValue>180){
			SetSpeed=ROTATING_SPEED;
		}else{
			SetSpeed=0;
		}

		if(stick.GetRawButton(AutoAngleBut)){
			if(!AnglePin1.Get()){
				if(AnglePin3.Get()){
					//ShootBallState=true;
				}
			}else{
				SetSpeed=AnglePin3.Get()?ROTATING_SPEED:ROTATING_SPEED*0.4;
				SetSpeed*=AnglePin2.Get()?1:-1;
			}
		}

		AngleMotor.Set(SetSpeed);
	}

	void TeleopPeriodic() override{
		int i;
		for(i=1;i<12;i++){
			ThisBut[i]=stick.GetRawButton(i);
		}

		ControlArm();
		PutCog(false);
		//AdjustCoe();
		RotateAngle();
		CollectBall();
		ShootBall();
		Climb();
		if(!AutoMove){
			Move(Noise(stick.GetY()*fabs(stick.GetY()),0.15),Noise(fabs(stick.GetX())*stick.GetX(),0.2));
		}


		for(i=1;i<12;i++){
			LastBut[i]=ThisBut[i];
		}
		if (!ahrs)
			return;
		//Move(stick.GetThrottle(),0);

		int final=-1;
		final=stick.GetPOV();
		SmartDashboard::PutNumber("B_POV_Value",final);

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
		SmartDashboard::PutNumber("A_Roll", ahrs->GetRoll());
		SmartDashboard::PutNumber("A_Pitch", ahrs->GetPitch());
		SmartDashboard::PutNumber("A_Yaw", ahrs->GetYaw());
		SmartDashboard::PutNumber("JoysitckX", stick.GetX());
		SmartDashboard::PutNumber("JoystickY", stick.GetY());
		SmartDashboard::PutNumber("A_LMag",LMag.GetPosition());
		SmartDashboard::PutNumber("A_RMag",RMag.GetPosition());
		SmartDashboard::PutNumber("TaskBut",TaskBut);
		SmartDashboard::PutBoolean("AnglePin1", AnglePin1.Get());
		SmartDashboard::PutBoolean("AnglePin2", AnglePin2.Get());
		SmartDashboard::PutBoolean("AnglePin3", AnglePin3.Get());
		//frc::Wait(0.001);
	}
	void TestPeriodic() override{
		lw->Run();
	}
};

START_ROBOT_CLASS(Robot);

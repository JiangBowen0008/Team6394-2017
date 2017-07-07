#include <IterativeRobot.h>
#include <Joystick.h>
#include <Solenoid.h>
#include <Talon.h>
#include <DoubleSolenoid.h>
#include <LiveWindow/LiveWindow.h>
#include <RobotDrive.h>
#include <Timer.h>
#include <math.h>

//按钮设定
const int ButNum = 12;
bool LastButState[ButNum];//储存上次按钮
bool ThisButState[ButNum];//储存上次按钮
bool ButStateChange[ButNum];//按钮变化

enum DefineBut {
	CollectBallBut = 2,//捡球摩擦轮按钮
	CallibrateBut = 6,//射球自动校准按钮
	ShootBallBut = 4,//射球按钮
	CogSolenoidDownBut = 3,//齿轮收集按钮
	PutCogBut = 5;//挂齿轮按钮
FastModeBut = 10,//快速模式切换按钮
SlowModeBut = 9,//慢速模式切换按钮
FastClimbBut = 8,//快速爬升
SlowClimbBut = 7//慢速爬升
} ButSetting;

//——————连接设定

//CAN
enum DefineCANPin {
	BotCogSolenoidPin = 0//底部气动DIO0
	UpCogSolenoidPin = 1;//上部气动推DIO2
} CANSetting;

///DIO
enum DefineDIOPin {
	CogCollectedPin = 0,//齿轮收入微动开关DIO
	CogDistancePin = 2;//红外测距挡板
} DIOSetting;

///PWM
enum DefinePWMPin {
	RBaseMotorPin = 0,//PWM0底盘右轮
	LBaseMotorPin = 1,//PWM1底盘左轮
	CogFriWheelPin = 2,//齿轮摩擦轮PWM2
	CollectBallWheelPin = 3,//捡球摩擦轮PWM3
	SBTransferBeltPin = 4,//球传送带PWM4
	SBTransferWheelPin = 5,//传球小轮PWM5
	SBShootWheelPin = 6,//射球轮PWM6
	SBAngleMotorPin = 7,//射击角度电机PWM7
	ClimbingMotorPin = 8//爬升电机PWM8
} PWMSetting;

//——————常量设定
const double CogFriWheelSpeed = 1;//齿轮摩擦轮满速
const double SpeedLowLimit = 0.3;//车身速度下限
const double AngleCoe = 0.2;//角度调整P系数
const int AngleTolerate = 10;//角度调整容忍范围
const int AngleSliderCoe = 50;//角度调整拨片调整倍数
const double SlowClimbSpeed = 0.3;//慢速爬绳转速
const double FastClimbSpeed = 1;//快速爬绳转速
const double MeterConvert = 1.3;
const double AutoMovingSpeed = 0.8;//自动移动速度

								   //——————状态设定
bool BotSolenoidDown = false;
bool UpSolenoidDown = false;
bool CogCollected = false;
bool FastMode = true;
int AnglePosition = 0;
bool Calibrated = false;


//——————阶段函数
class Robot : public frc::IterativeRobot {
public:
	Robot() {
		myRobot.SetExpiration(0.1);
		timer.Start();
	}

private:
	frc::RobotDrive myRobot{ RightBaseMotorPin, LeftBaseMotorPin };  // Robot drive system
	frc::Joystick stick{ 0 };         // Only joystick
	frc::LiveWindow* lw = frc::LiveWindow::GetInstance();
	frc::Timer timer;

	frc::Solenoid BotCogSolenoid{ BotCogSolenoidPin };
	frc::Solenoid UpCogSolenoid{ UpCogSolenoidPin };

	frc::Talon CogFriWheel{ CogFriWheelPin };//齿轮摩擦轮
	frc::Talon BallFriWheel{ CollectBallWheelPin };//捡球摩擦轮
	frc::Talon BallTransferBelt{ SBTransferBeltPin };//球传送带
	frc::Talon BallTransferWheel{ SBTransferWheelPin };//传球小轮
	frc::Talon ShootWheel{ SBShootWheelPin };//射球电机
	frc::Talon AngleMotor{ SBAngleMotorPin };//角度调整电机
	frc::Talon ClimbingMotor{ ClimbingMotorPin };//爬绳电机

	void AutonomousInit() override {
		timer.Reset();
		timer.Start();
	}

	void AutonomousPeriodic() override {
		// Drive for 2 seconds
		if (timer.Get() < 2.0) {
			myRobot.Drive(-0.5, 0.0);  // Drive forwards half speed
		}
		else {
			myRobot.Drive(0.0, 0.0);  // Stop robot
		}
	}

	void TeleopInit() override {
		int i = 1;
		for{i = 1; i<ButNum; i++} {
			LastButState[i] = false;
		}
	}

	void TeleopPeriodic() override {
		// Drive with arcade style (use right stick)

		SensorUpdate();

		//控制
		myRobot.ArcadeDrive(stick);

		///收齿轮
		if (NewCommend(CogSolenoidDownBut)) {
			if (BotSolenoidDown) {
				LeftSolenoid();//已经降下来的情况选择上抬
			}
			else {
				PutDownSolenoid()；//未降下来的情况选择下降
			}
		}

		///挂齿轮
		if (ThisButState[PutCogBut]) {
			if (NewCommend(PutCogBut) && BotSolenoidDown) {
				LeftSolenoid();//已经降下来的情况选择上抬
			}
			else {
				PutCog()；//挂齿轮
			}
		}


		if (CogCollected&&SolenoidDown)LeftSolenoid();//齿轮接入后自动抬升
		if (ThisButState[FastModeBut])FastMode = true;//切换为快速模式
		if (ThisButState[SlowModeBut])FastMode = false;//切换为微调模式
		if (ThisButState[CallibrateBut])AngleMotor.Set(AngleCallibrate());//按住调整角度
		if (ThisButState[ShootBallBut])ShootBall();//射击
		if (ThisButState[FastClimbBut])CimbingMotor.Set(FastClimbSpeed);//快速爬绳
		if (ThisButState[SlowClimbBut])CimbingMotor.Set(SlowClimbSpeed);//慢速爬绳

	}
	bool NewCommend(int ButIndex) {
		//输出true如果是一个新的true信号
		return ((ThisButState[ButIndex]) && (!LastButState[ButIndex]));
	}
	void TestPeriodic() override {
		lw->Run();
	}
	//——————更新函数
	///总更新
	void SensorUpdate() {
		//此处更新传感器状态!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		//CogCollected=
		//SolenoidDown=
		MapUpdate();
		ReadFromRaspberryPi();
		for{i = 1; i<ButNum; i++} {
			ThisButState[i] = stick.GetRawButton(i);
			ButStateChange[i] = (ThisButState[i] != LastButState[i])
		}
	}

	///地图
	const int x = 0;
	const int y = 1;

	double Pos[] = { 0,0 };
	double ThisFacingAngle = 0;

	double CogPos[] = { 100,200 };
	double CogAngle = 0;
	double BallPos[] = { 300,200 };

	void MapUpdate() {
		double ThisAcc[] = { 0,0 };
		double ThisVel[] = { 0,0 };
		double ThisTime = timer.Get();
		TimeChange = ThisTime - LastTime;

		/*
		！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！
		此处对ThisAcc,ThisFacingAngle更新
		读取出来之后进行换算 使用MeterConvert常量
		即
		ThisAcc/=MeterConvert;
		未读取出来的话记默认0，0
		*/
		ThisVel[x] = LastVel[x] + TimeChange*(LastAcc[x] + ThisAcc[x]) / 2;
		Pos[x] += TimeChange*(ThisVel[x] + LastVel[x]) / 2;
		ThisVel[y] = LastVel[y] + TimeChange*(LastAcc[y] + ThisAcc[y]) / 2;
		Pos[y] += TimeChange*(ThisVel[y] + LastVel[y]) / 2;

		LastAcc[x] = ThisAcc[x];
		LastAcc[y] = ThisAcc[y];
		LastVel[x] = ThisVel[x];
		LastVel[y] = ThisVel[y];
		LastTime = ThisTime;
	}


	//——————动作函数

	///读传感器
	bool ReadDIO(int pin) {
		///返回true或者false
		return;
	}

	///降铲子
	void PutDownSolenoid() {
		//收集齿轮 启动两个气缸 转动摩擦轮
		BotCogSolenoid.Set(true);
		UpCogSolenoid.Set(true);
		CogFriWheel.Set(CogFriWheelSpeed);
		UpSolenoidDown = true;
		BotSolenoidDown = true;
	}

	//挂齿轮

	void PutCog() {
		//挂齿轮 仅启动底部气缸 关闭摩擦轮
		double MarchingSpeed = 0.4;
		if (ReadDIO(CogDistancePin) && (!BotSolenoidDown))
		{
			BotCogSolenoid.Set(true);
			UpCogSolenoid.Set(true);
			CogFriWheel.Set(0);
			BotSolenoidDown = true;
		}
		else {
			if (MoveToPos(CogPos[x], CogPos[y], CogAngle)) {
				myRobot.Drive(MarchingSpeed, 0);
			}
			else {
				CalculatePos();
				MoveToPos(CogPos[x], CogPos[y], CogAngle, AutoMovingSpeed);
			}
		}

	}
	void CalculatePos() {
		//齿轮挂钩计算
		double direct_dis;
		double image_dis;
		double Standard_dis = 0.01;
		double Hook_dis = 0.5;

		ReadFromRaspberryPi();
		//读取image_dis;
		image_dis = image_dis*direct_dis*Standard_dis;
		CogPos[x] = Pos[x] + direct_dis*cos(ThisFacingAngle) + image_dis*sin(ThisFacingAngle);
		CogPos[y] = Pos[y] + direct_dis*sin(ThisFacingAngle) + image_dis*cos(ThisFacingAngle);
	}
	bool MoveToPos(double XPos, double YPos, double FinalFacingAngle, double MovingSpeed) {
		double TargetAngle;
		double x_dif;
		double y_dif;
		double angletol = 0.04;
		double angleact = 0.5;
		double P_angle = 0.9;//角度调整P系数
		double distol = 0.01;


		if ((InRange(XPos, distol, Pos[x]) && (InRange(YPos, distol, Pos[y])) {
			if (InRange(ThisFacingAngle, angletol, FinalFacingAngle)) {
				return true;
			}
			else {
				myRobot.Drive(0, -(FinalFacingAngle - ThisFacingAngle)*P_angle)
			}

		}
		else {
			x_dif = Pos[x] - XPos;
			y_dif = Pos[y] - YPos;
			TargetAngle = atan(x_dif / y_dif);
			if (InRange(ThisFacingAngle, angleact, TargetAngle)) {
				myRobot.Drive(MovingSpeed, -(TargetAngle - ThisFacingAngle)*P)
			}
			else {
				myRobot.Drive(0, -(TargetAngle - ThisFacingAngle)*P)
			}
		}

		return false;
	}
	bool InRange(double input, double tolerate, double target) {
		return ((input >= target - tolerate) && (input <= target + tolerate));
	}

	///抬铲子
	void LeftSolenoid() {
		BotCogSolenoid.Set(false);
		UpCogSolenoid.Set(false);
		CogFriWheel.Set(0);
		BotSolenoidDown = false;
		UpSolenoidDown = false;
	}

	///图像算法
	double err = 0;

	void ReadFromRaspberryPi() {
		err = round(rand() * 100);//临时使用随机误差，最终应读取树莓派图像数据!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	}

	int AngleCallibrate() {
		//读取误差

		err += AngleSliderCoe*(stick.GetZ() - 0.5);
		if (abs(err)>AngleTolerate) {
			AnglePosition -= err*AngleCoe;
		}
		else {
			Calibrated = true;
		}
		return AnglePosition;
	}

	///射球
	void ShootBall(double speed) {
		double ShootingSpeed;
		BallTransferBelt.Set(speed);
		BallTransferWheel.Set(speed);
		ShootWheel.Set(CalculateShootingSpeed());
	}

	double CalculateShootingSpeed() {
		//简略运算
		double distance = 0;
		double height = 3.4;
		double result = 0;
		double AirFriCoe = 0.02;
		double a = 2;
		double g = 9.8;

		distance = sqrt(pow((BallPos[x] - Pos[x]), 2) + pow((BallPos[y] - Pos[y]), 2));
		result = pow(distance, 2)*AirFriCoe + sqrt(2 * pow(cos(a), 2) / ((g*pow(distance, 2))*(tan(a)*distance - height));
		return result;
	}



};

START_ROBOT_CLASS(Robot)

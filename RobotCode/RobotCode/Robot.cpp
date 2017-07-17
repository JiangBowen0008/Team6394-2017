#include "WPILib.h"
#include "AHRS.h"
#include <math.h>

//按钮设定
const int BUT_NUMBER = 12;
bool LastButState[BUT_NUMBER]; //储存上次按钮
bool ThisButState[BUT_NUMBER]; //储存上次按钮

enum DefineBut {
	CollectBallBut = 2, //捡球摩擦轮按钮
	CallibrateBut = 6, //射球自动校准按钮
	ShootBallBut = 4, //射球按钮
	CogSolenoidDownBut = 3, //齿轮收集按钮
	PutCogBut = 5, //挂齿轮按钮
	FastModeBut = 10, //快速模式切换按钮
	SlowModeBut = 9, //慢速模式切换按钮
	FastClimbBut = 8, //快速爬升
	SlowClimbBut = 7 //慢速爬升
} ButSetting;

enum DefineCANPin {
	BotCogSolenoidPin = 0, //底部气动DIO0
	UpCogSolenoidPin = 1 //上部气动推DIO2
} CANSetting;

///DIO
enum DefineDIOPin {
	CogCollectedPin = 0, //齿轮收入微动开关DIO
	CogDistancePin = 2 //红外测距挡板
} DIOSetting;

///PWM
enum DefinePWMPin {
	RightBaseMotorPin = 0, //PWM0底盘右轮
	LeftBaseMotorPin = 1, //PWM1底盘左轮
	CogFriWheelPin = 2, //齿轮摩擦轮PWM2
	CollectBallWheelPin = 3, //捡球摩擦轮PWM3
	SBTransferBeltPin = 4, //球传送带PWM4
	SBTransferWheelPin = 5, //传球小轮PWM5
	SBShootWheelPin = 6, //射球轮PWM6
	SBAngleMotorPin = 7, //射击角度电机PWM7
	ClimbingMotorPin = 8 //爬升电机PWM8
} PWMSetting;


//——————常量设定
const double CogFriWheelSpeed = 1; //齿轮摩擦轮满速
const double SpeedLowLimit = 0.3; //车身速度下限
const double AngleCoe = 0.2; //角度调整P系数
const int AngleTolerate = 10; //角度调整容忍范围
const int AngleSliderCoe = 50; //角度调整拨片调整倍数
const double SlowClimbSpeed = 0.3; //慢速爬绳转速
const double FastClimbSpeed = 1; //快速爬绳转速
const double MeterConvert = 1.3;
const double DefaultShootingSpeed = 1;
const double MAX_BASE_SPEED=1;
const double MIN_BASE_SPEED=0.4;

///地图
const int x = 0;
const int y = 1;


//——————状态设定
bool BotSolenoidDown = false;
bool UpSolenoidDown = false;
bool CogCollected = false;
int AnglePosition = 0;
double BaseSpeed=MAX_BASE_SPEED;
bool Calibrated = false;


///电机控制器注册


//——————自动移动参数
double AutoMovingSpeed=0.5;
double CogPos[2]={0,0};
double CogAngle=0;
double BallPos[2]={0,0};

double angletol=0.04;
double angleact=0.5;
double P_angle=0.9;//角度调整P系数
double distol=0.01;

double NowX=0;
double NowY=0;
double NowAng=0;

//——————阶段函数
class Robot: public frc::IterativeRobot {

public:
	AHRS *ahrs;
	std::shared_ptr<NetworkTable> table;

	//frc::RobotDrive myRobot { RightBaseMotorPin, LeftBaseMotorPin }; // Robot drive system
	frc::Talon RMotor{RightBaseMotorPin};
	frc::Talon LMotor{LeftBaseMotorPin};

	frc::Joystick stick { 0 }; // Only joystick
	frc::LiveWindow* lw = frc::LiveWindow::GetInstance();
	frc::Timer timer;
	int autoLoopCounter;
	Robot() :
			table(NULL), stick(0), ahrs(NULL), lw(NULL), autoLoopCounter(0)
	{
		//myRobot.SetExpiration(0.1);
		timer.Start();
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
private:
	frc::Solenoid BotCogSolenoid { BotCogSolenoidPin };
	frc::Solenoid UpCogSolenoid { UpCogSolenoidPin };

	frc::Talon CogFriWheel { CogFriWheelPin };         //齿轮摩擦轮
	frc::Talon BallFriWheel { CollectBallWheelPin };         //捡球摩擦轮
	frc::Talon BallTransferBelt { SBTransferBeltPin };         //球传送带
	frc::Talon BallTransferWheel { SBTransferWheelPin };         //传球小轮
	frc::Talon ShootWheel { SBShootWheelPin };         //射球电机
	frc::Talon AngleMotor { SBAngleMotorPin };         //角度调整电机
	frc::Talon ClimbingMotor { ClimbingMotorPin };         //爬绳电机

	void AutonomousInit() override {
		timer.Reset();
		timer.Start();
	}

	void AutonomousPeriodic() override {
		// Drive for 2 seconds
		if (timer.Get() < 2.0) {
			Move(-0.5, 0.0);  // Drive forwards half speed
		} else {
			Move(0.0, 0.0);  // Stop robot
		}
	}

	void TeleopInit() override {
		int i = 1;
		for (i = 1; i < BUT_NUMBER; i++) {
			LastButState[i] = false;
		}
	}

	void TeleopPeriodic() override {

		int i=0;

		SensorUpdate();
		//控制

		Move((stick.GetRawAxis(1)-0.5)*2*BaseSpeed,(stick.GetRawAxis(0)-0.5)*2*BaseSpeed);

		///收齿轮
		if (NewCommend(CogSolenoidDownBut)) {
			if (BotSolenoidDown) {
				LeftSolenoid();  //已经降下来的情况选择上抬
			} else {
				PutDownSolenoid();  //未降下来的情况选择下降
			}
		}
		///挂齿轮
		if (ThisButState[PutCogBut]) {
			if (NewCommend(PutCogBut) && BotSolenoidDown) {
				LeftSolenoid();  //已经降下来的情况选择上抬
			} else {
				PutCog();  //挂齿轮
			}
		}

		if (CogCollected && BotSolenoidDown)
			LeftSolenoid();  //齿轮接入后自动抬升
		if (ThisButState[FastModeBut])
			BaseSpeed = MAX_BASE_SPEED;  //切换为快速模式
		if (ThisButState[SlowModeBut])
			BaseSpeed = MIN_BASE_SPEED;  //切换为微调模式
		if (ThisButState[CallibrateBut])
			AngleMotor.Set(AngleCallibrate());  //按住调整角度
		if (ThisButState[ShootBallBut])
			ShootBall(DefaultShootingSpeed);  //射击
		if (ThisButState[FastClimbBut])
			ClimbingMotor.Set(FastClimbSpeed);  //快速爬绳
		if (ThisButState[SlowClimbBut])
			ClimbingMotor.Set(SlowClimbSpeed);  //慢速爬绳


		for (i = 1; i < BUT_NUMBER; i++) {
			LastButState[i] = ThisButState[i];
		}
	}

	double err = 0;
	void ReadFromRaspberryPi() {
		err = round(rand() * 100);
		//临时使用随机误差，最终应读取树莓派图像数据!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	}
	bool NewCommend(int ButIndex) {
		//输出true如果是一个新的true信号
		return ((ThisButState[ButIndex]) && (!LastButState[ButIndex]));
	}
///抬铲子
	void LeftSolenoid() {
		BotCogSolenoid.Set(false);
		UpCogSolenoid.Set(false);
		CogFriWheel.Set(0);
		BotSolenoidDown = false;
		UpSolenoidDown = false;
	}

	void SensorUpdate() {
		//此处更新传感器状态!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		ReadFromRaspberryPi();

   		NowX=ahrs->GetDisplacementZ();
   		NowY=ahrs->GetDisplacementX();
   		NowAng=ahrs->GetRoll();

		int i;
		for (i = 1; i < BUT_NUMBER; i++) {
			ThisButState[i] = stick.GetRawButton(i);
		}
	}
	bool ReadDIO(int pin) {
		///返回true或者false!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		//return;
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
		if (ReadDIO(CogDistancePin) && (!BotSolenoidDown)) {
			BotCogSolenoid.Set(true);
			UpCogSolenoid.Set(true);
			CogFriWheel.Set(0);
			BotSolenoidDown = true;
		}
		else {
			//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
			if (MoveToPos(CogPos[x], CogPos[y], CogAngle,AutoMovingSpeed)) {
				Move(MarchingSpeed, 0);
			} else {
				CalculatePos();
				MoveToPos(CogPos[x], CogPos[y], CogAngle, AutoMovingSpeed);
			}
		}

	}
    void Move(double forward, double Rturn){
    	double tmp,Fcoe,Rcoe;
    	tmp=abs(forward)+abs(Rturn);
    	Fcoe=abs(forward)/tmp;
    	Rcoe=abs(Rturn)/tmp;

    	RMotor.Set(forward*Fcoe-Rturn*Rcoe);
    	LMotor.Set(forward*Fcoe+Rturn*Rcoe);
    }

    bool MoveToPos(double XPos,double YPos, double FinalFacingAngle,double MovingSpeed){
     	double TargetAngle=0;
      	double x_dif=0;
   		double y_dif=0;

      	if(InRange(XPos,distol,NowX)&&(InRange(YPos,distol,NowY))){
        	if(InRange(NowAng,angletol,FinalFacingAngle)){
    			Move(0,0);
        		return true;
       		}else{
   				Move(0,-(FinalFacingAngle-NowAng)*P_angle);
       		}
        }else{
   			x_dif=NowX-XPos;
   			y_dif=NowY-YPos;
   			TargetAngle=atan(x_dif/y_dif);
        	if(InRange(NowAng,angleact,TargetAngle)){
        		Move(MovingSpeed,-(TargetAngle-NowAng)*P_angle);
        	}else{
        		Move(0.0,-(TargetAngle-NowAng)*P_angle);
        	}
        }
    	return false;
    }

	bool InRange(double input, double tolerate, double target) {
		return ((input >= target - tolerate) && (input <= target + tolerate));
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
		CogPos[x] = NowX + direct_dis*cos(NowAng) + image_dis*sin(NowAng);
		CogPos[y] = NowY + direct_dis*sin(NowAng) + image_dis*cos(NowAng);
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

				distance = sqrt(pow((BallPos[x] - ahrs->GetDisplacementX()), 2) + pow((BallPos[y] - ahrs->GetDisplacementY()), 2));
				result = pow(distance, 2)*AirFriCoe + sqrt(2 * pow(cos(a), 2)) / ((g*pow(distance, 2))*(tan(a)*distance - height));
				return result;
			}

};

START_ROBOT_CLASS(Robot)

#include "WPILib.h"
#include "AHRS.h"

/**
 * This is a demo program providing a real-time display of navX
 * MXP values.
 *
 * In the operatorControl() method, all data from the navX sensor is retrieved
 * and output to the SmartDashboard.
 *
 * The output data values include:
 *
 * - Yaw, Pitch and Roll angles
 * - Compass Heading and 9-Axis Fused Heading (requires Magnetometer calibration)
 * - Linear Acceleration Data
 * - Motion Indicators
 * - Estimated Velocity and Displacement
 * - Quaternion Data
 * - Raw Gyro, Accelerometer and Magnetometer Data
 *
 * As well, Board Information is also retrieved; this can be useful for debugging
 * connectivity issues after initial installation of the navX MXP sensor.
 *
 */
class Robot: public IterativeRobot
{
	std::shared_ptr<NetworkTable> table;
	Joystick stick; // only joystick
    AHRS *ahrs;
    LiveWindow *lw;
    int autoLoopCounter;

public:
    Robot() :
        table(NULL),
        stick(0),		// as they are declared above.
        ahrs(NULL),
        lw(NULL),
        autoLoopCounter(0)
    {
    }

private:
    frc::Talon RMotor{0};
    frc::Talon LMotor{1};

    double AutoMovingSpeed=0.5;
    double CogPos[2]={0,0};
    double CogAngle=0;
    double BallPos[2]={0,0};

    double angletol=0.04;
    double angleact=0.5;
    double P_angle=0.9;//角度调整P系数
    double distol=0.01;

    void Move(double forward, double Rturn){
    	double tmp,Fcoe,Rcoe;
    	tmp=abs(forward)+abs(Rturn);
    	Fcoe=abs(forward)/tmp;
    	Rcoe=abs(Rturn)/tmp;

    	RMotor.Set(forward*Fcoe-Rturn*Rcoe);
    	LMotor.Set(forward*Fcoe+Rturn*Rcoe);
    }

    bool InRange(double input, double tolerate, double target){
        return ((input>=target-tolerate)&&(input<=target+tolerate));
    }


    bool MoveToPos(double XPos,double YPos, double FinalFacingAngle,double MovingSpeed){
     	double TargetAngle=0;
      	double x_dif=0;
   		double y_dif=0;
   		double NowX=0;
   		double NowY=0;
   		double NowAng=0;

   		NowX=ahrs->GetDisplacementZ();
   		NowY=ahrs->GetDisplacementX();
   		NowAng=ahrs->GetRoll();

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

    void RobotInit() override
    {
        table = NetworkTable::GetTable("datatable");
        lw = LiveWindow::GetInstance();
        try {
            ahrs = new AHRS(SPI::Port::kMXP);
        } catch (std::exception& ex ) {
            std::string err_string = "Error instantiating navX MXP:  ";
            err_string += ex.what();
            DriverStation::ReportError(err_string.c_str());
        }
        if ( ahrs ) {
            LiveWindow::GetInstance()->AddSensor("IMU", "Gyro", ahrs);
        }
	}

    void AutonomousInit() override
    {
        autoLoopCounter = 0;
    }

    void AutonomousPeriodic() override
    {
        if(autoLoopCounter < 100) //Check if we've completed 100 loops (approximately 2 seconds)
        {
            autoLoopCounter++;
        }
    }

    void TeleopInit() override
    {

    }

    void TeleopPeriodic() override
    {
        if ( !ahrs ) return;
        Move(stick.GetRawAxis(1),stick.GetRawAxis(0));
        if(stick.GetRawButton(2)){
        	MoveToPos(0,0,0,AutoMovingSpeed);
        }

        SmartDashboard::PutBoolean( "IMU_Connected",        ahrs->IsConnected());

        SmartDashboard::PutNumber(  "Velocity_X",           ahrs->GetVelocityX() );
        SmartDashboard::PutNumber(  "Velocity_Y",           ahrs->GetVelocityY() );
        SmartDashboard::PutNumber(  "Displacement_X",       ahrs->GetDisplacementX() );
        SmartDashboard::PutNumber(  "Displacement_Y",       ahrs->GetDisplacementY() );

    }

    void TestPeriodic() override
    {
        lw->Run();
    }
};

START_ROBOT_CLASS(Robot);

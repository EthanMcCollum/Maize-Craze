/* * * * * * * * * * * * * * * Program Description * * * * * * * * * * * * * * *\
 * This program controls a robot for the FRC 1992 inaugral game 'Maize Craze'. *
 * The imaginary robot is an 8-wheel tank drive with an L-shaped arm attached  *
 * to a motor on the right side.  The arm's purpose is to knock tennis balls   *
 * off of the posts in the corner of the field.  Since the robot must fit in a *
 * 38cm x 50cm x 34cm box, the arm will have to start parallel to the ground   *
 * along the bot's side and then extend up when the match begins.  The game    *
 * does not have an autonomous period, so this will have to be done manually   *
 * by the driver.  The robot also has a static dozer mechanism on the front to *
 * manipulate balls on the ground.  Robot specs are unknown, so all values are *
 * hypothetical.  However, I tried to stay within the realm of reason, so the  *
 * numbers I have for, say, encoder ticks should at least make some sense.     *
\* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "WPILib.h"

#define SRXP 0.85
#define SRXI 0.004
#define SRXD 0.0192

class Robot: public IterativeRobot
{
private:
	LiveWindow *lw;

	Joystick *stick;
	CANTalon *frontLeft;
	CANTalon *backLeft;
	CANTalon *frontRight;
	CANTalon *backRight;
	CANTalon *towerMotor;
	DoubleSolenoid *gearShift;

	// Declare the bools for each instance of the toggle function.
	bool *moveArmOutput;
	bool *changeGearOutput;

	void RobotInit()
	{
		lw = LiveWindow::GetInstance();
		frontLeft = new CANTalon(1);
		backLeft = new CANTalon(2);
		frontRight = new CANTalon(3);
		backRight = new CANTalon(4);
		towerMotor = new CANTalon(5);
		stick = new Joystick(1);

		moveArmOutput = new bool;
		*moveArmOutput = false;
		changeGearOutput = new bool;
		*changeGearOutput = false;

		gearShift = new DoubleSolenoid(1, 1, 2);
	}

	void AutonomousInit()
	{
		ConfigTalon(frontLeft, false, SRXP, SRXI, SRXD);
		ConfigTalon(frontRight, false, SRXP, SRXI, SRXD);
		ConfigTalon(backLeft, false, SRXP, SRXI, SRXD);
		ConfigTalon(backRight, false, SRXP, SRXI, SRXD);
		ConfigTalon(towerMotor, false, SRXP, SRXI, SRXD);

		gearShift->Set(DoubleSolenoid::kReverse);  // Assuming this puts the bot into low gear.
		frontLeft->SetPosition(0);
		frontRight->SetPosition(0);
		backLeft->SetPosition(0);
		backRight->SetPosition(0);
		towerMotor->SetPosition(0);
	}

	void AutonomousPeriodic()
	{
		// Technically there is no auto mode, but I set it up to hold everything still for safety purposes.  That should happen
		// despite my code, but hey, you never know.
		frontLeft->Set(0);
		frontRight->Set(0);
		backLeft->Set(0);
		backRight->Set(0);
		towerMotor->Set(0);
	}

	void TeleopInit()
	{
		ConfigTalon(frontLeft, true, SRXP, SRXI, SRXD);
		ConfigTalon(frontRight, true, SRXP, SRXI, SRXD);
		ConfigTalon(backLeft, true, SRXP, SRXI, SRXD);
		ConfigTalon(backRight, true, SRXP, SRXI, SRXD);
		ConfigTalon(towerMotor, false, SRXP, SRXI, SRXD);
	}

	void TeleopPeriodic()
	{

		float xAxis = Deadband(stick->GetX(), 0.2);
		float yAxis = Deadband(stick->GetY(), 0.2);
		float zAxis = Deadband(stick->GetZ(), 0.2);
		CustomArcade(xAxis, yAxis, zAxis);

		// This section here resets the I error value in the PID loops controlling the drive motors when yAxis and zAxis inputs are
		// both 0. This stops driving from getting finicky over the course of the match.
		if(yAxis == 0.0 && zAxis == 0.0)
		{
			frontLeft->ClearIaccum();
			frontRight->ClearIaccum();
			backLeft->ClearIaccum();
			backRight->ClearIaccum();
		}

		bool armUp = Toggle(stick->GetRawButton(1), moveArmOutput);
		MoveArm(armUp);
		
		bool highGear = Toggle(stick->GetRawButton(1), changeGearOutput);
		ChangeGear(gearShift, highGear);
	}

	void TestPeriodic()
	{
		lw->Run();
	}

	/**
	 * Custom arcade drive code. It takes input from a single joystick. The code determines which axis has the highest absolute value and then feeds it into
	 * the motors. If the x-axis has a higher absolute value, we check to see whether the y-axis or z-axis is greater.
	 */
	void CustomArcade(float xAxis, float yAxis, float zAxis)  // xAxis, yAxis, and zAxis are the floating-point input values from the corresponding joystick axes.
	{
		// Power to be supplied to the left and right motors.
		float leftPower = 0;
		float rightPower = 0;

		// The previous power values for the motors. These are used for the ramping function.
		static float *previousFrontLeft = 0;
		static float *previousBackLeft = 0;
		static float *previousFrontRight = 0;
		static float *previousBackRight = 0;

		if(fabs(yAxis) >= fabs(xAxis) && fabs(yAxis) >= fabs(zAxis))
		{
			leftPower = yAxis;
			rightPower = yAxis;
		}
		else if(fabs(xAxis) > fabs(yAxis) && fabs(xAxis) > fabs(zAxis))
		{
			if(fabs(yAxis) >= fabs(zAxis))
			{
				leftPower = yAxis;
				rightPower = yAxis;
			}
			else
			{
				leftPower = zAxis * -1.0;
				rightPower = zAxis;
			}
		}
		else if(fabs(zAxis) > fabs(yAxis) && fabs(zAxis) >= fabs(xAxis))
		{
			leftPower = zAxis * -1.0;
			rightPower = zAxis;
		}
		// This code is totally fabs.

		// Pass power values to the motors.  Also, use the Ramp function for smoother drive train acceleration.
		frontLeft->Set(Ramp(leftPower, previousFrontLeft));
		backLeft->Set(Ramp(leftPower, previousBackLeft));
		frontRight->Set(Ramp(rightPower, previousFrontRight));
		backRight->Set(Ramp(rightPower, previousBackRight));
	}

	/**
	 * Controls the movement of the robot's arm. armUp is a toggleable bool based on button1 input. If it is true, the arm will rise high
	 * enough to knock the tennis ball off the tallest tower. Otherwise it stays down. You could modify this code to move the arm to
	 * varying positions so that it can hit balls on other towers as well. Note that the arm begins the match down.
	 */
	void MoveArm(bool armUp)
	{
		if(armUp == true)
		{
			towerMotor->Set(11000);
		}
		else
		{
			towerMotor->Set(0);
		}
	}

	/**
	 * Toggles a bool between true and false. inputButton is the true/false state of a button on the joystick, and *output is the output of
	 * the function. It is a parameter because the previous output must be known in order to switch to the opposite output the next time the
	 * button is pushed. You can initialize the pointer to either false or true, but you will need a separate pointer for all your different
	 * uses of the function.
	 */
	bool Toggle(bool inputButton, bool *output)
	{
		if (inputButton == true)
		{
			*output = !*output;
			return *output;
		}
		else
		{
			return *output;
		}
	}

	/**
	 * Function to cutoff low values from the joystick. inputValue is the input from the joystick axis. lowerLimit is the minimum allowed value,
	 * and any input lower in it will result in an output of 0. This deadband function also squares the inputs.
	 */
	float Deadband(float inputValue, float lowerLimit)
	{
		float outputValue = 0;

		if(fabs(inputValue) < fabs(lowerLimit))
		{
			outputValue = 0;
		}
		else
		{
			int sign;
			if(inputValue < 0)
			{
				sign = -1;
			}
			else
			{
				sign = 1;
			}

			outputValue = inputValue * inputValue * sign;
		}

		return outputValue;
	}

	/**
	 * This switches between high and low gear. gearShifter is the solenoid used for shifting gears. highGear is a bool that when true should shift
	 * the drive train into high gear, and when false shift the drive train into low gear. If it does the opposite, just switch kForward and kReverse.
	 */
	void ChangeGear(DoubleSolenoid *gearShifter, bool highGear)
	{
		if(highGear == true)
		{
			gearShifter->Set(DoubleSolenoid::kForward);
		}
		else
		{
			gearShifter->Set(DoubleSolenoid::kReverse);
		}
	}

	/**
	 * A function to cleanly configure CANTalons. *motor is the CANTalon you wish to configure. controlModeVelocity is a bool that when true will
	 * tell the program to control the motor in velocity mode, and when false will tell the program to control it in position mode. P, I, and D are
	 * the PID constants. They will automatically initialize to 0 (and therefore not effect anything) if you do not pass an argument for them.
	 */
	void ConfigTalon(CANTalon *motor, bool controlModeVelocity, float P = 0, float I = 0, float D = 0)
	{
		motor->SetPID(P, I, D);
		if (controlModeVelocity)
		{
			motor->SetControlMode(CANTalon::ControlMode::kSpeed);
		}
		else
		{
			motor->SetControlMode(CANTalon::ControlMode::kPosition);
		}
		motor->EnableControl();
	}

	/**
	 * Ramping algorithm to reduce acceleration and therefore protect gearboxes. currentInput is the currentInput from the joystick.
	 * rampConstant effects the intensity of acceleration; a higher ramping constant means lower acceleration, and vice versa.
	 * The default rampingConstant is 0.7.
	 */
	float Ramp(float currentInput, float* previousInput, float rampConstant = 0.7)
	{
		float rampedPower = rampConstant * currentInput + rampConstant * *previousInput;
		*previousInput = currentInput;
		return rampedPower;
	}
};

START_ROBOT_CLASS(Robot);

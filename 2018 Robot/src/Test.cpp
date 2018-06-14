#include "Robot.h"

void Robot::TestPeriodic()
{
	AutonomousTest();
}

void Robot::TestInit()
{
	SmartDashboard::PutNumber("Elevator Setpoint", 0);
	SmartDashboard::PutBoolean("Reset Elevator Encoder?", false);
	SmartDashboard::PutNumber("Elevator P", 0.03);
	SmartDashboard::PutNumber("Elevator Constant", 0.5);
	SmartDashboard::PutNumber("Elevator Limit", 0.8);

	StopCurrentProcesses();

	if(SmartDashboard::GetBoolean("Test Angle", 0))
	{
		TurnAngleTest(0);
	}
	else if(SmartDashboard::GetBoolean("Test Maintain", 0))
	{
		MaintainHeadingTest();
	}
	else if(SmartDashboard::GetBoolean("Test Distance", 0))
	{
		DriveDistanceTest(0);
	}
}

void Robot::TeleopTest()
{
	if(SmartDashboard::GetBoolean("Drive", 0)) DriveTest();
	if(SmartDashboard::GetBoolean("Full Elevator", 0))
	{
		FullElevatorTest();
		SmartDashboard::PutBoolean("PID Elevator", 0);
		SmartDashboard::PutBoolean("Manual Elevator", 0);
		SmartDashboard::PutNumber("Elevator Height", ElevatorPID.PIDGet());
	}
	else if(SmartDashboard::GetBoolean("Full Elevator", 0))
	{
		ManualElevatorTest();
		SmartDashboard::PutBoolean("PID Elevator", 0);
		SmartDashboard::PutBoolean("Full Elevator", 0);
		SmartDashboard::PutNumber("Elevator Height", ElevatorPID.PIDGet());
	}
	else if(SmartDashboard::GetBoolean("PID Elevator", 0))
	{
		PIDElevatorTest();
		SmartDashboard::PutBoolean("PID Elevator", 0);
		SmartDashboard::PutBoolean("Manual Elevator", 0);
		SmartDashboard::PutNumber("Elevator Height", ElevatorPID.PIDGet());
	}
	if(SmartDashboard::GetBoolean("Manual Elevator", 0)) ManualElevatorTest();
	if(SmartDashboard::GetBoolean("PID Elevator", 0)) PIDElevatorTest();
	if(SmartDashboard::GetBoolean("Intake", 0)) IntakeTest();
}

void Robot::AutonomousTest()
{
	//	SmartDashboard::PutNumber("Auto Pos Val", (int) AutoLocationChooser->GetSelected());
	//	SmartDashboard::PutNumber("Auto Obj Val", (int) AutoObjectiveChooser->GetSelected());

	//Display Data
	SmartDashboard::PutNumber("Angle Sensor", AngleSensors.GetAngle());
	SmartDashboard::PutNumber("Encoder R", DistancePID.PIDGet());


	SmartDashboard::PutNumber("Elevator Height", ElevatorPID.PIDGet());
	if(SmartDashboard::GetBoolean("Reset Elevator Encoder?", false)) {
		RightElevatorMotor.SetSelectedSensorPosition(0, consts::PID_LOOP_ID, consts::TALON_TIMEOUT_MS);
		SmartDashboard::PutBoolean("Reset Elevator Encoder?", false);
	}

	if(!SmartDashboard::GetBoolean("Go Forward, Turn Right", 0))
	{
		//Reset Angle Button
		if(SmartDashboard::GetBoolean("Reset Angle", 0))
		{
			AngleSensors.Reset();
			SmartDashboard::PutBoolean("Reset Angle", 0);
		}
		//Reset Encoder Button
		if(SmartDashboard::GetBoolean("Reset Encoders", 0))
		{
			ResetDriveEncoders();
			SmartDashboard::PutBoolean("Reset Encoders", 0);
		}

		//Maintain Angle Test Buttons
		if(SmartDashboard::GetBoolean("Toggle Maintain Test", 0))
		{
			//Toggle the two buttons
			SmartDashboard::PutBoolean("Enable Test Distance Output",
					SmartDashboard::GetBoolean("Enable Test Distance Output", 0) ^ 1);
			SmartDashboard::PutBoolean("Enable Maintain Controller",
					SmartDashboard::GetBoolean("Enable Maintain Controller", 0) ^ 1);

			SmartDashboard::PutBoolean("Toggle Maintain Test", 0);
		}
		if(SmartDashboard::GetBoolean("Enable Test Distance Output", 0))
		{
			AnglePIDOut.SetTestDistOutput(SmartDashboard::GetNumber(
					"Test Maintain Output", 0));
		}
		else
		{
			AnglePIDOut.SetTestDistOutput(0);
		}
		if(SmartDashboard::GetBoolean("Enable Maintain Controller", 0))
		{
			MaintainAngleController.Enable();
		}
		else
		{
			if(MaintainAngleController.IsEnabled()) MaintainAngleController.Disable();
		}
	}
	if(SmartDashboard::GetBoolean("Test Distance", 0))
	{
		if(SmartDashboard::GetBoolean("Toggle Distance Test", 0))
		{
			SmartDashboard::PutBoolean("Enable Maintain Controller", 1);
			MaintainAngleController.Enable();
			DistanceController.Enable();
		}
		else
		{
			SmartDashboard::PutBoolean("Enable Maintain Controller", 0);
			DistanceController.Disable();
			MaintainAngleController.Disable();
		}
	}
	else if(SmartDashboard::GetBoolean("Test Auto Elevator", 0))
	{
		AutoElevatorTest();
	}
	else
	{
//		DriveDistance(148);
//		TurnAngle(90);
//		SmartDashboard::PutBoolean("Go Forward, Turn Right", 0);
	}

	SmartDashboard::PutBoolean("DistancePID OnTarget", DistanceController.OnTarget());
	SmartDashboard::PutBoolean("MaintainAnglePID OnTarget", MaintainAngleController.OnTarget());
	SmartDashboard::PutBoolean("AnglePID OnTarget", AngleController.OnTarget());
}

void Robot::MaintainHeadingTest()
{
	//Disable other controllers
	AngleController.Disable();
	DistanceController.Disable();

	//Zeroing the angle sensor
	AngleSensors.Reset();

	//Enable test dist output
	AnglePIDOut.SetTestDistOutput(0.35);

	//Remove the pointers since only one PID is being used
	DistancePID.SetAnglePID(nullptr);
	AnglePIDOut.SetDistancePID(nullptr);

	//Configure the PID controller to make sure the robot drives straight with the NavX
	MaintainAngleController.Reset();
	MaintainAngleController.SetSetpoint(0);

	MaintainAngleController.Enable();
}

void Robot::DriveDistanceTest(double distance)
{
	//Disable other controllers
	AngleController.Disable();

	//Zeroing the angle sensor and encoders
	ResetDriveEncoders();
	AngleSensors.Reset();

	//Disable test dist output for angle
	AnglePIDOut.SetTestDistOutput(0);
	//Make sure the PID objects know about each other to avoid conflicts
	DistancePID.SetAnglePID(&AnglePIDOut);
	AnglePIDOut.SetDistancePID(&DistancePID);

	//Configure the PID controller to make sure the robot drives straight with the NavX
	MaintainAngleController.Reset();
	MaintainAngleController.SetSetpoint(0);

	//Configure the robot to drive a given distance
	DistanceController.Reset();
	DistanceController.SetSetpoint(distance);

	MaintainAngleController.Enable();
	DistanceController.Enable();
}


void Robot::TurnAngleTest(double angle)
{
	//Disable other controllers
	DistanceController.Disable();
	MaintainAngleController.Disable();

	//Zeroing the angle sensor
	AngleSensors.Reset();

	//Disable test dist output for angle
	AnglePIDOut.SetTestDistOutput(0);

	//Remove the pointers since only one PID is being used
	DistancePID.SetAnglePID(nullptr);
	AnglePIDOut.SetDistancePID(nullptr);

	AngleController.Reset();
	AngleController.SetSetpoint(angle);
	AngleController.Enable();

	SmartDashboard::PutNumber("Target Angle", angle);
}

void Robot::RunMotorsTestFor(int numberOfSeconds)
{
	DriveTrain.ArcadeDrive(1, 0);
	Wait(numberOfSeconds / 2);
	DriveTrain.ArcadeDrive(-1, 0);
	Wait(numberOfSeconds / 2);
	DriveTrain.ArcadeDrive(0, 0);
}

void Robot::DriveTest()
{
	double forwardSpeed = 0;
	double turnSpeed = 0;

	// If they press A, use single stick arcade with the left joystick
	if(DriveController.GetAButton())
	{
		forwardSpeed = DriveController.GetY(GenericHID::JoystickHand::kLeftHand);
		turnSpeed = DriveController.GetX(GenericHID::JoystickHand::kLeftHand);
	}
	// If they press the left bumper, use the left joystick for forward and
	// backward motion and the right joystick for turning
	else if(DriveController.GetBumper(GenericHID::JoystickHand::kLeftHand))
	{
		forwardSpeed = DriveController.GetY(GenericHID::JoystickHand::kLeftHand);
		turnSpeed = DriveController.GetX(GenericHID::JoystickHand::kRightHand);
	}
	// If they press the right bumper, use the right joystick for forward and
	// backward motion and the left joystick for turning
	else if(DriveController.GetBumper(GenericHID::JoystickHand::kRightHand))
	{
		forwardSpeed = DriveController.GetY(GenericHID::JoystickHand::kRightHand);
		turnSpeed = DriveController.GetX(GenericHID::JoystickHand::kLeftHand);
	}

	// Negative is used to make forward positive and backwards negative
	// because the y-axes of the XboxController are natively inverted
	DriveTrain.ArcadeDrive(-forwardSpeed, turnSpeed);
}

void Robot::ManualElevatorTest()
{
	// Use the right trigger to manually raise the elevator and
	// the left trigger to lower the elevator
	double raiseElevatorOutput = applyDeadband(OperatorController.GetTriggerAxis(
			GenericHID::JoystickHand::kRightHand));
	double lowerElevatorOutput = applyDeadband(OperatorController.GetTriggerAxis(
			GenericHID::JoystickHand::kLeftHand));

	SmartDashboard::PutNumber("RaiseElev", raiseElevatorOutput);
	SmartDashboard::PutNumber("LowerElev", lowerElevatorOutput);

	if(raiseElevatorOutput != 0.0 || lowerElevatorOutput != 0.0)
	{
		ElevatorPIDController.Disable();
		double output = CapElevatorOutput(dabs(raiseElevatorOutput) - dabs(lowerElevatorOutput),
				SmartDashboard::GetBoolean("Toggle Elevator Safety", 0));
		RightElevatorMotor.Set(output);
		LeftElevatorMotor.Set(output);
		return;
	}
	else if(!ElevatorPIDController.IsEnabled())
	{
		RightElevatorMotor.Set(0);
		LeftElevatorMotor.Set(0);
	}
}

void Robot::CapElevatorSetpoint(double& setpoint)
{
	// Prevent the setpoint from dipping below the min
	if(setpoint < consts::ELEVATOR_SETPOINTS[0])
	{
		setpoint = consts::ELEVATOR_SETPOINTS[0];
	}
	// Prevent the setpoint from exceeding the max
	else if(setpoint > consts::ELEVATOR_SETPOINTS[consts::NUM_ELEVATOR_SETPOINTS - 1])
	{
		setpoint = consts::ELEVATOR_SETPOINTS[consts::NUM_ELEVATOR_SETPOINTS - 1];
	}
	else
	{
		return;
	}
}

void Robot::PIDElevatorTest()
{
	// Use the right trigger to manually raise the elevator and
	// the left trigger to lower the elevator
	double raiseElevatorOutput = applyDeadband(OperatorController.GetTriggerAxis(
			GenericHID::JoystickHand::kRightHand));
	double lowerElevatorOutput = applyDeadband(OperatorController.GetTriggerAxis(
			GenericHID::JoystickHand::kLeftHand));

	// If either triggers are being pressed
	if(raiseElevatorOutput != 0.0 || lowerElevatorOutput != 0.0)
	{
		// Output ranges from -1 to 1 and will act as a multiplier for the max increment
		double output = CapElevatorOutput(dabs(raiseElevatorOutput) - dabs(lowerElevatorOutput));

		// If the elevator is in automatic mode, turn it off, and set the desired height to
		// the current height plus some increment
		if(m_isElevatorInAutoMode)
		{
			m_isElevatorInAutoMode = false;
			double desiredSetpoint = ElevatorPID.GetHeightInches() + consts::ELEVATOR_INCREMENT_PER_CYCLE * output;
			CapElevatorSetpoint(desiredSetpoint);
			ElevatorPIDController.SetSetpoint(desiredSetpoint);
		}
		else // If automatic mode isn't on, just increment the previous setpoint
		{
			double desiredSetpoint = ElevatorPIDController.GetSetpoint() + consts::ELEVATOR_INCREMENT_PER_CYCLE * output;
			CapElevatorSetpoint(desiredSetpoint);
			ElevatorPIDController.SetSetpoint(desiredSetpoint);
		}
		ElevatorPIDController.Enable();
		return;
	}
	else // If neither of the triggers are being pressed, keep the elevator at its current height
	{
		ElevatorPIDController.SetSetpoint(ElevatorPID.GetHeightInches());
	}

	// Automatic Mode is controlled by both bumpers
	if (OperatorController.GetBumperPressed(GenericHID::JoystickHand::kRightHand))
	{
		// If elevator is lowering and the right bumper is pressed, stop elevator where it is
		if (m_isElevatorLowering)
		{
			ElevatorPIDController.SetSetpoint(ElevatorPID.GetHeightInches());
			m_isElevatorLowering = false;
		}
		else
		{
			// If right bumper is being pressed for the first time, increase the desired preset by 1
			if (!m_isElevatorInAutoMode)
			{
				m_targetElevatorStep = GetClosestStepNumber();
			}
			// If right bumper has already been pressed, go to the next step.
			else if (m_targetElevatorStep < consts::NUM_ELEVATOR_SETPOINTS - 1)
			{
				m_targetElevatorStep++;
			}
			ElevatorPIDController.SetSetpoint(consts::ELEVATOR_SETPOINTS[m_targetElevatorStep]);
			m_isElevatorInAutoMode = true;
			m_isElevatorLowering = false;
		}
	}
	// The left bumper will lower the elevator to the bottom
	else if (OperatorController.GetBumperPressed(GenericHID::JoystickHand::kLeftHand))
	{
		m_isElevatorInAutoMode = true;
		m_isElevatorLowering = true;
		ElevatorPIDController.SetSetpoint(consts::ELEVATOR_SETPOINTS[0]);
	}

	ElevatorPIDController.Enable();

	SmartDashboard::PutBoolean("Lowering?", m_isElevatorLowering);
	SmartDashboard::PutBoolean("Automatic?", m_isElevatorInAutoMode);
	SmartDashboard::PutNumber("Elevator Height", ElevatorPID.GetHeightInches());
	SmartDashboard::PutNumber("Elevator Setpoint", ElevatorPIDController.GetSetpoint());
	SmartDashboard::PutNumber("Elevator Output", ElevatorPIDController.Get());
}

void Robot::FullElevatorTest()
{
	// Use the right trigger to manually raise the elevator and
	// the left trigger to lower the elevator
	double raiseElevatorOutput = applyDeadband(OperatorController.GetTriggerAxis(
			GenericHID::JoystickHand::kRightHand));
	double lowerElevatorOutput = applyDeadband(OperatorController.GetTriggerAxis(
			GenericHID::JoystickHand::kLeftHand));

	SmartDashboard::PutNumber("RaiseElev", raiseElevatorOutput);
	SmartDashboard::PutNumber("LowerElev", lowerElevatorOutput);

	// If either triggers are being pressed, disable the PID and
	// set the motor to the given speed
	if(raiseElevatorOutput != 0.0 || lowerElevatorOutput != 0.0)
	{
		ElevatorPIDController.Disable();
		double output = CapElevatorOutput(dabs(raiseElevatorOutput) - dabs(lowerElevatorOutput),
				SmartDashboard::GetBoolean("Toggle Elevator Safety", 0));
		RightElevatorMotor.Set(output);
		LeftElevatorMotor.Set(output);
		return;
	}
	else if(!ElevatorPIDController.IsEnabled())
	{
		RightElevatorMotor.Set(0);
		LeftElevatorMotor.Set(0);
	}

	// Automatic Mode is controlled by both bumpers
	if (OperatorController.GetBumperPressed(GenericHID::JoystickHand::kRightHand))
	{
		// If elevator is lowering and the right bumper is pressed, stop elevator where it is
		if (m_isElevatorLowering)
		{
			RightElevatorMotor.Set(0);
			LeftElevatorMotor.Set(0);
			m_isElevatorLowering = false;
			ElevatorPIDController.Disable();
		}
		else
		{
			// If right bumper is being pressed for the first time, increase the desired preset by 1
			if (!ElevatorPIDController.IsEnabled())
			{
				m_targetElevatorStep = GetClosestStepNumber();
			}
			// If right bumper has already been pressed, go to the next step.
			else if (m_targetElevatorStep < consts::NUM_ELEVATOR_SETPOINTS - 1)
			{
				m_targetElevatorStep++;
			}
			ElevatorPIDController.SetSetpoint(consts::ELEVATOR_SETPOINTS[m_targetElevatorStep]);
			ElevatorPIDController.Enable();
			m_isElevatorLowering = false;
		}
	}
	// The left bumper will lower the elevator to the bottom
	if (OperatorController.GetBumperPressed(GenericHID::JoystickHand::kLeftHand))
	{
		m_isElevatorLowering = true;
		ElevatorPIDController.SetSetpoint(consts::ELEVATOR_SETPOINTS[0]);
		ElevatorPIDController.Enable();
	}
}

void Robot::AutoElevatorTest()
{
	SmartDashboard::PutNumber("Elevator Height", ElevatorPID.PIDGet());
//
//	if(SmartDashboard::GetBoolean("Go to Increment", 0))
//	{
//		ElevatorPIDController.SetSetpoint(SmartDashboard::GetNumber("Desired Increment", 0));
//	}

		double elevatorHeight = SmartDashboard::GetNumber("Elevator Setpoint", 0);
		if(dabs(elevatorHeight - ElevatorPID.PIDGet()) > consts::ELEVATOR_PID_DEADBAND)
		{
			double error = elevatorHeight - ElevatorPID.PIDGet();
			while(error > 1)
			{
				bool inAuto = IsTest();
				if(!inAuto)
				{
					RightElevatorMotor.Set(0);
					LeftElevatorMotor.Set(0);
					return;
				}

				SmartDashboard::PutBoolean("Elev On Target?", false);
				//To avoid damage, use basic p-control with an added constant output speed of 0.5
				error = elevatorHeight - ElevatorPID.PIDGet();
				RightElevatorMotor.Set(limit(error * SmartDashboard::GetNumber("Elevator P", 0) +
						SmartDashboard::GetNumber("Elevator Constant", 0), SmartDashboard::GetNumber("Elevator Limit", 0)));
				LeftElevatorMotor.Set(limit(error * SmartDashboard::GetNumber("Elevator P", 0) +
						SmartDashboard::GetNumber("Elevator Constant", 0), SmartDashboard::GetNumber("Elevator Limit", 0)));

				SmartDashboard::PutNumber("Elevator Height", ElevatorPID.PIDGet());
			}

			// ElevatorMotors set to a slow but constant speed to keep the elevator from falling
			// due to gravity
			RightElevatorMotor.Set(0.25);
			LeftElevatorMotor.Set(0.25);
		}
		EjectCube(consts::INTAKE_SPEED / 2.);


		SmartDashboard::PutBoolean("Elev On Target?", true);
		SmartDashboard::PutBoolean("Test Auto Elevator", false);

		// ElevatorMotors reset to 0
		RightElevatorMotor.Set(0);
		LeftElevatorMotor.Set(0);
}

void Robot::IntakeTest()
{
	// Use the B button to intake, X button to override IntakeUltrasonic
	if((OperatorController.GetBButton() && IntakeUltrasonic.GetRangeInches() > consts::MIN_DISTANCE_TO_CUBE) ||
			OperatorController.GetXButton())
	{
		RightIntakeMotor.Set(consts::INTAKE_SPEED);
		LeftIntakeMotor.Set(-consts::INTAKE_SPEED);
	}
	else
	{
		// Use the A button to eject if the B button is not being held
		if(OperatorController.GetAButton())
		{
			RightIntakeMotor.Set(-consts::INTAKE_SPEED);
			LeftIntakeMotor.Set(consts::INTAKE_SPEED);
		}
		else
		{
			RightIntakeMotor.Set(0);
			LeftIntakeMotor.Set(0);
		}
	}
}

void Robot::CurrentTest()
{
	SmartDashboard::PutNumber("FL Current",      FrontLeftMotor.GetOutputCurrent());
	SmartDashboard::PutNumber("FR Current",      FrontRightMotor.GetOutputCurrent());
	SmartDashboard::PutNumber("BL Current",      BackLeftMotor.GetOutputCurrent());
	SmartDashboard::PutNumber("BR Current",      BackRightMotor.GetOutputCurrent());
	SmartDashboard::PutNumber("RIntake Current", RightIntakeMotor.GetOutputCurrent());
	SmartDashboard::PutNumber("LIntake Current", LeftIntakeMotor.GetOutputCurrent());
}

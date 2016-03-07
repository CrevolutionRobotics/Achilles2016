#include "RobotDrive.h"
#include "Encoder.h"
#include "CANTalon.h"
#include "Analogpotentiometer.h"
#include "Joystick.h"
#include "IterativeRobot.h"
#include "Timer.h"
#include "CameraServer.h"


class Robot: public IterativeRobot
{
private:

	CANTalon *front_right_motor, *rear_right_motor, *front_left_motor, *rear_left_motor, *arm, *intake_roller, *shooter;
	Joystick *driver_controller, *operator_controller;
	AnalogPotentiometer *potcrack;
	Encoder *left_encoder;
	RobotDrive *first_person_drive;
	Timer *timer;


	int m1_a1 = 1400;

	bool completed_run = true;

	void RobotInit()
	{

	    CameraServer::GetInstance()->SetQuality(50);

	    CameraServer::GetInstance()->StartAutomaticCapture("cam1");

		front_left_motor = new CANTalon(1);
		rear_left_motor = new CANTalon(2);
		front_right_motor = new CANTalon(3);
		rear_right_motor = new CANTalon(4);

		intake_roller = new CANTalon(5);
		shooter = new CANTalon(6);
		arm = new CANTalon(7);

		potcrack = new AnalogPotentiometer(0);

		left_encoder = new Encoder(8, 9);

		driver_controller = new Joystick(0);
		operator_controller = new Joystick(1);

		first_person_drive = new RobotDrive(front_left_motor, rear_left_motor, front_right_motor, rear_right_motor);

		first_person_drive->SetSafetyEnabled(false);


	}



	void AutonomousInit()
	{
		left_encoder->Reset();

	}
/* This area of the code is for the autonomous actions that the robot must complete during autonomous period of the match.
 *
 * The structure of the code is outlined below:
 *
 * The autonomous code is organized by a switch/case function. The list of autons and their description is shown inside the AutonomousPeriodic() function
 *
 * auton_chooser: this variable controls which auton a script should run in the matc.h
 * Description of autons listed below
 *
 * auto_counter: this variable is used as a mimic time based function that uses the RoboRIO's clock to mimic time.
 * when auto_counter =  100  = 1 second
 *
 *
 *
 */
	void AutonomousPeriodic()
	{
		int auto_chooser = 6;
		int auto_counter = 0;
		while(IsAutonomous())
		{
			double pote = potcrack->Get();
			int left_encoder_value = left_encoder->Get();
			//SmartDashboard::PutNumber("pote value", pote);
			//SmartDashboard::PutNumber("left encoder value", left_encoder_value);
			switch(auto_chooser)
			{
				case 1:
				// auton for going over most defense that only require drivetrain
				//goes through low bar CHANGED SPEED TO 0.75 BC ARM WENT OPPOSITE
				{
				// This while loop allows the arm to drop from starting configuration to be able to pass through the low bar
					while(auto_counter < 200 )
					{
						arm->Set(0.75);
						auto_counter++;
					}
					arm->Set(0);
				//  This while loop controls the drivetrain to drive through any of the drive train based defense
					while(left_encoder->Get() < 1400)
					{
					// Don't need to go fast in auton, percision is the key to sucess
						first_person_drive->SetLeftRightMotorOutputs(0.515, 0.515);
					}
					first_person_drive->SetLeftRightMotorOutputs(0, 0);
				break;
				// End of auton
				}
			case 3:
			{
				while(potcrack->Get() > 0.934)
				{
					arm->Set(.6);
				}
				while(auto_counter < 100 )
				{
					first_person_drive->SetLeftRightMotorOutputs(0, 0);
					auto_counter++;
				}
				while(left_encoder->Get() < m1_a1)
				{
					first_person_drive->SetLeftRightMotorOutputs(0.715, 0.715);
				}
	     		first_person_drive->SetLeftRightMotorOutputs(0, 0);
				break;
			}

			case 2:// auton for going over most defense that only require drivetrain
			{
				while(left_encoder->Get() < 650)
				{
					left_encoder_value = left_encoder->Get();
					SmartDashboard::PutNumber("pote value", pote);
					SmartDashboard::PutNumber("left encoder value", left_encoder_value);
					first_person_drive->SetLeftRightMotorOutputs(0.515, 0.515);
				}
				first_person_drive->SetLeftRightMotorOutputs(0, 0);
				while(auto_counter < 500 )
				{
					first_person_drive->SetLeftRightMotorOutputs(0, 0);
					auto_counter++;
				}
				while(left_encoder->Get() < 1400)
				{
					first_person_drive->SetLeftRightMotorOutputs(0.515, 0.715);
					left_encoder_value = left_encoder->Get();
					SmartDashboard::PutNumber("pote value", pote);
					SmartDashboard::PutNumber("left encoder value", left_encoder_value);
				}
				first_person_drive->SetLeftRightMotorOutputs(0, 0);
				timer->Stop();
				break;
			}
			case 4:// auton for going over most defense that only require drivetrain
			{

				while(auto_counter < 650)
				{
					left_encoder_value = left_encoder->Get();
					SmartDashboard::PutNumber("pote value", pote);
					SmartDashboard::PutNumber("left encoder value", left_encoder_value);
					first_person_drive->SetLeftRightMotorOutputs(0.515, 0.515);
					auto_counter++;
				}
				first_person_drive->SetLeftRightMotorOutputs(0, 0);
				break;

			}
			case 5: // going over defense that requires only drive-train, then coming back for quick damage to defense
						{
							while(left_encoder->Get() < m1_a1)
										{
											first_person_drive->SetLeftRightMotorOutputs(0.515, 0.515);
										}
											first_person_drive->SetLeftRightMotorOutputs(0, 0);
							while(left_encoder->Get() > 300)
										{
											first_person_drive->SetLeftRightMotorOutputs(-0.515,-0.515);
										}
											first_person_drive->SetLeftRightMotorOutputs(0, 0);
										break;
						}
						case 6://Goes through Low Bar and then back through to start position
						{
							while(auto_counter < 230 )
								{
								arm->Set(0.70);
								auto_counter++;
								}
							arm->Set(0);
							while(left_encoder->Get() < 1600 && completed_run)
								{
								first_person_drive->SetLeftRightMotorOutputs(0.4, 0.4);
								}
							first_person_drive->SetLeftRightMotorOutputs(0, 0);
							while(left_encoder->Get() > 240 && completed_run)
								{
								first_person_drive->SetLeftRightMotorOutputs(-0.4, -0.4);
								}
							first_person_drive->SetLeftRightMotorOutputs(0, 0);
							completed_run = false;
							break;

						}
						case 7://Goes over Rockwall
						{
							while(left_encoder->Get() < 1600 && completed_run)
							{
								first_person_drive->SetLeftRightMotorOutputs(0.4, 0.4);
												}
											first_person_drive->SetLeftRightMotorOutputs(0, 0);
						}
			case 9: // herpderp
			{
				left_encoder->Reset();
				arm->Set(0);
				first_person_drive->SetLeftRightMotorOutputs(0, 0);
				intake_roller->Set(0);
				break;
			}

			}
		}
	}

	void PreSetsArm()
	{
		double pote = potcrack->Get();

		double calAbrate_point = 0.088;
		double Lowest_point = calAbrate_point - 0.076;
		double Low_point_1 = 6.3;
		double Low_point_2 = 0.914;

		double Intake_point_1 = 0.922;
		double Intake_point_2 = 0.924;

		if(operator_controller->GetRawButton(2)) // This preset is for moving arm down to low/portoperator_controllerallis position
		{
//			if(pote > Low_point_2)
//			{
//				arm->Set(-0.45);
//			}
//			else if(Low_point_2 > pote)
//			{
//				arm->Set(0.45);
//			}
//			else if(Low_point_1 > pote && Low_point_2 < pote)
//			{
//				arm->Set(0);
//			}
//			else
//			{
//				arm->Set(0);
//			}
		}
		else if(operator_controller->GetRawButton(3)) //This preset is for moving arm into intake position
		{
			if(pote > Intake_point_1 && pote < Intake_point_2)
			{
				arm->Set(0);
			}
			else if(pote > Intake_point_1)
			{
				arm->Set(0.45);
			}
			else if(pote < Intake_point_2)
			{
				arm->Set(-0.45);
			}
			else
			{
				arm->Set(0);
			}
		}
		else if(operator_controller->GetRawButton(4)) // This preset is for  shooting (if we have it)
		{
			//If we have intagrated a shooter we can use this to work with
		}
		else if(pote  < calAbrate_point && operator_controller->GetRawAxis(1) < 0)
		{
			arm->Set(0);
		}
		else if(pote > Lowest_point && operator_controller->GetRawAxis(1) > 0)
		{
			arm->Set(0);
		}
		else if(operator_controller->GetRawButton(1))
		{
			arm->Set(-0.8);
		}
		else
		{
			//pos  move back, neg moves forward
			arm->Set(0.75*operator_controller->GetRawAxis(1));
		}
	}
//	void drive_controls(int drive_type)
//	{
//		if(drive_type == 1)// first person shooter
//		{
//			 first_person_drive->SetLeftRightMotorOutputs(-driver_controller->GetRawAxis(1) + driver_controller->GetRawAxis(2), -driver_controller->GetRawAxis(1) - driver_controller->GetRawAxis(2));
//		}
//		else if(drive_type == 2)// tank drive
//		{
//			 first_person_drive->SetLeftRightMotorOutputs(-driver_controller->GetRawAxis(1), -driver_controller->GetRawAxis(3));
//		}
//		else if(drive_type == 3)// arcadic drive
//		{
//			 first_person_drive->SetLeftRightMotorOutputs(-driver_controller->GetRawAxis(1) + driver_controller->GetRawAxis(0), -driver_controller->GetRawAxis(1) - driver_controller->GetRawAxis(0));
//		}
//	}
	void TeleopInit()
	{
		left_encoder->Reset();
	}
	void TeleopPeriodic()
	{
		while(IsOperatorControl() && IsEnabled())
		{

			//double pote = potcrack->Get();
			int left_encoder_value = left_encoder->Get();
			//SmartDashboard::PutNumber("pote value", pote);
			SmartDashboard::PutNumber("left encoder value", left_encoder_value);

			first_person_drive->SetLeftRightMotorOutputs(-driver_controller->GetRawAxis(1) + driver_controller->GetRawAxis(2), -driver_controller->GetRawAxis(1) - driver_controller->GetRawAxis(2));

			if(operator_controller->GetRawButton(7))
			{
				intake_roller->Set(1);
			}
			else if(operator_controller->GetRawButton(8))
			{
				intake_roller->Set(-1);
			}
			else
			{
				intake_roller->Set(0);
			}
			PreSetsArm();
			//intake_controls();
			arm->Set(.75*operator_controller->GetRawAxis(1));
	        Wait(0.005);
	  }
	}

	void TestPeriodic()
	{

	}
};
START_ROBOT_CLASS(Robot)

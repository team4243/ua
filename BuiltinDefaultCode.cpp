#include "WPILib.h"
#include "ADXL345_I2C.h"
#include "DigitalModule.h"
#include "NetworkCommunication/UsageReporting.h"
#include "I2C.h"

/**
 * This "BuiltinDefaultCode" provides the "default code" functionality as used in the "Benchtop Test."
 * 
 * The BuiltinDefaultCode extends the IterativeRobot base class to provide the "default code"
 * functionality to confirm the operation and usage of the core control system components, as 
 * used in the "Benchtop Test" described in Chapter 2 of the 2009 FRC Control System Manual.
 * 
 * This program provides features in the Disabled, Autonomous, and Teleop modes as described
 * in the benchtop test directions, including "once-a-second" debugging printouts when disabled, 
 * a "KITT light show" on the solenoid lights when in autonomous, and elementary driving
 * capabilities and "button mapping" of joysticks when teleoperated.  This demonstration
 * program also shows the use of the MotorSafety timer.
 * 
 * This demonstration is not intended to serve as a "starting template" for development of
 * robot code for a team, as there are better templates and examples created specifically
 * for that purpose.  However, teams may find the techniques used in this program to be
 * interesting possibilities for use in their own robot code.
 * 
 * The details of the behavior provided by this demonstration are summarized below:
 *  
 * Disabled Mode:
 * - Once per second, print (on the console) the number of seconds the robot has been disabled.
 * 
 * Autonomous Mode:
 * - Flash the solenoid lights like KITT in Knight Rider
 * - Example code (commented out by default) to drive forward at half-speed for 2 seconds
 * 
 * Teleop Mode:
 * - Select between two different drive options depending upon Z-location of Joystick1
 * - When "Z-Up" (on Joystick1) provide "arcade drive" on Joystick1
 * - When "Z-Down" (on Joystick1) provide "tank drive" on Joystick1 and Joystick2
 * - Use Joystick buttons (on Joystick1 or Joystick2) to display the button number in binary on
 *   the solenoid LEDs (Note that this feature can be used to easily "map out" the buttons on a
 *   Joystick.  Note also that if multiple buttons are pressed simultaneously, a "15" is displayed
 *   on the solenoid LEDs to indicate that multiple buttons are pressed.)
 *
 * This code assumes the following connections:
 * - Driver Station:
 *   - USB 1 - The "right" joystick.  Used for either "arcade drive" or "right" stick for tank drive
 *   - USB 2 - The "left" joystick.  Used as the "left" stick for tank drive
 * 
 * - Robot:
 *   - Digital Sidecar 1:
 *     - PWM 1/3 - Connected to "left" drive motor(s)
 *     - PWM 2/4 - Connected to "right" drive motor(s)
 */
class BuiltinDefaultCode : public IterativeRobot
{
	// Declare variable for the robot drive system
//	RobotDrive *m_robotFrontDrive;		// robot will use PWM 1 & 2 (right & left) for front drive motors
//	RobotDrive *m_robotRearDrive;		// robot will use PWM 3 & 4 (right & left) for rear drive motors
	RobotDrive *m_robot;				// Arcade drive will use PWM's 2,1,4,3
	Jaguar	*m_feedMotor;	// frisbee feeder
	Jaguar *m_loadMotor; // robot loader
	Jaguar *m_launchMotor; // robot launcher
	Jaguar *m_elevatorMotor; // launcher elevation angle
	DigitalInput *m_loaderLimit; // limit switch
	ADXL345_I2C *m_Accelerometer; // accelerometer
	ADXL345_I2C::AllAxes *m_accelerations;
	
//	DigitalInput *m_loaderResetLimit; //limit switch

	//Declare driverstation display
	
	/*DriverStationLCD *m_dsLCD = DriverStationLCD::GetInstance();
	m_dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, "Hello World");
	*/
	// Declare a variable to use to access the driver station object
	DriverStation *m_ds;						// driver station object
	UINT32 m_priorPacketNumber;					// keep track of the most recent packet number from the DS
	UINT8 m_dsPacketsReceivedInCurrentSecond;	// keep track of the ds packets received in the current second
	
	// Declare variables for the two joysticks being used
	Joystick *m_rightStick;			// joystick 1 (arcade stick or right tank stick)
	Joystick *m_leftStick;			// joystick 2 (tank left stick)
//	double m_driveGain;				// common gain on commands to drive train allows more agile turning
	
	bool m_feeding;					// flag to know frisbee is feeding into loader
	int m_feedCount;				// loop count duration to feed frisbee
	double m_feedSpeed;				// Motor speed to feed frisbee
	int m_feedCounter;				// cycle counter for frisbee feeding time
	bool m_loading;					// flag to know frisbee is being loaded into breech
	int m_loadCount;				// loop count duration to load frisbee
	int m_loadPauseCount;			// loop count duration to turn off frisbee load motor before resetting
	double m_loadSpeed;				// Motor speed to load frisbee
	int m_loadCounter;				// cycle counter for frisbee loading time
	// define button numbers and speeds for the up and down controls
	unsigned int m_elevatorUpButton;
	float m_elevatorUpSpeed;
	unsigned int m_elevatorDownButton;
	float m_elevatorDownSpeed;
	double m_accelerationX;
	double m_accelerationY;
	double m_accelerationZ;
		
	int m_frisbeeCounter;			// frisbee counter for autonomous
	int m_frisbeeCountMax;			// maximum number of frisbees we're going to feed in autonomous
	bool m_breachFrisbee;			// frisbee in the breach
	static const int NUM_JOYSTICK_BUTTONS = 16;
	bool m_rightStickButtonState[(NUM_JOYSTICK_BUTTONS+1)];
	bool m_leftStickButtonState[(NUM_JOYSTICK_BUTTONS+1)];	
	
	// Declare variables for each of the eight solenoid outputs
	static const int NUM_SOLENOIDS = 8;
	Solenoid *m_solenoids[(NUM_SOLENOIDS+1)];

/*	enum {							// drive mode selection
		UNINITIALIZED_DRIVE = 0,
		ARCADE_DRIVE = 1,
		TANK_DRIVE = 2
	} m_driveMode; */
	
	// Local variables to count the number of periodic loops performed
	UINT32 m_autoPeriodicLoops;
	UINT32 m_disabledPeriodicLoops;
	UINT32 m_telePeriodicLoops;
		
public:
/**
 * Constructor for this "BuiltinDefaultCode" Class.
 * 
 * The constructor creates all of the objects used for the different inputs and outputs of
 * the robot.  Essentially, the constructor defines the input/output mapping for the robot,
 * providing named objects for each of the robot interfaces. 
 */
	BuiltinDefaultCode(void)

		{
		printf("BuiltinDefaultCode Constructor Started\n");

		// Create a robot using standard right/left robot drive on PWMS 1, 2, 3, and #4
		m_robot = new RobotDrive(2,4,1,3);
//		m_robotFrontDrive = new RobotDrive(2, 1); //swapped right and left since motors are inboard
//		m_robotRearDrive = new RobotDrive(4, 3); //swapped right and left since motors are inboard

		m_launchMotor = new Jaguar(6);
		m_feedMotor = new Jaguar(5);
		m_loadMotor = new Jaguar(7);
		m_loaderLimit = new DigitalInput(1,1); //limit switch
//		m_loaderResetLimit = new DigitalInput(1,2); //limit switch
		m_elevatorMotor = new Jaguar(8);
		m_elevatorUpButton = 8;
		m_elevatorUpSpeed = 1.0;
		m_elevatorDownButton = 9;
		m_elevatorDownSpeed = -m_elevatorUpSpeed;
		m_Accelerometer = new ADXL345_I2C(1,ADXL345_I2C::kRange_2G); // module 1
		  /*we're trying to get the constructer to work and we're having trouble with the second argument */
		
		// Acquire the Driver Station object
		m_ds = DriverStation::GetInstance();
		m_priorPacketNumber = 0;
		m_dsPacketsReceivedInCurrentSecond = 0;

		// Define joysticks being used at USB port #1 and USB port #2 on the Drivers Station
		m_rightStick = new Joystick(1);
		m_leftStick = new Joystick(2);

		// Iterate over all the buttons on each joystick, setting state to false for each
		UINT8 buttonNum = 1;						// start counting buttons at button 1
		for (buttonNum = 1; buttonNum <= NUM_JOYSTICK_BUTTONS; buttonNum++) {
			m_rightStickButtonState[buttonNum] = false;
			m_leftStickButtonState[buttonNum] = false;
		}

		// Iterate over all the solenoids on the robot, constructing each in turn
		UINT8 solenoidNum = 1;						// start counting solenoids at solenoid 1
		for (solenoidNum = 1; solenoidNum <= NUM_SOLENOIDS; solenoidNum++) {
			m_solenoids[solenoidNum] = new Solenoid(solenoidNum);
		}

		// Set drive mode to uninitialized
//		m_driveMode = UNINITIALIZED_DRIVE;

		// Initialize counters to record the number of loops completed in autonomous and teleop modes
		m_autoPeriodicLoops = 0;
		m_disabledPeriodicLoops = 0;
		m_telePeriodicLoops = 0;
		
		printf("BuiltinDefaultCode Constructor Completed\n");
	}
	
	
	/********************************** Init Routines *************************************/

	void RobotInit(void) {
		// Actions which would be performed once (and only once) upon initialization of the
		// robot would be put here.
		
		printf("RobotInit() completed.\n");
	}
	
	void DisabledInit(void) {
		m_disabledPeriodicLoops = 0;			// Reset the loop counter for disabled mode
//		ClearSolenoidLEDsKITT();
		// Move the cursor down a few, since we'll move it back up in periodic.
		printf("\x1b[2B");
	}

	void AutonomousInit(void) {
		m_autoPeriodicLoops = 0;				// Reset the loop counter for autonomous mode
//		ClearSolenoidLEDsKITT();
		m_feeding = true;
		m_feedCount = 50;						// (loops)
		m_feedSpeed = 0.5;						// (unitless)
		m_feedCounter = 0;
		m_loading = true;
		m_loadCount = 60;						// (loops)
		m_loadPauseCount = 2;					// (loops)
		m_loadSpeed = 0.2;						// (unitless)
		m_loadCounter = 0;
		m_frisbeeCounter = 0;					// (loops) frisbee counter for autonomous
		m_frisbeeCountMax = 3;					// (loops) maximum number of frisbees we're going to feed in autonomous
		m_breachFrisbee = true;
		
		m_robot->SetSafetyEnabled(false);
//		m_robotFrontDrive->SetSafetyEnabled(false);		
//		m_robotRearDrive->SetSafetyEnabled(false);
		m_launchMotor->SetSafetyEnabled(false);
		m_feedMotor->SetSafetyEnabled(false);
	}

	void TeleopInit(void) {
		m_telePeriodicLoops = 0;				// Reset the loop counter for teleop mode
		m_dsPacketsReceivedInCurrentSecond = 0;	// Reset the number of dsPackets in current second
//		m_driveMode = UNINITIALIZED_DRIVE;		// Set drive mode to uninitialized
//		ClearSolenoidLEDsKITT();
		m_feeding = false;
		m_feedCount = 50;						// (loops)
		m_feedSpeed = 0.5;						// (unitless)
		m_feedCounter = 0;
		m_loading = false;
		m_loadCount = 58;						// (loops)
		m_loadPauseCount = 2;					// (loops)
		m_loadSpeed = 0.2;						// (unitless)
		m_loadCounter = 0;
		m_robot->SetSafetyEnabled(false); // motor safety timers disabled as workaround for timeout problems on cRIO 2 
//		m_robotFrontDrive->SetSafetyEnabled(false);		
//		m_robotRearDrive->SetSafetyEnabled(false);
		m_launchMotor->SetSafetyEnabled(false);
		m_feedMotor->SetSafetyEnabled(false);
		m_loadMotor->SetSafetyEnabled(false);
		m_elevatorMotor->SetSafetyEnabled(false);
		
	}

	/********************************** Periodic Routines *************************************/
	
	void DisabledPeriodic(void)  {
		static INT32 printSec = (INT32)GetClock() + 1;
		static const INT32 startSec = (INT32)GetClock();


		// increment the number of disabled periodic loops completed
		m_disabledPeriodicLoops++;
		
		// while disabled, printout the duration of current disabled mode in seconds
		if (GetClock() > printSec) {
			// Move the cursor back to the previous line and clear it.
			printf("\x1b[1A\x1b[2K");
			printf("Disabled seconds: %d\r\n", printSec - startSec);			
			printSec++;
		}
	}

	void AutonomousPeriodic(void) {
		
		GetWatchdog().Feed();
		m_autoPeriodicLoops++;
		
		if (m_frisbeeCounter < m_frisbeeCountMax){
		m_launchMotor->Set(0.8);
		}

	if ((m_breachFrisbee)&& (m_autoPeriodicLoops >= 100)){ //delay for launcher to get speed
		if ((m_loading) && (m_loadCounter <= m_loadCount)){	// load breech
			m_loadCounter++; 
			m_loadMotor->Set(m_loadSpeed);
			} 
		else if ((m_loading) && (m_loadCounter > m_loadCount) && \
				(m_loadCounter <= ((m_loadCount+m_loadPauseCount)))) { // pause loader
			m_loadCounter++;
			m_loadMotor->Set(0.);	// pause loader 
			}
		else if ((m_loading) && \
				(m_loadCounter >= (m_loadCount+m_loadPauseCount)) && \
				(m_loaderLimit->Get())) {  // loader arm is still retracting
					// loader limit switch reads true when not depressed
			m_loadMotor->Set(-m_loadSpeed);;	// reset loader
			}
		else {
			// not loading, not feeding, finished retracting, ready to feed first frisbee
			m_loading = false;
			m_loadCounter = 0;
			m_loadMotor->Set(0.);; // stop loader			
			m_breachFrisbee = false;
			}
	}
		
	else if ((m_frisbeeCounter < m_frisbeeCountMax) && (!m_breachFrisbee)){	//launch all frisbees in the magazine and breach 
			
		if (m_feeding && (!m_loaderLimit->Get()) && \
				(m_feedCounter <= m_feedCount)){	
			//* loader limit switch reads true when not depressed *//
			//feed a frisbee from magazine
			m_feedCounter++;  
			m_feedMotor->Set(m_feedSpeed);
			}
		else if (m_feeding && (!m_loaderLimit->Get()) && \
				(m_feedCounter > m_feedCount)){	//done feeding a frisbee
			m_feeding = false;
			m_loading = true;
			m_feedCounter = 0;
			m_feedMotor->Set(0.);
			}
		else if ((m_loading && (!m_feeding)) && (m_loadCounter <= m_loadCount)){	// load breech
			m_loadCounter++; 
			m_loadMotor->Set(m_loadSpeed);
			} 
		else if ((m_loading && (!m_feeding)) && (m_loadCounter > m_loadCount) && \
				(m_loadCounter <= ((m_loadCount+m_loadPauseCount)))) { // pause loader
			m_loadCounter++;
			m_loadMotor->Set(0.);	// pause loader 
			}
		else if ((m_loading && (!m_feeding)) && \
				(m_loadCounter >= (m_loadCount+m_loadPauseCount)) && \
				(m_loaderLimit->Get())) {  // loader arm is still retracting
					// loader limit switch reads true when not depressed
			m_loadMotor->Set(-m_loadSpeed);;	// reset loader
			}
		else {
			// not loading, not feeding, finished retracting, ready for next frisbee
			m_loading = false;
			m_loadCounter = 0;
			m_loadMotor->Set(0.);; // stop loader			
			m_frisbeeCounter++;
			m_feeding = true;
			}
		}
	else if (m_frisbeeCounter >= m_frisbeeCountMax){ //finished with autonomous
		m_launchMotor->Set(0.0);
		}
		
	
	// 
		// generate KITT-style LED display on the solenoids
//		SolenoidLEDsKITT( m_autoPeriodicLoops );
				
		/* the below code (if uncommented) would drive the robot forward at half speed
		 * for two seconds.  This code is provided as an example of how to drive the 
		 * robot in autonomous mode, but is not enabled in the default code in order
		 * to prevent an unsuspecting team from having their robot drive autonomously!
		 */
		//* below code commented out for safety
//		if (m_autoPeriodicLoops <= (10 * (UINT32)GetLoopsPerSec())) {
		/*if (m_autoPeriodicLoops <= 100) {
			// When on the first periodic loop in autonomous mode, start driving forwards at half speed
			m_robot->Drive(0.25, 0.);			// drive forwards at quarter speed
//			m_robotFrontDrive->Drive(0.25, 0.25);			// drive forwards at quarter speed
//			m_robotRearDrive->Drive(0., 0.0);			// drive forwards at quarter speed
			}
		else {
			m_robot->Drive(0., 0.);			// drive forwards at quarter speed
//			m_robotFrontDrive->Drive(0.0, 0.0);			// stop robot
//			m_robotRearDrive->Drive(0.0, 0.0);			// stop robot			
			}
*/
	/* Driver station display output. */
	char msg[256];
static	DriverStationLCD *dsLCD = DriverStationLCD::GetInstance();
	sprintf(msg, "Frisbee Counter = %d", m_frisbeeCounter);
	dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, msg);
	sprintf(msg, "Feeder %s", m_feeding? "ON " : "OFF");
	dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, msg);
	sprintf(msg, "Feed Counter = %d", m_feedCounter);
	dsLCD->Printf(DriverStationLCD::kUser_Line3, 1, msg);
	sprintf(msg, "Load Counter = %d", m_loadCounter);
	dsLCD->Printf(DriverStationLCD::kUser_Line4, 1, msg);

				// line number (enum), starting col, format string, args for format string ...
	dsLCD->UpdateLCD();

	}      

	
	void TeleopPeriodic(void) {
		// increment the number of teleop periodic loops completed
		GetWatchdog().Feed();
		m_telePeriodicLoops++;

		/*
		 * No longer needed since periodic loops are now synchronized with incoming packets.
		if (m_ds->GetPacketNumber() != m_priorPacketNumber) {
		*/
			/* 
			 * Code placed in here will be called only when a new packet of information
			 * has been received by the Driver Station.  Any code which needs new information
			 * from the DS should go in here
			 */
			 
			m_dsPacketsReceivedInCurrentSecond++;					// increment DS packets received
						
			// put Driver Station-dependent code here

			// Demonstrate the use of the Joystick buttons
//			DemonstrateJoystickButtons(m_rightStick, m_rightStickButtonState, "Right Stick", &m_solenoids[1]);
//			DemonstrateJoystickButtons(m_leftStick, m_leftStickButtonState, "Left Stick ", &m_solenoids[5]);
		/***
			// determine if tank or arcade mode, based upon position of "Z" wheel on kit joystick
			if (m_rightStick->GetZ() <= 0) {    // Logitech Attack3 has z-polarity reversed; up is negative
				// use arcade drive
				m_robotDrive->ArcadeDrive(m_rightStick);			// drive with arcade style (use right stick)
				if (m_driveMode != ARCADE_DRIVE) {
					// if newly entered arcade drive, print out a message
					printf("Arcade Drive\n");
					m_driveMode = ARCADE_DRIVE;
				}
			} else {
				// use tank drive
				m_robotDrive->TankDrive(m_leftStick, m_rightStick);	// drive with tank style
				if (m_driveMode != TANK_DRIVE) {
					// if newly entered tank drive, print out a message
					printf("Tank Drive\n");
					m_driveMode = TANK_DRIVE;
				}
			}
***/
			
		/* Grab z-wheel value and transform from [1, -1] to [0,4600] 
		 * 4600 rpm is a guess */ 
		float rawZ, transformedZ;
		rawZ = m_leftStick->GetZ();
//		transformedZ = (1.0 - rawZ)/(-2.0);
		transformedZ = -2300.0 * rawZ + 2300.0;
		
		/* Driver station display output. */
		char msg[256];
static	DriverStationLCD *dsLCD = DriverStationLCD::GetInstance();
		sprintf(msg, "Launcher Speed = %f RPM", transformedZ);
		dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, msg);
		sprintf(msg, "Loader Limit = %u", m_loaderLimit->Get());
		dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, msg);
		sprintf(msg, "Loader Trigger = %u", m_leftStick->GetTrigger());
		dsLCD->Printf(DriverStationLCD::kUser_Line3, 1, msg);
					// line number (enum), starting col, format string, args for format string ...
		dsLCD->UpdateLCD();
		
		// use arcade drive
		//m_driveGain = (1.0 - m_rightStick->GetZ())/(-2.0);
		m_robot->ArcadeDrive(m_rightStick);			// drive with arcade style (use right stick)
//		m_robotFrontDrive->Drive(-m_driveGain*m_rightStick->GetY(),-m_driveGain*m_rightStick->GetX());	// negative sign to fix bug in turn
//		m_robotRearDrive->Drive(m_driveGain*m_rightStick->GetY(),-m_driveGain*m_rightStick->GetX());	// negative sign to fix bug in turn
		
		// -Add an elevation control, add a motor controller to port 8, and add a joystick button pair to control up/down
		if (m_rightStick->GetRawButton(m_elevatorUpButton)){
			m_elevatorMotor->Set(m_elevatorUpSpeed);
		}
		else if (m_rightStick->GetRawButton(m_elevatorDownButton)){
			m_elevatorMotor->Set(m_elevatorDownSpeed);
		}
		else {
			m_elevatorMotor->Set(0.0);
		}
		// trigger button activates feeding and loading mechanisms
		if ((m_leftStick->GetTop() || m_feeding) && (!m_loading) && (m_feedCounter <= m_feedCount)){
			m_feeding = true;
			m_feedCounter++;  
			m_feedMotor->Set(m_feedSpeed);
		}
		else {
			m_feeding = false;
			m_feedCounter = 0;
			m_feedMotor->Set(0.);
			}
		m_loaderLimit->Get(); //just testing digital input port 1
		
		// top(2) button uses the Z wheel to control the speed of the loading mechanisms
		if (((m_leftStick->GetTrigger() || m_loading) && (!m_feeding)) && (m_loadCounter <= m_loadCount)){//comment out this line
//		if (((m_leftStick->GetTrigger() || m_loading) && (!m_feeding)) && (bool) m_loaderLimit->Get()){  //uncomment this line
			m_loading = true;
			m_loadCounter++; // comment out this line
			m_loadMotor->Set(m_loadSpeed);	// load breech
			} 
		else if ((m_loading && (!m_feeding)) && (m_loadCounter > m_loadCount) && (m_loadCounter <= ((m_loadCount+m_loadPauseCount)))) { // comment out the pause logic
			m_loading = true;// comment out the pause logic
			m_loadCounter++;// comment out the pause logic,
			m_loadMotor->Set(0.);	// pause loader // comment out the pause logic
			}
//		else if ((m_loading && (!m_feeding)) && (m_loadCounter >= (m_loadCount+m_loadPauseCount)) && (m_loadCounter<= 2*m_loadCount-13)) {  // comment out this line
		else if ((m_loading && (!m_feeding)) && (m_loadCounter >= (m_loadCount+m_loadPauseCount)) && (m_loaderLimit->Get()!=0)) {
				//Retracting loader arm (feeder), have not yet hit the limit switch.
//		else if ((m_loading &! m_feeding) && (bool) m_loaderResetLimit->Get()) { // uncomment this line
			m_loading = true;
			m_loadCounter++;	// comment out the pause logic
			m_loadMotor->Set(-m_loadSpeed);;	// reset loader
			}
		else {
			//Finished retracting loader arm (feeder), so stop it.
			m_loading = false;
			m_loadCounter = 0;
			m_loadMotor->Set(0.);; // stop loader			
			}
		// Set launcher motor command		
		m_launchMotor->Set(-(1.0 - m_leftStick->GetZ())/(-2.0)); // transform from [-1.0, +1.0] to [0.0, +1.0]
	
		/*
		}  // if (m_ds->GetPacketNumber()...
		*/
		// use buttons 8 and 9 to trim load motor position in maintenance mode
	
		/*if (m_leftStick->GetRawButton(8) && (!m_feeding) && (!m_loading)){	
					m_loadMotor->Set(0.12);
					}
				else if (m_leftStick->GetRawButton(9) && (!m_feeding) && (!m_loading)){
					m_loadMotor->Set(-0.1);
					}*/
		//Use buttons 10 and 11 to trim feed motor position in maintenance mode
		if (m_leftStick->GetRawButton(11) && (!m_feeding) && (!m_loading)){	
							m_feedMotor->Set(0.15);
							}
						else if (m_leftStick->GetRawButton(10) && (!m_feeding) && (!m_loading)){
							m_feedMotor->Set(-0.1);
							}
				
	} // TeleopPeriodic(void)


/********************************** Continuous Routines *************************************/

	/* 
	 * These routines are not used in this demonstration robot
	 *
	 * 
	void DisabledContinuous(void) {
	}

	void AutonomousContinuous(void)	{
	}

	void TeleopContinuous(void) {
	}
	*/

	
/********************************** Miscellaneous Routines *************************************/
	
	/**
	 * Clear KITT-style LED display on the solenoids
	 * 
	 * Clear the solenoid LEDs used for a KITT-style LED display.
	 */	
/*	void ClearSolenoidLEDsKITT() {
		// Iterate over all the solenoids on the robot, clearing each in turn
		UINT8 solenoidNum = 1;						// start counting solenoids at solenoid 1
		for (solenoidNum = 1; solenoidNum <= NUM_SOLENOIDS; solenoidNum++) {
			m_solenoids[solenoidNum]->Set(false);
		}
	}
	
	/**
	 * Generate KITT-style LED display on the solenoids
	 * 
	 * This method expects to be called during each periodic loop, with the argument being the 
	 * loop number for the current loop.
	 * 
	 * The goal here is to generate a KITT-style LED display.  (See http://en.wikipedia.org/wiki/KITT )
	 * However, since the solenoid module has two scan bars, we can have ours go in opposite directions!
	 * The scan bar is written to have a period of one second with six different positions.
	 *
	void SolenoidLEDsKITT(UINT32 numloops) {
		unsigned int const NUM_KITT_POSITIONS = 6;
		UINT16 numloop_within_second = numloops % (UINT32)GetLoopsPerSec();

		if (numloop_within_second == 0) {
			// position 1; solenoids 1 and 8 on
			m_solenoids[1]->Set(true);  m_solenoids[8]->Set(true);
			m_solenoids[2]->Set(false); m_solenoids[7]->Set(false);
		} else if (numloop_within_second == ((UINT32)GetLoopsPerSec() / NUM_KITT_POSITIONS)) {
			// position 2; solenoids 2 and 7 on
			m_solenoids[2]->Set(true);  m_solenoids[7]->Set(true);
			m_solenoids[1]->Set(false); m_solenoids[8]->Set(false);
		} else if (numloop_within_second == ((UINT32)GetLoopsPerSec() * 2 / NUM_KITT_POSITIONS)) {
			// position 3; solenoids 3 and 6 on
			m_solenoids[3]->Set(true);  m_solenoids[6]->Set(true);
			m_solenoids[2]->Set(false); m_solenoids[7]->Set(false);
		} else if (numloop_within_second == ((UINT32)GetLoopsPerSec() * 3 / NUM_KITT_POSITIONS)) {
			// position 4; solenoids 4 and 5 on
			m_solenoids[4]->Set(true);  m_solenoids[5]->Set(true);
			m_solenoids[3]->Set(false); m_solenoids[6]->Set(false);
		} else if (numloop_within_second == ((UINT32)GetLoopsPerSec() * 4 / NUM_KITT_POSITIONS)) {
			// position 5; solenoids 3 and 6 on
			m_solenoids[3]->Set(true);  m_solenoids[6]->Set(true);
			m_solenoids[4]->Set(false); m_solenoids[5]->Set(false);
		} else if (numloop_within_second == ((UINT32)GetLoopsPerSec() * 5 / NUM_KITT_POSITIONS)) {
			// position 6; solenoids 2 and 7 on
			m_solenoids[2]->Set(true);  m_solenoids[7]->Set(true);
			m_solenoids[3]->Set(false); m_solenoids[6]->Set(false);
		} 
	}

	/**
	 * Demonstrate handling of joystick buttons
	 * 
	 * This method expects to be called during each periodic loop, providing the following
	 * capabilities:
	 * - Print out a message when a button is initially pressed
	 * - Solenoid LEDs light up according to joystick buttons:
	 *   - When no buttons pressed, clear the solenoid LEDs
	 *   - When only one button is pressed, show the button number (in binary) via the solenoid LEDs
	 *   - When more than one button is pressed, show "15" (in binary) via the solenoid LEDs
	 *
	void DemonstrateJoystickButtons(Joystick *currStick,
									bool *buttonPreviouslyPressed,
									const char *stickString,
									Solenoid *solenoids[]) {
		
		UINT8 buttonNum = 1;				// start counting buttons at button 1
		bool outputGenerated = false;		// flag for whether or not output is generated for a button
		INT8 numOfButtonPressed = 0;		// 0 if no buttons pressed, -1 if multiple buttons pressed
		
		/* Iterate over all the buttons on the joystick, checking to see if each is pressed
		 * If a button is pressed, check to see if it is newly pressed; if so, print out a
		 * message on the console
		 *
		for (buttonNum = 1; buttonNum <= NUM_JOYSTICK_BUTTONS; buttonNum++) {
			if (currStick->GetRawButton(buttonNum)) {
				// the current button is pressed, now act accordingly...
				if (!buttonPreviouslyPressed[buttonNum]) {
					// button newly pressed; print out a message
					if (!outputGenerated) {
						// print out a heading if no other button pressed this cycle
						outputGenerated = true;
						printf("%s button pressed:", stickString);
					}
					printf(" %d", buttonNum);
				}
				// remember that this button is pressed for the next iteration
				buttonPreviouslyPressed[buttonNum] = true;
				
				// set numOfButtonPressed appropriately
				if (numOfButtonPressed == 0) {
					// no button pressed yet this time through, set the number correctly
					numOfButtonPressed = buttonNum;
				} else {
					// another button (or buttons) must have already been pressed, set appropriately
					numOfButtonPressed = -1;
				}
			} else {
				buttonPreviouslyPressed[buttonNum] = false;
			}
		}
		
		// after iterating through all the buttons, add a newline to output if needed
		if (outputGenerated) {
			printf("\n");
		}
		
		if (numOfButtonPressed == -1) {
			// multiple buttons were pressed, display as if button 15 was pressed
			DisplayBinaryNumberOnSolenoidLEDs(15, solenoids);
		} else {
			// display the number of the button pressed on the solenoids;
			// note that if no button was pressed (0), the solenoid display will be cleared (set to 0)
			DisplayBinaryNumberOnSolenoidLEDs(numOfButtonPressed, solenoids);
		}
	}


	/**
	 * Display a given four-bit value in binary on the given solenoid LEDs
	 *
	void DisplayBinaryNumberOnSolenoidLEDs(UINT8 displayNumber, Solenoid *solenoids[]) {

		if (displayNumber > 15) {
			// if the number to display is larger than can be displayed in 4 LEDs, display 0 instead
			displayNumber = 0;
		}
		
		solenoids[3]->Set( (displayNumber & 1) != 0);
		solenoids[2]->Set( (displayNumber & 2) != 0);
		solenoids[1]->Set( (displayNumber & 4) != 0);
		solenoids[0]->Set( (displayNumber & 8) != 0);
	}
*/			
};

START_ROBOT_CLASS(BuiltinDefaultCode);

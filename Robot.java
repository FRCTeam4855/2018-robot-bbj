package org.usfirst.frc.team4855.robot;

import com.kauailabs.navx.frc.*;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
//import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {
	
	//======== LE HIGH CONTROL PANEL ========
	final double A_LIFTSPEED 		= 0.6; //autonomous speed at which the lift rises AND falls
	final double A_LIFTSPEEDFAST 	= 0.7; //autonomous speed at which the lift rises when approaching scale
	final double A_LIFTSPEEDSLOW	= 0.5;
	final double A_ARMRAISESPEED 	= 0.5; 	//autonomous speed at which pivot arm rises
	final double A_BOTTOMMAXSPEED 	= 0.75; //max speed assigned in auto for shorter distances
	final double A_SLOWDECREMENT 	= 0.35; //used in the straightline control loop for slow trips
	final double A_FASTDECREMENT 	= 0.55; //0.45; //used in the straightline control loop for fast trips
	final double A_SLOWOUTSPEED 	= -0.5; //autonomous speed at which arm ejects for switch
	
	final double T_DRIVEDIVFACTOR 	= 2; 	//teleop drivetrain inputs are divided by this number when turbo is NOT engaged
	/* outdated
	final double T_ARMUPSPEED 		= 0.4; //teleop speed at which pivot arm rises
	final double T_ARMDOWNSPEED 	= -0.4; //teleop speed at which pivot arm lowers
	*/
	final double T_LIFTBOTTOMFACTOR = 0.45; 	//teleop factor multiplied to joystick input when in precise mode
	final double T_LIFTTOPFACTOR 	= 0.7; 	//teleop factor multiplied to joystick input when NOT in precise mode
	final double T_DRIVEDEADZONE 	= 0.15; //minimum value before joystick inputs will be considered
	final double T_MAXWHEELSPEED 	= 0.6; 	//max speed that the outtake wheels will spin at
	final double T_MAXWHEELSPEEDIN  = 0.6;	//max speed that the intake wheels will spin at
	final double T_PIVOTJOYSTICKSPEED = 0.4;//max speed that the pivot will move up (no control for down)
	//=======================================
	
	//Mag encoders
	Encoder Enc[] = {
		new Encoder(2,3),
		new Encoder(6,7),
		new Encoder(4,5),
		new Encoder(0,1)
	};
	
	//The one CIMcoder on the bot, measures distance traveled
	Encoder distance = new Encoder(8,9);
	
	//The XBox controllers
	Joystick testStick = new Joystick(0);
	Joystick joystick2 = new Joystick(1);

	//===ALL MOTORS
	//Directional motors
	Spark Dir[] = {
		new Spark(7),
		new Spark(6),
		new Spark(4),
		new Spark(5)
	};
	//Movement motors
	Spark Move[] = {
		new Spark(1),
		new Spark(2),
		new Spark(10),
		new Spark(0)
	};
	
	//Lift motor
	Spark lift = new Spark(3);
	
	//Intake-related motors and solenoid
	VictorSP intakeWheels = new VictorSP(8);
	VictorSP pivot = new VictorSP(9);
	DoubleSolenoid intakeSolenoid = new DoubleSolenoid(0,1);
	Solenoid quickRelease = new Solenoid(3);
	
	//Arm open variable
	boolean openArm = false;
	
	//Limit Switches
	DigitalInput liftSwitchMax = new DigitalInput(13);
    DigitalInput liftSwitchMin = new DigitalInput(11);
	
	int wheelTune = 0; //Remembers what wheel we are tweaking in test mode
	boolean emergencyTank = false; //True if the robot is in emergency tank drive mode
	boolean reverseRotate = false; //?????
	boolean driverOriented = true; //true = driver oriented, false = robot oriented
	
	//All for calculating wheel speed/angle
	final double ROBOT_WIDTH = 33;
	final double ROBOT_LENGTH = 28;
	final double ROBOT_R = Math.sqrt(Math.pow(ROBOT_LENGTH,2)+Math.pow(ROBOT_WIDTH,2));
	final double ENC_TO_DEG = 1.158333;
	final double ABS_TO_DEG = 11.244444;
	final double ENC_360 = 417;
	final double IN_TO_ENC = 10.394;
	double a, b, c, d, max, temp, rads; 
	double eA, eB, eC, eD;
	double jStr, jFwd, jRcw;
	double ws1, ws2, ws3, ws4;
	//Gradual starts/stops in teleop
	double wsActual1 = 0, wsActual2 = 0, wsActual3 = 0, wsActual4 = 0;
	Timer wsTicker = new Timer();
	
	//NAVX CONSTRUCTOR
	AHRS ahrs = new AHRS(SPI.Port.kMXP); //Use this for gyro
	
	UsbCamera camera;
	UsbCamera camera2; // beep beep lettuce
	NetworkTable table;
	
	//Swerve Wheels
	SwerveWheel wheels[] = {
		new SwerveWheel(Enc[0]),
		new SwerveWheel(Enc[1]),
		new SwerveWheel(Enc[2]),
		new SwerveWheel(Enc[3])
	};
	
	//These control the steering motors using the mers
	PIDController Dirloop[] = {
		new PIDController(0.035,0,0.01,Enc[0],Dir[0]),
		new PIDController(0.035,0,0.01,Enc[1],Dir[1]),
		new PIDController(0.035,0,0.01,Enc[2],Dir[2]),
		new PIDController(0.035,0,0.01,Enc[3],Dir[3])
	};
	
	//These are used for autonomous in the turning to a specific angle
	PIDController angle[] = {
		new PIDController(0.06,0,0.013,ahrs,Move[0]),
		new PIDController(0.06,0,0.013,ahrs,Move[1]),
		new PIDController(0.06,0,0.013,ahrs,Move[2]),
		new PIDController(0.06,0,0.013,ahrs,Move[3])
	};
	
	//These are used for autonomous in moving to a certain distance
	PIDController distances[] = {
		new PIDController(0.025,0,0.01,distance,Move[0]),
		new PIDController(0.025,0,0.01,distance,Move[1]),
		new PIDController(0.025,0,0.01,distance,Move[2]),
		new PIDController(0.025,0,0.01,distance,Move[3])
	};
	
	//Autonomous vars
	final int ROTATE_ANGLE = 1;
	final int PREPARE_TURN = 2;
	final int DRIVE_STRAIGHT = 3;
	final int SIT_STILL = 4;
	final int PREPARE_DRIVESTRAIGHT = 5;
	
	final int DIR_FORWARD = 0;
	final int DIR_RIGHT = 1;
	final int DIR_LEFT = 2;
	final int DIR_BACKWARD = 3;
	
	double[] autoQueueTime=new double[15];
	int[] autoQueueCommand=new int[15];
	double[] autoQueueArgument = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	double[] autoQueueArgument2 = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	int autoIndex = 0;
	int autoCommand = 0;
	double autoArgument = 0;
	Timer autoTimer = new Timer(); //Keeps track of how long an autonomous command has been executing for
	double autoTimeLimit = 0;
	String autoProgram = ""; //Stores what program we are running
	boolean hasTriggeredAutonomous = false; //Does exactly what it sounds like
	NetworkTable db = NetworkTable.getTable("SmartDashboard/DB");
	double theError = 0; //Used in vision tracking to store margin of error for correction
	int applyingIndexes = 0; //Used so the programmer doesn't have to specify index number for each command
	boolean initialized = false;
	double rememberSetpoint = 0;
	//Autonomous Actuation vars (completely independent from all the movement vars)
	final int RAISE_LIFT = 1;
	final int LOWER_LIFT = 2;
	final int OPEN_ARM = 3;
	final int CLOSE_ARM = 4;
	final int INTAKE_IN = 5;
	final int INTAKE_OUT = 6;
	final int DO_NOTHING = 7;
	final int RAISE_ARM = 8;
	final int LOWER_ARM = 9;
	final int INTAKE_OUT_HALF = 10;
	final int RAISE_LIFT_FAST = 11;
	final int RAISE_LIFT_SLOW = 12;
	double[] autoActuateTime = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	int[] autoActuateCommand = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	int applyingIndexes2 = 0;
	int autoCommand2 = 0;
	double autoTimeLimit2 = 0;
	int autoIndex2 = -1;
	Timer autoTimer2 = new Timer();
	//UI Vars
	boolean[] leftBoxes = {false,false,false};
	boolean[] centerBoxes = {false,false,false};
	boolean[] rightBoxes = {false,false,false};
	String autoPosition;
	String finalSelection = "Forward"; //our final autonomous selection
	boolean prioritizeScale = false;
	boolean attempt2Cube = true;
	
	@Override
	public void robotInit() {
		
		SmartDashboard.putBoolean("L) Switch", true);
		SmartDashboard.putBoolean("L) Scale", true);
		SmartDashboard.putBoolean("L) Cross Line", true);
		SmartDashboard.putBoolean("C) Right Switch", true);
		SmartDashboard.putBoolean("C) Left Switch", true);
		SmartDashboard.putBoolean("C) FORWARD ONLY", false);
		SmartDashboard.putBoolean("R) Switch", true);
		SmartDashboard.putBoolean("R) Scale", true);
		SmartDashboard.putBoolean("R) Cross Line", true);
		SmartDashboard.putBoolean("Prioritize the Scale", true);
		SmartDashboard.putBoolean("Attempt Switch 2-cube", true);
		
		SmartDashboard.putString("Starting Position", "C");
		SmartDashboard.putString("AUTO OVERRIDE", "");
		
		//Prepare the drive camera
		camera = CameraServer.getInstance().startAutomaticCapture(0);
		camera.setBrightness(20);
		camera.setExposureManual(50);
		camera.setResolution(320, 240);
		camera.setFPS(15);
		
		camera2 = CameraServer.getInstance().startAutomaticCapture(1);
		camera2.setBrightness(25);
		camera2.setExposureManual(37);
		camera2.setResolution(320, 240);
		camera2.setFPS(15);
		
		Dirloop[0].setOutputRange(-1, 1);
		Dirloop[1].setOutputRange(-1, 1);
		Dirloop[2].setOutputRange(-1, 1);
		Dirloop[3].setOutputRange(-1, 1);
		
		//Autonomous wheel speed loop settings
		
		for (int i=0;i<=3;i++) {
			angle[i].setOutputRange(-0.5, 0.5);
			angle[i].setInputRange(-180, 180);
			angle[i].setContinuous();
			distances[i].setOutputRange(-0.2, 0.2);
		}
	}
	
	public void disabledPeriodic() {
		// --*** ENTER HELL ***---
		leftBoxes[0] = SmartDashboard.getBoolean("L) Switch", false);
		leftBoxes[1] = SmartDashboard.getBoolean("L) Scale", false);
		leftBoxes[2] = SmartDashboard.getBoolean("L) Cross Line", true);
		centerBoxes[0] = SmartDashboard.getBoolean("C) Right Switch", false);
		centerBoxes[1] = SmartDashboard.getBoolean("C) Left Switch", false);
		centerBoxes[2] = SmartDashboard.getBoolean("C) FORWARD ONLY", false);
		rightBoxes[0] = SmartDashboard.getBoolean("R) Switch", false);
		rightBoxes[1] = SmartDashboard.getBoolean("R) Scale", false);
		rightBoxes[2] = SmartDashboard.getBoolean("R) Cross Line", true);
		prioritizeScale = SmartDashboard.getBoolean("Prioritize the Scale", false);
		String gameData = DriverStation.getInstance().getGameSpecificMessage();
		autoPosition = SmartDashboard.getString("Starting Position", "C");
		attempt2Cube = SmartDashboard.getBoolean("Attempt Switch 2-cube", true);
		finalSelection = "";
		if(gameData.length() > 0) {
			if (autoPosition.equals("L")) {
				
				//WE ARE STARTING LEFT
				if (leftBoxes[0]) { //if switch program is enabled
					if (gameData.charAt(0) == 'L') {
						finalSelection = "Left Switch";
					}
				}
				if (leftBoxes[1]) { //if scale program is enabled
					if (gameData.charAt(1) == 'L') {
						if (!(finalSelection.equals("Left Switch")&&!prioritizeScale)) {
							finalSelection = "Left Scale";
						}
					}
				}
				if (leftBoxes[2]) { //if drive forward is enabled (PLZ)
					if (finalSelection == "") finalSelection = "Left";
				} else { //PLZ NO
					if (finalSelection == "") finalSelection = "Sit Still";
				}
				
			} else if (autoPosition.equals("R")) {
				
				//WE ARE STARTING RIGHT
				if (rightBoxes[0]) { //if switch program is enabled
					if (gameData.charAt(0) == 'R') {
						finalSelection = "Right Switch";
					}
				}
				if (rightBoxes[1]) { //if scale program is enabled
					if (gameData.charAt(1) == 'R') {
						if (!(finalSelection.equals("Right Switch")&&!prioritizeScale)) {
							finalSelection = "Right Scale";
						}
					}
				}
				if (rightBoxes[2]) { //if drive forward program is enabled (PLZ)
					if (finalSelection == "") finalSelection = "Right";
				} else { //PLZ NO
					if (finalSelection == "") finalSelection = "Sit Still";
				}
				
			} else {
				
				//SURELY WE MUST BE STARTING CENTER
				if (centerBoxes[0]) {
					if (gameData.charAt(0) == 'R') {
						finalSelection = "Forward Switch";
					}
				}
				if (centerBoxes[1]) {
					if (gameData.charAt(0) == 'L') {
						finalSelection = "Strafe Left Switch";
					}
				}
				if (centerBoxes[2]) {
					//FORWARD ONLY - do not attempt to claim either switch (why are you doing this??)
					finalSelection = "Forward";
				}
				if (finalSelection.equals("")) {
					finalSelection = "Forward";
				}
				
			}
		}
		
		//OVERRIDE for testing
		if (SmartDashboard.getString("AUTO OVERRIDE", "").equals("Test Arm")) {
			finalSelection = "Test Arm";
		}
	}

	public void setAutoQueue(double time, int command, double arg1, double arg2) {
		autoQueueTime[applyingIndexes] = time;
		autoQueueCommand[applyingIndexes] = command;
		autoQueueArgument[applyingIndexes] = arg1;
		autoQueueArgument2[applyingIndexes] = arg2;
		applyingIndexes ++;
	}
	
	public void setAutoQueue(double time, int command, double arg1) {
		autoQueueTime[applyingIndexes] = time;
		autoQueueCommand[applyingIndexes] = command;
		autoQueueArgument[applyingIndexes] = arg1;
		applyingIndexes ++;
	}
	
	public void setAutoQueue(double time, int command) {
		autoQueueTime[applyingIndexes] = time;
		autoQueueCommand[applyingIndexes] = command;
		applyingIndexes ++;
	}
	
	public void setActuateQueue(double time, int command) {
		autoActuateTime[applyingIndexes2] = time;
		autoActuateCommand[applyingIndexes2] = command;
		applyingIndexes2 ++;
	}
	
	@Override
	public void autonomousInit() {
		
		wsTicker.start();
		wsTicker.reset();
		distance.reset();
		ahrs.reset();
		applyingIndexes = 0;
		applyingIndexes2 = 0;
		initialized = false;
		rememberSetpoint = 0;
		
		openArm = false; //The arm is NOT OPEN
		
		distance.setReverseDirection(true);
		driverOriented = true;
		setAllPIDControllers(Dirloop, true);
		hasTriggeredAutonomous = true;
		//autoProgram = db.getString("String 0","0");
		
		ahrs.reset();
		Enc[0].reset();Enc[1].reset();Enc[2].reset();Enc[3].reset();
		resetAllWheels();
		
		switch (finalSelection) {
		
		//!!IMPORTANT!!
		//DIR_LEFT and DIR_RIGHT are flipped. Currently you should use
		//DIR_LEFT when going right, and DIR_RIGHT when going left.
		//ALWAYS!!!!!!!!!!
		
		case "0":
			setAutoQueue(1,PREPARE_TURN);
			setAutoQueue(3,ROTATE_ANGLE,90);
			setAutoQueue(1,PREPARE_TURN);
			setAutoQueue(3,ROTATE_ANGLE,0);
			break;
			
		case "1":
			setAutoQueue(5,DRIVE_STRAIGHT,145,DIR_FORWARD);
			break;
			
		case "Forward Switch":
			setAutoQueue(3,DRIVE_STRAIGHT,135,DIR_FORWARD);
			//We've hit the switch
			setAutoQueue(0.5,PREPARE_DRIVESTRAIGHT,DIR_BACKWARD);
			setAutoQueue(2,DRIVE_STRAIGHT,80,DIR_BACKWARD);
			//We've backed up from the switch
			setAutoQueue(0.5,PREPARE_DRIVESTRAIGHT,DIR_RIGHT);
			setAutoQueue(2,DRIVE_STRAIGHT,90,DIR_RIGHT);
			//We're now in front of the pile of cubes
			setAutoQueue(0.5,PREPARE_DRIVESTRAIGHT,DIR_FORWARD);
			setAutoQueue(1.5,DRIVE_STRAIGHT,25);
			//We've now stuck our arms in a cube
			setAutoQueue(0.5,PREPARE_DRIVESTRAIGHT,DIR_LEFT);
			setAutoQueue(1.5,DRIVE_STRAIGHT,80);
			//We're in front of the switch again
			setAutoQueue(0.5,PREPARE_DRIVESTRAIGHT,DIR_FORWARD);
			setAutoQueue(1.5,DRIVE_STRAIGHT,90); //original: 70
			//We've hit the switch again
			setAutoQueue(0.5,PREPARE_DRIVESTRAIGHT,DIR_BACKWARD);
			setAutoQueue(1,DRIVE_STRAIGHT,40);
			//We've backed up from the switch again
			
			setActuateQueue(0.5,RAISE_LIFT_SLOW);
			setActuateQueue(1,RAISE_LIFT_FAST);
			setActuateQueue(1,RAISE_LIFT);
			setActuateQueue(0.5,DO_NOTHING);
			//eject first cube
			setActuateQueue(1,INTAKE_OUT_HALF);
			setActuateQueue(0.5,LOWER_LIFT);
			setActuateQueue(5.5,OPEN_ARM);
			setActuateQueue(0.5,CLOSE_ARM);
			setActuateQueue(0.5,INTAKE_IN);
			setActuateQueue(1,RAISE_LIFT_FAST);
			setActuateQueue(1.5,RAISE_LIFT);
			setActuateQueue(1,INTAKE_OUT_HALF);
			break;
			
		case "Right":
			setAutoQueue(1,PREPARE_DRIVESTRAIGHT,DIR_LEFT);
			setAutoQueue(3,DRIVE_STRAIGHT,150,DIR_LEFT);
			break;
			
		case "Left":
			//this is the conservative just-cross-the-baseline program
			//setAutoQueue(1,PREPARE_DRIVESTRAIGHT,DIR_RIGHT);
			//setAutoQueue(3,DRIVE_STRAIGHT,150,DIR_RIGHT);
			
			//this is the crossover program that attempts to put us in a more strategic place
			setAutoQueue(1,PREPARE_DRIVESTRAIGHT,DIR_RIGHT);
			setAutoQueue(4,DRIVE_STRAIGHT,235,DIR_RIGHT);
			//setAutoQueue(1,PREPARE_DRIVESTRAIGHT,DIR_FORWARD);
			//setAutoQueue(4,DRIVE_STRAIGHT,190,DIR_FORWARD);
			break;
			
		case "Sit Still":
			break;
			
		case "Strafe Left Switch":
			setAutoQueue(2,DRIVE_STRAIGHT,75,DIR_FORWARD);
			setAutoQueue(0.5,PREPARE_DRIVESTRAIGHT,DIR_RIGHT);
			setAutoQueue(3.5,DRIVE_STRAIGHT,140,DIR_RIGHT);
			setAutoQueue(0.5,PREPARE_DRIVESTRAIGHT,DIR_FORWARD);
			setAutoQueue(2,DRIVE_STRAIGHT,95,DIR_FORWARD);
			//We start backin up heres
			setAutoQueue(1,PREPARE_DRIVESTRAIGHT,DIR_BACKWARD);
			setAutoQueue(2,DRIVE_STRAIGHT,100,DIR_BACKWARD);
			setAutoQueue(1,PREPARE_DRIVESTRAIGHT,DIR_LEFT);
			setAutoQueue(2,DRIVE_STRAIGHT,65,DIR_LEFT);
			//setAutoQueue(1,PREPARE_DRIVESTRAIGHT,DIR_FORWARD);
			//setAutoQueue(2,DRIVE_STRAIGHT,45,DIR_FORWARD);
			
			setActuateQueue(4.5,DO_NOTHING);
			setActuateQueue(0.5,RAISE_LIFT_SLOW);
			setActuateQueue(2.5,RAISE_LIFT);
			setActuateQueue(0.5,DO_NOTHING);
			setActuateQueue(0.5,INTAKE_OUT_HALF);
			setActuateQueue(0.5,RAISE_LIFT);
			setActuateQueue(1,OPEN_ARM);
			break;
			
		case "Left Switch":
			setAutoQueue(0.5,PREPARE_DRIVESTRAIGHT,DIR_RIGHT);
			setAutoQueue(4,DRIVE_STRAIGHT,175,DIR_RIGHT);
			setAutoQueue(0.5,PREPARE_DRIVESTRAIGHT,DIR_FORWARD);
			setAutoQueue(2,DRIVE_STRAIGHT,35,DIR_FORWARD);
			//BACK UP - NEEDS TESTING
			setAutoQueue(1,PREPARE_DRIVESTRAIGHT,DIR_BACKWARD);
			setAutoQueue(1.5,DRIVE_STRAIGHT,30,DIR_BACKWARD);
			setAutoQueue(0.5,PREPARE_DRIVESTRAIGHT,DIR_RIGHT);
			setAutoQueue(1.5,DRIVE_STRAIGHT,60,DIR_RIGHT);
			
			setActuateQueue(2.5,DO_NOTHING);
			setActuateQueue(0.5,RAISE_LIFT_SLOW);
			setActuateQueue(3,RAISE_LIFT);
			setActuateQueue(1,DO_NOTHING);
			setActuateQueue(1,INTAKE_OUT_HALF);
			break;
			
		case "Left Scale":
			setAutoQueue(1,PREPARE_DRIVESTRAIGHT,DIR_RIGHT);
			setAutoQueue(8,DRIVE_STRAIGHT,370,DIR_RIGHT); //ORIGINAL: 320
			//setAutoQueue(0.5,PREPARE_DRIVESTRAIGHT,DIR_LEFT);
			//setAutoQueue(2,DRIVE_STRAIGHT,60);
			//setAutoQueue(1,PREPARE_TURN);
			//setAutoQueue(2,ROTATE_ANGLE,90);
			
			setActuateQueue(3.5,DO_NOTHING);
			setActuateQueue(0.5,RAISE_LIFT_SLOW);
			setActuateQueue(2.5,RAISE_LIFT_FAST);
			setActuateQueue(2,RAISE_LIFT_SLOW);
			setActuateQueue(0.6,RAISE_ARM);
			setActuateQueue(0.5,INTAKE_OUT);
			setActuateQueue(0.5,LOWER_LIFT);
			setActuateQueue(0.5,OPEN_ARM);
			break;
			
		case "Left Crossover":
			setAutoQueue(1,PREPARE_DRIVESTRAIGHT,DIR_RIGHT);
			setAutoQueue(4,DRIVE_STRAIGHT,265,DIR_RIGHT);
			//setAutoQueue(1,PREPARE_DRIVESTRAIGHT,DIR_FORWARD);
			//setAutoQueue(4,DRIVE_STRAIGHT,150,DIR_FORWARD);
			break;
			
		case "Right Switch":
			setAutoQueue(0.5,PREPARE_DRIVESTRAIGHT,DIR_LEFT);
			setAutoQueue(4,DRIVE_STRAIGHT,175,DIR_LEFT);
			setAutoQueue(0.5,PREPARE_DRIVESTRAIGHT,DIR_FORWARD);
			setAutoQueue(2,DRIVE_STRAIGHT,65,DIR_FORWARD);
			//BACK UP
			setAutoQueue(1,PREPARE_DRIVESTRAIGHT,DIR_BACKWARD);
			setAutoQueue(3,DRIVE_STRAIGHT,30,DIR_BACKWARD);
			
			setActuateQueue(2.5,DO_NOTHING);
			setActuateQueue(0.5,RAISE_LIFT_SLOW);
			setActuateQueue(3,RAISE_LIFT);
			setActuateQueue(1,DO_NOTHING);
			setActuateQueue(1,INTAKE_OUT_HALF);
			break;
			
		case "Back":
			setAutoQueue(1,PREPARE_DRIVESTRAIGHT,DIR_BACKWARD);
			setAutoQueue(3,DRIVE_STRAIGHT,30,DIR_BACKWARD);
			break;
			
		case "Forward":
			setAutoQueue(3,DRIVE_STRAIGHT,120,DIR_FORWARD);
			break;
			
		case "Right Scale":
			setAutoQueue(1,PREPARE_DRIVESTRAIGHT,DIR_LEFT);
			setAutoQueue(5,DRIVE_STRAIGHT,370,DIR_LEFT); //original: 335
			setAutoQueue(1,PREPARE_DRIVESTRAIGHT,DIR_FORWARD);
			//7 seconds have passed
			setAutoQueue(1,DRIVE_STRAIGHT,20,DIR_FORWARD);
			//8 seconds have passed and we've completed our forwards motion
			
			setActuateQueue(3.5,DO_NOTHING);
			setActuateQueue(0.5,RAISE_LIFT_SLOW);
			//4 seconds have passed
			setActuateQueue(2,RAISE_LIFT_FAST);
			//6 seconds have passed
			setActuateQueue(2,RAISE_LIFT_SLOW);
			//8 seconds have passed
			setActuateQueue(0.6,RAISE_ARM);
			setActuateQueue(0.5,INTAKE_OUT);
			setActuateQueue(0.5,LOWER_LIFT);
			break;
			
		case "Test Arm":
			setActuateQueue(0.5,RAISE_LIFT_SLOW);
			setActuateQueue(2.5,RAISE_LIFT_FAST);
			break;
		}
		
		setAutoQueue(90,SIT_STILL);
		setActuateQueue(90,DO_NOTHING);
		
		autoTimer.reset();
		autoTimer2.reset();
		autoTimeLimit = 0;
		autoTimeLimit2 = 0;
		autoIndex = -1;
		autoIndex2 = -1;
		autoCommand = 0;
		autoCommand2 = 0;
		autoTimer.start();
		autoTimer2.start();
	}

	/**
	 * This function is called periodically during autonomous
	 */
	
	@Override
	public void autonomousPeriodic() {
		
		SmartDashboard.putNumber("Gyro", ahrs.getYaw());
		SmartDashboard.putNumber("CIMCODER",distance.get());
		
		if (autoTimer.get() >= autoTimeLimit) {
			autoIndex ++;
			autoTimer.reset();
			autoTimeLimit = autoQueueTime[autoIndex];
			autoCommand = autoQueueCommand[autoIndex];
			initialized = false;
			rememberSetpoint = 0;
			for (int i=0;i<wheels.length;i++) wheels[i].lockFlip(false);
			System.out.println("Initialization code has run");
			//autoArgument = autoQueueArgument[autoIndex];
		}
		if (autoTimer2.get() >= autoTimeLimit2) {
			autoIndex2 ++;
			autoTimer2.reset();
			autoTimeLimit2 = autoActuateTime[autoIndex2];
			autoCommand2 = autoActuateCommand[autoIndex2];
		}
			
			switch (autoCommand2) {
				case RAISE_LIFT:
					liftTheLift(A_LIFTSPEED);
					intakeWheels.set(0);
					pivot.set(0);
					break;
				
				case LOWER_LIFT:
					liftTheLift(-A_LIFTSPEED);
					intakeWheels.set(0);
					pivot.set(0);
					break;
					
				case CLOSE_ARM:
					liftTheLift(0);
					intakeWheels.set(0);
					intakeSolenoid.set(DoubleSolenoid.Value.kReverse);
					pivot.set(0);
					break;
					
				case OPEN_ARM:
					liftTheLift(0);
					intakeWheels.set(0);
					pivot.set(0);intakeSolenoid.set(DoubleSolenoid.Value.kForward);
					break;
					
				case INTAKE_OUT:
					liftTheLift(0);
					intakeWheels.set(-1);
					pivot.set(0);
					break;
					
				case INTAKE_OUT_HALF:
					liftTheLift(0);
					intakeWheels.set(A_SLOWOUTSPEED);
					pivot.set(0);
					break;
					
				case INTAKE_IN:
					liftTheLift(0);
					intakeWheels.set(0.3);
					pivot.set(0);
					break;
					
				case DO_NOTHING:
					liftTheLift(0);
					intakeWheels.set(0);
					pivot.set(0);
					break;
					
				case RAISE_ARM:
					liftTheLift(0);
					intakeWheels.set(0);
					pivot.set(A_ARMRAISESPEED);
					break;
					
				case LOWER_ARM:
					
					break;
					
				case RAISE_LIFT_FAST:
					liftTheLift(A_LIFTSPEEDFAST);
					intakeWheels.set(0);
					pivot.set(0);
					break;
					
				case RAISE_LIFT_SLOW:
					liftTheLift(A_LIFTSPEEDSLOW);
					intakeWheels.set(0);
					pivot.set(0);
					break;
			}
			
			//AUTO COMMANDS
			switch (autoCommand) {
			case PREPARE_TURN:
				setAllPIDControllers(distances, false);
				setAllPIDControllers(angle, false);
				for (int i=0;i<wheels.length;i++) {
					wheels[i].lockFlip(true);
				}
				doSwerve(0,0,0.01);
				theError = 0;
				break;
				
			case ROTATE_ANGLE:
				for (int i=0;i<wheels.length;i++) {
					wheels[i].lockFlip(false);
				}
				if (!angle[0].isEnabled()) {
					setAllPIDControllers(angle, true);
				}
				
				if (autoQueueArgument[autoIndex] != -1) {
					setAllPIDSetpoints(angle, autoQueueArgument[autoIndex]);
				}
				break;
				
			case PREPARE_DRIVESTRAIGHT:
				setAllPIDControllers(distances, false);
				setAllPIDControllers(angle, false);
				for (int i=0;i<wheels.length;i++) wheels[i].lockFlip(true);
				setAllPIDControllers(Dirloop, true);
				switch ((int)autoQueueArgument[autoIndex]) {
					case DIR_FORWARD: doSwerve(0.01,0,0); break;
					case DIR_LEFT: doSwerve(0,0.01,0); break;
					case DIR_RIGHT: doSwerve(0,-0.01,0); break;
					case DIR_BACKWARD: doSwerve(-0.01,0,0); break;
				}
				break;
				
			case DRIVE_STRAIGHT:
				if (!initialized) {
					rememberSetpoint = Dirloop[0].getSetpoint();
					distance.reset();
					ahrs.reset();
					initialized = true;
				}
				setAllPIDSetpoints(Dirloop, rememberSetpoint);
				
				double targetDistance = autoQueueArgument[autoIndex]*IN_TO_ENC;
				double maxspeed = 1;
				if (autoQueueArgument[autoIndex] < 250) {maxspeed = A_BOTTOMMAXSPEED;}
				double movespeed = 0;
				
				if (distance.get() < 20*IN_TO_ENC) {
					movespeed = 0.02*distance.get() + 0.4;
				} else if (distance.get() < targetDistance - (40*IN_TO_ENC) && targetDistance > (10*IN_TO_ENC)) { //TEST THIS 
					movespeed = 1;
				} else if (distance.get() < targetDistance - (10*IN_TO_ENC)) {
					setAllPIDControllers(distances, false);
					movespeed = -0.02 * (distance.get() - (targetDistance - (40*IN_TO_ENC))) + 0.8;
				} else {
					setAllPIDControllers(distances, true);
					setAllPIDSetpoints(distances, targetDistance);
				}
				if (!distances[0].isEnabled()) {
					if (movespeed > maxspeed) movespeed = maxspeed; //failsafe
					if (movespeed < 0) movespeed = 0; //also a failsafe
					double set0=movespeed, set1=movespeed, set2=movespeed, set3=movespeed;
					int dir = (int)autoQueueArgument2[autoIndex];
					double decrement = A_SLOWDECREMENT;
					if (autoQueueArgument[autoIndex] > 250) {decrement = A_FASTDECREMENT;}
					if (ahrs.getYaw() > 0.4) {
						if (dir == DIR_FORWARD) {set1 -= decrement;set2 -= decrement;}
						if (dir == DIR_LEFT) {set0 -= decrement;set1 -= decrement;}
						if (dir == DIR_RIGHT) {set2 -= decrement;set3 -= decrement;}
						if (dir == DIR_BACKWARD) {set0 -= decrement;set3 -= decrement;}
							Move[0].set(set0);
							Move[1].set(set1);
							Move[2].set(set2);
							Move[3].set(set3);
					} else if (ahrs.getYaw() < -0.4) {
						if (dir == DIR_FORWARD) {set0 -= decrement;set3 -= decrement;}
						if (dir == DIR_LEFT) {set2 -= decrement;set3 -= decrement;}
						if (dir == DIR_RIGHT) {set1 -= decrement;set0 -= decrement;}
						if (dir == DIR_BACKWARD) {set1 -= decrement;set2 -= decrement;}
							Move[0].set(set0);
							Move[1].set(set1);
							Move[2].set(set2);
							Move[3].set(set3);
					} else {
						for (int i=0; i<4; i++) {
							Move[i].set(movespeed);
						}
					}
				}
				break;
				
				case SIT_STILL: //Use to end every auton program, disables all movement
					for (int i=0;i<4;i++) {Move[i].set(0);}
					setAllPIDControllers(distances, false);
					setAllPIDControllers(angle, false);
					break;
			}
		}

	public void teleopInit() {
		if (angle[0].isEnabled()) {
			setAllPIDControllers(angle, false);
		}
		if (distances[0].isEnabled()) {
			setAllPIDControllers(distances, false);
		}
		setAllPIDControllers(Dirloop, true);
		
		distance.reset();
		
		wsTicker.start();
		wsTicker.reset();
		if (!hasTriggeredAutonomous) {
			ahrs.reset();
			Enc[0].reset();Enc[1].reset();Enc[2].reset();Enc[3].reset();
			resetAllWheels();
		} else {
			hasTriggeredAutonomous = false;
		}
	}
	
	public void disabledInit() {
		setAllPIDControllers(Dirloop, false);
		setAllPIDControllers(angle, false);
		setAllPIDControllers(distances, false);
		
		setAllPIDSetpoints(Dirloop, 0);
	}
	
	/**
	 * This adjusts the angle of the wheels and sets their speed based on joystick/autonomous input.
	 * 
	 * @param FWD The desired forward speed of the robot
	 * @param STR The desired strafing speed of the robot
	 * @param RCW The desired rotation speed of the robot
	 */
	public void doSwerve(double FWD,double STR,double RCW) {
		if (driverOriented) {
			rads = ahrs.getYaw() * Math.PI/180;
			temp = FWD*Math.cos(rads) + STR*Math.sin(rads);
			STR = -FWD*Math.sin(rads) + STR*Math.cos(rads);
			FWD = temp;
		}

		a = STR - RCW * (ROBOT_LENGTH / ROBOT_R);
		b = STR + RCW * (ROBOT_LENGTH / ROBOT_R);
		c = FWD - RCW * (ROBOT_WIDTH / ROBOT_R);
		d = FWD + RCW * (ROBOT_WIDTH / ROBOT_R);

		//1..4: front_right, front_left, rear_left, rear_right

		ws1 = Math.sqrt(Math.pow(b,2)+Math.pow(c,2));
		ws2 = Math.sqrt(Math.pow(b,2)+Math.pow(d,2));
		ws3 = Math.sqrt(Math.pow(a,2)+Math.pow(d,2));
		ws4 = Math.sqrt(Math.pow(a,2)+Math.pow(c,2));

		eA = wheels[0].calculateWheelAngle(b,c);
		Dirloop[0].setSetpoint(eA);SmartDashboard.putNumber("Enc. A setpoint", eA);
		
		eB = wheels[1].calculateWheelAngle(b,d);
		Dirloop[1].setSetpoint(eB);SmartDashboard.putNumber("Enc. B setpoint", eB);
		
		eC = wheels[2].calculateWheelAngle(a,d);
		Dirloop[2].setSetpoint(eC);SmartDashboard.putNumber("Enc. C setpoint", eC);
		
		eD = wheels[3].calculateWheelAngle(a,c);
		Dirloop[3].setSetpoint(eD);SmartDashboard.putNumber("Enc. D setpoint", eD);

		max=ws1; if(ws2>max)max=ws2; if(ws3>max)max=ws3; if(ws4>max)max=ws4;
		if(max>1){ws1/=max; ws2/=max; ws3/=max; ws4/=max;}
		
		ws1*=wheels[0].getFlip();
		ws2*=wheels[1].getFlip();
		ws3*=wheels[2].getFlip();
		ws4*=wheels[3].getFlip();
		
		//Move[2].set(testStick.getRawAxis(1));
		
		if (wsTicker.get()>0.1) {
			if (ws1 - wsActual1 > 0.1) {wsActual1 += 0.1;} else if (ws1 - wsActual1 < -0.1) {wsActual1 -= 0.1;} else {wsActual1 = ws1;}
			if (ws2 - wsActual2 > 0.1) {wsActual2 += 0.1;} else if (ws2 - wsActual2 < -0.1) {wsActual2 -= 0.1;} else {wsActual2 = ws2;}
			if (ws3 - wsActual3 > 0.1) {wsActual3 += 0.1;} else if (ws3 - wsActual3 < -0.1) {wsActual3 -= 0.1;} else {wsActual3 = ws3;}
			if (ws4 - wsActual4 > 0.1) {wsActual4 += 0.1;} else if (ws4 - wsActual4 < -0.1) {wsActual4 -= 0.1;} else {wsActual4 = ws4;}
			wsTicker.reset();
		}
		//Move[0].set(wsActual1);Move[1].set(wsActual2);Move[2].set(wsActual3);Move[3].set(wsActual4);
		
		Move[0].set(ws1);Move[1].set(ws2);Move[2].set(ws3);Move[3].set(ws4);
	}
	
	@Override
	public void teleopPeriodic() {
		//SmartDashboard.putNumber("Arm Position Value", ai1.getValue());
		
		SmartDashboard.putNumber("CIMCODER", distance.get());
		SmartDashboard.putNumber("Gyro", ahrs.getYaw());
		
		if (!emergencyTank) {
			if (!testStick.getRawButton(1)) {
				driverOriented = true;
				jFwd = -testStick.getRawAxis(1);if (Math.abs(jFwd) < T_DRIVEDEADZONE) jFwd = 0;
				if (!testStick.getRawButton(5)) jFwd /= T_DRIVEDIVFACTOR;
				jStr = testStick.getRawAxis(0);if (Math.abs(jStr) < T_DRIVEDEADZONE) jStr = 0;
				if (!testStick.getRawButton(5)) jStr /= T_DRIVEDIVFACTOR;
				jRcw = testStick.getRawAxis(4);if (Math.abs(jRcw) < T_DRIVEDEADZONE) jRcw = 0;
				if (!testStick.getRawButton(5)) jRcw /= T_DRIVEDIVFACTOR;
				if (reverseRotate) {jRcw=-jRcw;}
				doSwerve(jFwd,jStr,jRcw);
			} else {
				driverOriented = false;
				if (ahrs.getRoll() >= 2) {
					doSwerve(0,0.5,0);
				} else if (ahrs.getRoll() <= -2) {
					doSwerve(0,-0.5,0);
				} else if (ahrs.getPitch() >= 2) {
					doSwerve(0.5,0,0);
				} else if (ahrs.getPitch() <= -2) {
					doSwerve(-0.5,0,0);
				}
			}
		} else {
			setAllPIDSetpoints(Dirloop, 0);
			resetAllWheels();
			Move[0].set(testStick.getRawAxis(5));Move[3].set(testStick.getRawAxis(5));
			Move[2].set(testStick.getRawAxis(1));Move[1].set(testStick.getRawAxis(1));
		}
				
		if (testStick.getRawButton(3)) {ahrs.reset();}
		
		//if (testStick.getRawButton(1)) {reverseRotate = true;}
		//if (testStick.getRawButton(2)) {reverseRotate = false;}
		if (testStick.getRawButton(4)) {
			resetAllWheels();
			setAllPIDSetpoints(Dirloop, 0);
		}
		//if (testStick.getRawButton(10)) {
			//driverOriented = true;
		//}
		//if (testStick.getRawButton(9)) {
			//driverOriented = false;
		//}
		
		//Dirloop1.setPID(SmartDashboard.getNumber("P",0), SmartDashboard.getNumber("I",0), SmartDashboard.getNumber("D",0));
		//Dirloop2.setPID(SmartDashboard.getNumber("P",0), SmartDashboard.getNumber("I",0), SmartDashboard.getNumber("D",0));
		//Dirloop3.setPID(SmartDashboard.getNumber("P",0), SmartDashboard.getNumber("I",0), SmartDashboard.getNumber("D",0));
		//Dirloop4.setPID(SmartDashboard.getNumber("P",0), SmartDashboard.getNumber("I",0), SmartDashboard.getNumber("D",0));
		
		SmartDashboard.putNumber("Encoder1:", Enc[0].get());
		SmartDashboard.putNumber("Encoder2:", Enc[1].get());
		SmartDashboard.putNumber("Encoder3:", Enc[2].get());
		SmartDashboard.putNumber("Encoder4:", Enc[3].get());
		
		// === BEGIN OPERARTOR CONTROLS ===
		
		if (joystick2.getRawAxis(3) >= 0.1) {
			intakeWheels.set(joystick2.getRawAxis(3) * T_MAXWHEELSPEEDIN);
		} else if (joystick2.getRawAxis(2) >= 0.1) {
			intakeWheels.set(-joystick2.getRawAxis(2) * T_MAXWHEELSPEED);
		} else if (joystick2.getRawButton(1)){
			intakeWheels.set(-0.15);
		} else {
			if (!openArm) {
				intakeWheels.set(0.25);
			} else {
				intakeWheels.set(0);
			}
		}
		
		double precision;
		if (joystick2.getRawButton(4)) {
			precision = T_LIFTBOTTOMFACTOR;
		} else precision = T_LIFTTOPFACTOR;
		liftTheLift(-joystick2.getRawAxis(1) * precision);
		
		if (joystick2.getRawButtonPressed(5)) {
			//Open arm
			intakeSolenoid.set(DoubleSolenoid.Value.kForward);
			openArm = true;
		}
		if (joystick2.getRawButtonPressed(6)) {
			//Close arm
			intakeSolenoid.set(DoubleSolenoid.Value.kReverse);
			openArm = false;
		}
		
		/* Old button-based pivot
		if (joystick2.getRawButton(1)) {
			//Pivot up
			pivot.set(T_ARMDOWNSPEED);
		} else if (joystick2.getRawButton(2)) {
			//Pivot down
			pivot.set(T_ARMUPSPEED);
		} else pivot.set(0);
		*/
		
		double pivotVal = joystick2.getRawAxis(5);
		if (Math.abs(pivotVal) > 0.1) {
			pivot.set(-pivotVal * T_PIVOTJOYSTICKSPEED);
		} else {
			pivot.set(0);
		}
		
		//Slow intake move
		//if (joystick2.getRawButton(3)) {
		//	intakeWheels.set(.1);
		//} else {
		//	intakeWheels.set(0);
		//}
		
		if (joystick2.getRawButton(5)) { //AAAAAAAAAAAAAAAAAAAAAAAAAa
			//Quick release
			quickRelease.set(true);
		} else quickRelease.set(false);
		
	}

	/**
	 * Resets all of the SwerveWheel objects, putting them on a clean slate
	 * (eliminates flipped orientations, stacked setpoints, etc.)
	 */
	public void resetAllWheels() {
		for (int i=0;i<=3;i++) {
			wheels[i].reset();
		}
	}
	
	/**
	 * Enables or disables a given array of four PIDController objects.
	 * @param pids The array of PID Controllers to set
	 * @param enabled True to enable, false to disable
	 */
	public void setAllPIDControllers(PIDController[] pids, boolean enabled) {
		for (int i=0;i<=3;i++) {
			pids[i].setEnabled(enabled);
		}
	}
	
	/**
	 * Sets the setpoints for an array of four PIDController objects.
	 * @param pids The array of PID Controllers to set
	 * @param setpoint The setpoint
	 */
	public void setAllPIDSetpoints(PIDController[] pids, double setpoint) {
		for (int i=0;i<=3;i++) {
			pids[i].setSetpoint(setpoint);
		}
	}
	
	/**
	 * Works the same as lift.set(double speed), except it considers
	 * limit switch input to stop the lift from going too far one way
	 * or the other.
	 * This should ALWAYS be used in place of lift.set().
	 * @param liftOutput Desired speed for the motors
	 */
	public void liftTheLift(double liftOutput) {
		if (!liftSwitchMin.get()) {
			if (liftOutput < 0) {
				liftOutput = 0;
			}
		}
        if (!liftSwitchMax.get()) {
        	if (liftOutput > 0) {
        		liftOutput = 0;
        	}
        }
		lift.set(liftOutput);
	}
	
	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		if (angle[0].isEnabled()) {
			setAllPIDControllers(angle, false);
		}
		if (distances[0].isEnabled()) {
			setAllPIDControllers(distances, false);
		}
		if (Dirloop[0].isEnabled()) {
			setAllPIDControllers(Dirloop, false);
		}
		
		//SmartDashboard.putNumber("AI 1", ai1.getValue());
		
		SmartDashboard.putNumber("Joystick y axis", testStick.getRawAxis(1));
		
		SmartDashboard.putBoolean("Lower Limit Value", liftSwitchMin.get());
		SmartDashboard.putBoolean("Upper Limit Value", liftSwitchMax.get());
		
		SmartDashboard.putNumber("Encoder1:", Enc[0].get());
		SmartDashboard.putNumber("Encoder2:", Enc[1].get());
		SmartDashboard.putNumber("Encoder3:", Enc[2].get());
		SmartDashboard.putNumber("Encoder4:", Enc[3].get());
		
		SmartDashboard.putNumber("NavX Pitch:", ahrs.getPitch());
		SmartDashboard.putNumber("Navx Roll:", ahrs.getRoll());
		SmartDashboard.putNumber("NavX Yaw:", ahrs.getYaw());
		SmartDashboard.putNumber("NavX Angle:", ahrs.getAngle());
		SmartDashboard.putNumber("NavX Raw X:", ahrs.getRawGyroX());
		
		if (testStick.getRawButton(1)) wheelTune = 0;
		if (testStick.getRawButton(2)) wheelTune = 1;
		if (testStick.getRawButton(3)) wheelTune = 2;
		if (testStick.getRawButton(4)) wheelTune = 3;
		
		switch (wheelTune) {
		case 0:
			if (testStick.getRawButton(5)) Dir[0].set(0.3);
			else if (testStick.getRawButton(6)) Dir[0].set(-0.3); else Dir[0].set(0);
			break;
		case 1:
			if (testStick.getRawButton(5)) Dir[1].set(0.3);
			else if (testStick.getRawButton(6)) Dir[1].set(-0.3); else Dir[1].set(0);
			break;
		case 2:
			if (testStick.getRawButton(5)) Dir[2].set(0.3);
			else if (testStick.getRawButton(6)) Dir[2].set(-0.3); else Dir[2].set(0);
			break;
		case 3:
			if (testStick.getRawButton(5)) Dir[3].set(0.3);
			else if (testStick.getRawButton(6)) Dir[3].set(-0.3); else Dir[3].set(0);
			break;
		}
		
		/*
		switch (wheelTune) {
		case 0:
			if (testStick.getRawButton(5)) Move1.set(0.3);
			else if (testStick.getRawButton(6)) Move1.set(-0.3); else Move1.set(0);
			break;
		case 1:
			if (testStick.getRawButton(5)) Move2.set(0.3);
			else if (testStick.getRawButton(6)) Move2.set(-0.3); else Move2.set(0);
			break;
		case 2:
			if (testStick.getRawButton(5)) Move3.set(0.3);
			else if (testStick.getRawButton(6)) Move3.set(-0.3); else Move3.set(0);
			break;
		case 3:
			if (testStick.getRawButton(5)) Move4.set(0.3);
			else if (testStick.getRawButton(6)) Move4.set(-0.3); else Move4.set(0);
			break;
		}*/
		
		/*if (Math.abs(testStick.getRawAxis(1)) > 0.2) {
			Move2.set(testStick.getRawAxis(1));
		} else {
			Move2.set(0);
		}*/
	}
}
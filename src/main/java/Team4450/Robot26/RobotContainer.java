package Team4450.Robot26;

import static Team4450.Robot26.Constants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;

import Team4450.Lib.Util;
import Team4450.Lib.XboxController;
import Team4450.Robot26.commands.DriveCommand;
import Team4450.Robot26.subsystems.DriveBase;
import Team4450.Robot26.subsystems.QuestNavSubsystem;
import Team4450.Robot26.subsystems.ShuffleBoard;
import Team4450.Robot26.subsystems.TestSubsystem;
import Team4450.Robot26.subsystems.VisionSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
	// Subsystems.
	public static ShuffleBoard shuffleBoard;
	public static DriveBase driveBase;
    public static VisionSubsystem visionSubsystem;
    public static QuestNavSubsystem questNavSubsystem;

	public final DriveCommand			 driveCommand;

    public TestSubsystem testSubsystem;

    // General todo list for Cole Pearson
    //
    // Remove any unused smartdashboard varuables
    // Remove the candle
    // Remove console logs that are causing radio issues
    //
    // Feat.
    //
    // Find accuracy of Limelight
    // Test Questnav
    // Find accuracy of Qeustnav
    // Average between the Limelight and Questnav
    //
    // Add a boolean to disable the controller rotation intput to the drivebase
    // Add a simple p-loop for heading targeting
    // Add a little more detail into the drivebase heading target
    //
    // Hud swival rotation tracking and right stick intake targeting
    //
    // While not driveing for a little while move to x stationary
    //
    // When in the middle swap from hub heading targeting to left or right of the hub targeting driving
    //
    // Shoot on the move ability
	




	// Subsystem Default Commands.

    // Persistent Commands.

	// Some notes about Commands.
	// When a Command is created with the New operator, its constructor is called. When the
	// command is added to the Scheduler to be run, its initialize method is called. Then on
	// each scheduler run, as long as the command is still scheduled, its execute method is
	// called followed by isFinished. If isFinished it false, the command remains in the
	// scheduler list and on next run, execute is called followed by isFinished. If isFinished
	// returns true, the end method is called and the command is removed from the scheduler list.
	// Now if you create another instance with new, you get the constructor again. But if you 
	// are re-scheduling an existing command instance (like the ones above), you do not get the
	// constructor called, but you do get initialize called again and then on to execute & etc.
	// So this means you have to be careful about command initialization activities as a persistent
	// command in effect has two lifetimes (or scopes): Class global and each new time the command
	// is scheduled. Note the FIRST doc on the scheduler process is not accurate as of 2020.
	
	// GamePads. 2 Game Pads use RobotLib XboxController wrapper class for some extra features.
	// Note that button responsiveness may be slowed as the schedulers command list gets longer 
	// or commands get longer as buttons are processed once per scheduler run.
	
	public static XboxController driverController =  new XboxController(DRIVER_PAD);
	public static XboxController utilityController = new XboxController(UTILITY_PAD);

	private PowerDistribution		pdp = new PowerDistribution(REV_PDB, PowerDistribution.ModuleType.kRev);

	// Trajectories we load manually.
	//public static PathPlannerTrajectory	ppTestTrajectory;

	private static SendableChooser<Command>	autoChooser;
	
	private static String 			autonomousCommandName = "none";

	private static PIDController headingPID;

	/**
	 * The container for the robot. Contains subsystems, Opertor Interface devices, and commands.
	 */
	public RobotContainer() throws Exception {
		Util.consoleLog();

        this.testSubsystem = new TestSubsystem();
		
	    SendableRegistry.addLW(pdp, "PDH"); // Only sent to NT in Test mode.

		// Get information about the match environment from the Field Control System.
      
		getMatchInformation();

		// Read properties file from RoboRio "disk". If we fail to open the file,
		// log the exception but continue and default to competition robot.
      
		try {
			robotProperties = Util.readProperties();
		} catch (Exception e) { Util.logException(e);}

		// Is this the competition or clone robot?
   		
		if (robotProperties == null || robotProperties.getProperty("RobotId").equals("comp"))
			isComp = true;
		else
			isClone = true;
 		
		// Invert driving joy sticks Y axis so + values mean forward.
		// Invert driving joy sticks X axis so + values mean right.
	  
		driverController.invertY(true);
		driverController.invertX(true);		

		// Create subsystems prior to button mapping.

		shuffleBoard = new ShuffleBoard();

        // The pigeon is setup somewhere in the drivebase function.
        // It is important to note that the pigeon documentation says that the device does not need to be still on boot,
        // however the documentation also says that the drift is worse when started while moving.
		driveBase = new DriveBase();
        visionSubsystem = new VisionSubsystem(driveBase);
        questNavSubsystem = new QuestNavSubsystem(driveBase);

		headingPID = new PIDController(Constants.ROBOT_HEADING_KP, Constants.ROBOT_HEADING_KI, Constants.ROBOT_HEADING_KD);

		// Create any persistent commands.

		// Set any subsystem Default commands.

		// Set the default drive command. This command will be scheduled automatically to run
		// every teleop period and so use the gamepad joy sticks to drive the robot. 

		// We pass the GetY() functions on the Joysticks as a DoubleSuppier. The point of this 
		// is removing the direct connection between the Drive and XboxController classes. We
		// are in effect passing functions into the Drive command so it can read the values
		// later when the Drive command is executing under the Scheduler. Drive command code does
		// not have to know anything about the JoySticks (or any other source) but can still read
		// them. We can pass the DoubleSupplier two ways. First is with () -> lambda expression
		// which wraps the getLeftY() function in a DoubleSupplier instance. Second is using the
		// controller class convenience method getRightYDS() which returns getRightY() as a 
		// DoubleSupplier. We show both ways here as an example.

		// The joystick controls for driving:
		// Left stick Y axis -> forward and backwards movement (throttle)
		// Left stick X axis -> left and right movement (strafe)
		// Right stick X axis -> rotation
		// Note: X and Y axis on stick is opposite X and Y axis on the WheelSpeeds object
		// and the odometry pose2d classes.
		// Wheelspeeds +X axis is down the field away from alliance wall. +Y axis is left
		// when standing at alliance wall looking down the field.
		// This is handled here by swapping the inputs. Note that first axis parameter below
		// is the X wheelspeeds input and the second is Y wheelspeeds input.

		// Note that field oriented driving does the movements in relation to the field. So
		// throttle is always down the field and back and strafe is always left right from
		// the down the field axis, no matter which way the robot is pointing. Robot oriented
		// driving movemments are in relation to the direction the robot is currently pointing.

		// Note that the controller instance is passed to the drive command for use in displaying
		// debugging information on Shuffleboard. It is not required for the driving function.

		driveCommand = new DriveCommand(driveBase,
		 							() -> driverController.getLeftY(),
									driverController.getLeftXDS(), 
									driverController.getRightXDS(),
									driverController.getRightYDS(),
									driverController, headingPID);
		driveBase.setDefaultCommand(driveCommand);

        // IDK if I have to init SmartDashboard data
        SmartDashboard.putNumber("Test Motor Power", 0);
		
		// Start a thread that will wait 30 seconds then disable the missing
		// joystick warning. This is long enough for when the warning is valid
		// but will stop flooding the console log when we are legitimately
		// running without both joysticks plugged in.

        new Thread(() -> {
            try {
                Timer.delay(30);    

                DriverStation.silenceJoystickConnectionWarning(true);
            } catch (Exception e) { }
        }).start();
        
        // Configure autonomous routines and send to dashboard.
		
		setAutoChoices();

		// Configure the button bindings.
		
        configureButtonBindings();
		
        // Warmup PathPlanner to avoid Java pauses.

		CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
		
		Util.consoleLog(functionMarker);
	}

	/**
	 * Use this method to define your button->command mappings.
     * 
     * These buttons are for robot driver station with 2 Xbox or F310 controllers.
	 */
	private void configureButtonBindings() {
		Util.consoleLog();
	  
		// ------- Driver controller buttons -------------
		
		// For simple functions, instead of creating commands, we can call convenience functions on
		// the target subsystem from an InstantCommand. It can be tricky deciding what functions
		// should be an aspect of the subsystem and what functions should be in Commands...

		// POV buttons do same as alternate driving mode but without any lateral
		// movement and increments of 45deg.
		// new Trigger(()-> driverController.getPOV() != -1)
		// 	.onTrue(new PointToYaw(()->PointToYaw.yawFromPOV(driverController.getPOV()), driveBase, false))

		// vibrate between 30 and 25 sec left in match.
		new Trigger(() -> Timer.getMatchTime() < 30 && Timer.getMatchTime() > 25).whileTrue(new StartEndCommand(
			() -> {
				driverController.setRumble(RumbleType.kBothRumble, 0.5);
				utilityController.setRumble(RumbleType.kBothRumble, 0.5);},
			() -> {
				driverController.setRumble(RumbleType.kBothRumble, 0);
				utilityController.setRumble(RumbleType.kBothRumble, 0);
		}));

		// holding top right bumper enables the alternate rotation mode in
		// which the driver points stick to desired heading.

		//new Trigger(() -> driverController.getRightBumperButton())
		//	.whileTrue(new PointToYaw(
		//		()->PointToYaw.yawFromAxes(
		//			-MathUtil.applyDeadband(driverController.getRightX(), Constants.DRIVE_DEADBAND),
		//			-MathUtil.applyDeadband(driverController.getRightY(), Constants.DRIVE_DEADBAND)
		//		), driveBase, false
		//));

		// Toggle slow-mode
		new Trigger(() -> driverController.getLeftBumperButton())  // Rich
		 	.onChange(new InstantCommand(driveBase::toggleSlowMode));

		// Reset field orientation (direction).
		new Trigger(() -> driverController.getStartButton()) // Rich
			.onTrue(new InstantCommand(driveBase::resetFieldOrientation));

		// Toggle field-oriented driving mode.
		// new Trigger(() -> driverController.getAButton()) // Rich
		//  	.onTrue(new InstantCommand(driveBase::toggleFieldRelativeDriving));

		new Trigger(() -> driverController.getAButton())
		 	.onTrue(new InstantCommand(testSubsystem::start));

		new Trigger(() -> driverController.getBButton())
		 	.onTrue(new InstantCommand(testSubsystem::stop));

		// // Toggle motor brake mode.
		// new Trigger(() -> driverController.getBButton()) // Rich
		//  	.onTrue(new InstantCommand(driveBase::toggleNeutralMode));

		// Right D-Pad button sets X pattern to stop movement.
		new Trigger(() -> driverController.getPOV() == 90) // Rich
			.onTrue(new InstantCommand(driveBase::setX));
			
		// -------- Utility controller buttons ----------

		// Driver controller A/B used for flywheel start/stop (TestSubsystem currently drives the motor)
		new Trigger(() -> driverController.getAButton())
			.onTrue(new InstantCommand(testSubsystem::start));

		new Trigger(() -> driverController.getBButton())
			.onTrue(new InstantCommand(testSubsystem::stop));

	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 * Determines which auto command from the selection made by the operator on the
	 * DS drop down list of commands.
	 * @return The Command to run in autonomous.
	 */
	public Command getAutonomousCommand() {
		// PathPlannerAuto  	ppAutoCommand;
		Command				autoCommand;

		autoCommand = autoChooser.getSelected();

		if (autoCommand == null) 
		{
			autonomousCommandName = "none";

			return autoCommand;
		}

		autonomousCommandName = autoCommand.getName();

		Util.consoleLog("auto name=%s", autonomousCommandName);

		if (autoCommand instanceof PathPlannerAuto)
		{
			// ppAutoCommand = (PathPlannerAuto) autoCommand;
	
			// Util.consoleLog("pp starting pose=%s", PathPlannerAuto.getStaringPoseFromAutoFile(autoCommand.getName().toString()));
		}

		return autoCommand;
  	}

	public static String getAutonomousCommandName()
	{
		return autonomousCommandName;
	}
  
    // Configure SendableChooser (drop down list on dashboard) with auto program choices and
	// send them to SmartDashboard/ShuffleBoard.
	
	private void setAutoChoices()
	{
	 	Util.consoleLog();
		
		// Register commands called from PathPlanner Autos.

		// Create a chooser with the PathPlanner Autos located in the PP deploy
		// folder.

	    autoChooser = AutoBuilder.buildAutoChooser();
		
    	SmartDashboard.putData("Auto Program", autoChooser);
	}

	/**
	 *  Get and log information about the current match from the FMS or DS.
	 */
	public void getMatchInformation()
	{
		alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
  	  	location = DriverStation.getLocation().orElse(0);
  	  	eventName = DriverStation.getEventName();
	  	matchNumber = DriverStation.getMatchNumber();
	  	gameMessage = DriverStation.getGameSpecificMessage();
    
	  	Util.consoleLog("Alliance=%s, Location=%d, FMS=%b event=%s match=%d msg=%s", 
    		  		   alliance.name(), location, DriverStation.isFMSAttached(), eventName, matchNumber, 
    		  		   gameMessage);
	}
		
	// public void fixPathPlannerGyro() { rich
	// 	driveBase.fixPathPlannerGyro();
	// }
}

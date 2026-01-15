package Team4450.Robot26.subsystems;

import static Team4450.Robot26.Constants.DriveConstants.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

import Team4450.Lib.Util;
import static Team4450.Robot26.Constants.*;
import Team4450.Robot26.Constants.DriveConstants;
import Team4450.Robot26.subsystems.SDS.CommandSwerveDrivetrain;
import Team4450.Robot26.subsystems.SDS.Telemetry;
import Team4450.Robot26.subsystems.SDS.TunerConstants;
import Team4450.Robot26.utility.AdvantageScope;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import Team4450.Robot26.utility.RobotOrientation;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * This class wraps the SDS drive base subsystem allowing us to add/modify drive base
 * functions without modifyinig the SDS code generated from Tuner. Also allows for
 * convenience wrappers for more complex functions in SDS code.
 */
public class DriveBase extends SubsystemBase {
    private CommandSwerveDrivetrain     sdsDriveBase = TunerConstants.createDrivetrain();

    public PigeonWrapper                gyro = new PigeonWrapper(sdsDriveBase.getPigeon2());
    public Pose2d robotPose = null;
    
    private final Telemetry     		logger = new Telemetry(kMaxSpeed);
    // Field2d object creates the field display on the simulation and gives us an API
    // to control what is displayed (the simulated robot).

    private final Field2d               field2d = new Field2d();
    
    private boolean                     fieldRelativeDriving = true, slowMode = false;
    private boolean                     neutralModeBrake = true;
    private double                      maxSpeed = kMaxSpeed * kDriveReductionPct; 
    private double                      maxRotRate = kMaxAngularRate * kRotationReductionPct;

    private boolean swivalTracking;
    private double headingTarget;

    private final SwerveRequest.SwerveDriveBrake driveBrake = new SwerveRequest.SwerveDriveBrake();

    private final SwerveRequest.FieldCentric driveField = new SwerveRequest.FieldCentric()
            .withDeadband(kMaxSpeed * DRIVE_DEADBAND)
            .withRotationalDeadband(kMaxAngularRate * ROTATION_DEADBAND) // Add deadbands
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.RobotCentric driveRobot = new SwerveRequest.RobotCentric()
            .withDeadband(kMaxSpeed * DRIVE_DEADBAND)
            .withRotationalDeadband(kMaxAngularRate * ROTATION_DEADBAND) // Add deadbands
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    public DriveBase() {
        Util.consoleLog();

        // Add gyro as a Sendable. Updates the dashboard heading indicator automatically.

		SmartDashboard.putData("Gyro2", gyro); 

        SmartDashboard.putData("Field2d", field2d);

		// Check Gyro.
	  
		if (gyro.getPigeon().isConnected())
			Util.consoleLog("Pigeon connected version=%s", gyro.getPigeon().getVersion());
		else
		{
			Exception e = new Exception("Pigeon is NOT connected!");
			Util.logException(e);
		}

        // Set drive motors to brake when power is zero.

        sdsDriveBase.configNeutralMode(NeutralModeValue.Brake);

		// Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        
		final var idle = new SwerveRequest.Idle();

        RobotModeTriggers.disabled().whileTrue(
            sdsDriveBase.applyRequest(() -> idle).ignoringDisable(true)
        );

        // Set tracking of robot field position at starting point. Blue perspective.
        // note that this doesn't really do much because PathPlanner redoes this anyway.
        // More for a starting pose in sim testing.

        resetOdometry(DriveConstants.DEFAULT_STARTING_POSE);

        // Under sim, we starting pose the robot (above) before you can change the alliance 
        // in the sim UI. We can't really do it anywhere else or it would interfere with 
        // transition from auto to teleop. So we pose robot at lower left (blue) corner and
        // force the blue driving perspective.

        if (RobotBase.isSimulation()) driveField.ForwardPerspective = ForwardPerspectiveValue.BlueAlliance;
		         
        // Register for SDS telemetry.
        sdsDriveBase.registerTelemetry(logger::telemeterize);

        updateDS();
    }

    @Override
    public void periodic() {
        sdsDriveBase.periodic();

        // update 3d simulation: look in AdvantageScope.java for more
        AdvantageScope.getInstance().setRobotPose(getPose());
        AdvantageScope.getInstance().update();
        AdvantageScope.getInstance().setSwerveModules(sdsDriveBase);

        // See this function for more information.
        updateModulePoses(sdsDriveBase);

        // Basic telemetry
        SmartDashboard.putNumber("Gyro angle", getYaw());
        SmartDashboard.putString("Robot od pose", getPose().toString());
        if (robotPose != null) {
            SmartDashboard.putString("Robot pose pose pose pose", robotPose.toString());
        }
    }

    public void drive(double throttle, double strafe, double rotation) {
        if (fieldRelativeDriving)
            sdsDriveBase.setControl(
                driveField.withVelocityX(throttle * maxSpeed) 
                        .withVelocityY(strafe * maxSpeed) 
                        .withRotationalRate(rotation * maxRotRate));
        else if (swivalTracking) {
            // Get the angle target to the hub
            sdsDriveBase.setControl(
                driveRobot.withVelocityX(throttle * maxSpeed) 
                        .withVelocityY(strafe * maxSpeed) 
                        .withRotationalRate(rotation * maxRotRate));
        }

        SmartDashboard.putNumber("Drive Velocity X", driveField.VelocityX);
        SmartDashboard.putNumber("Drive Velocity Y", driveField.VelocityY);
        SmartDashboard.putNumber("Drive Rot Rate", driveField.RotationalRate);
    }

    /**
     * Set drive wheels to X configuration to lock robot from moving.
     */
    public void setX()
    {
        Util.consoleLog();

        sdsDriveBase.applyRequest(() -> driveBrake);
    }
        
	public void toggleFieldRelativeDriving()
	{
		fieldRelativeDriving = !fieldRelativeDriving;

        Util.consoleLog("%b", fieldRelativeDriving);

        updateDS();
	}

    public void toggleSlowMode() 
    {
        slowMode = !slowMode;

        Util.consoleLog("%b", slowMode);

        if (slowMode)
        {
            // In slow mode, apply slow mode percentages.
            maxSpeed = kMaxSpeed * kSlowModeLinearPct;
            maxRotRate = kMaxAngularRate * kSlowModeRotationPct;
        }
        else
        {
            // In normal mode, apply reduction percentages.
            maxSpeed = kMaxSpeed * kDriveReductionPct;
            maxRotRate = kMaxAngularRate * kRotationReductionPct;
        }
    }

    public void toggleNeutralMode()
    {
        neutralModeBrake = !neutralModeBrake;

        Util.consoleLog("%b", neutralModeBrake);

        if (neutralModeBrake)
            sdsDriveBase.configNeutralMode(NeutralModeValue.Brake);
        else
            sdsDriveBase.configNeutralMode(NeutralModeValue.Coast);

        updateDS();
    }

    public void resetFieldOrientation()
    {
        Util.consoleLog();
        
        sdsDriveBase.seedFieldCentric();
    }

    /**
     * Sets the gyroscope yaw angle to zero. This can be used to set the direction
     * the robot is currently facing to the 'forwards' direction.
     */
    public void zeroGyro()
    {
        Util.consoleLog();

        gyro.reset();
    }
    
    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        Util.consoleLog(pose.toString());

        sdsDriveBase.resetPose(pose);

        setStartingGyroYaw(-pose.getRotation().getDegrees());
    }
    
    /**
     * Set a starting yaw for the case where robot is not starting
     * with back bumper parallel to the wall. 
     * @param degrees - is clockwise (cw or right).
     */
    public void setStartingGyroYaw(double degrees)
    {
        gyro.setStartingGyroYaw(degrees);
    }

    /**
     * Returns current pose of the robot.
     * @return Robot pose.
     */
    public Pose2d getPose()
    {
        return sdsDriveBase.getState().Pose;
    }

    public double getYaw()
    {
        return gyro.getYaw();
    }

    public double getYaw180()
    {
        return gyro.getYaw180();
    }

    // The PigeonWrapper is named gyro
    public RobotOrientation getRobotOrientation() { // IDK if this will really work, I'm not sure if get AngularVelocityX, Y, Z are the yaw, pitch, and roll rates
        return new RobotOrientation(gyro.pigeon.getYaw().getValueAsDouble(), gyro.pigeon.getAngularVelocityXWorld().getValueAsDouble(), gyro.pigeon.getPitch().getValueAsDouble(), gyro.pigeon.getAngularVelocityYDevice().getValueAsDouble(), gyro.pigeon.getRoll().getValueAsDouble(), gyro.pigeon.getAngularVelocityZWorld().getValueAsDouble());
    }

    private void updateDS()
    {
        SmartDashboard.putBoolean("Brakes", neutralModeBrake);
        SmartDashboard.putBoolean("Field Oriented", fieldRelativeDriving);
    }

    // AddVisionUpdate
    public void addVisionMeasurement(Pose2d pose, double timestampSeconds) {
        // Use the visionBuffer
        // Truncate vision buffer
        // Append current vision measurement
        // Replay vision poses
        // Remove any vision poses the break the laws of physics

        // Basic vision update that just sets the pose, this is good enough for testing if it is working
        robotPose = pose;
    }

    public double getAngleToAim(Pose2d targetPose) {
        Pose2d currentPose = getPose();
    
        double deltaX = targetPose.getX() - currentPose.getX();
        double deltaY = targetPose.getY() - currentPose.getY();

        double distance = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));

        double airTime;

        int lowerPointIndex = 0;    
        double lowerPoint = FLYWHEEL_SPEED_DISTANCE_TABLE[lowerPointIndex];

        int higherPointIndex = FLYWHEEL_SPEED_DISTANCE_TABLE.length - 1;
        double higherPoint = FLYWHEEL_SPEED_DISTANCE_TABLE[higherPointIndex];

        double currentDistance;
        for (int i = FLYWHEEL_SPEED_DISTANCE_TABLE.length - 2; i > 0; i--){
            currentDistance = FLYWHEEL_SPEED_DISTANCE_TABLE[i];
            if (currentDistance > distance) {
                if (currentDistance < higherPoint) {
                    higherPoint = currentDistance;
                    higherPointIndex = i;
                }
            } else if (currentDistance < distance) {
                if (currentDistance >= lowerPoint) {
                    lowerPoint = currentDistance;
                    lowerPointIndex = i;
                }
            } else if (currentDistance == distance) {
                airTime = FUEL_AIR_TIME_TABLE_SEC[i];
                break;
            }
        }

        double lowerTime = FUEL_AIR_TIME_TABLE_SEC[lowerPointIndex];
        double higherTime = FUEL_AIR_TIME_TABLE_SEC[higherPointIndex];

        airTime = lowerTime + ((higherTime - lowerTime) * (distance - lowerPoint) / (higherPoint - lowerPoint));

        double xVelocityOffset = ((driveField.VelocityX * airTime) / 100) / 2.54;
        double yVelocityOffset = ((driveField.VelocityY * airTime) / 100) / 2.54;
        
        deltaX += xVelocityOffset;
        deltaY += yVelocityOffset;

        double angleToAim = Math.toDegrees(Math.atan2(deltaY, deltaX));

        return angleToAim;
    }

    /**
     * Update the robot & swerve module displays on the "Field2d" field display in sim.
     * @param sdsDriveBase Reference to SDS drivebase class under our DriveBase class.
     * 
     * SDS code has it's own class for logging and doing the field display ("Pose") under sim 
     * and other data logged to the dashboard and to log files (Telemetry). 
     * 
     * We also have our own code for doing the field display and it draws the swerve modules 
     * and thier angles on the simulated robot. This can be handy so we are displaying both fields
     * for the time being. This function drives that display called Field2d under SmartDashboard. 
     * If this field display is not needed it can be commented out and just use the SDS Telemetry.
     */
    @SuppressWarnings("rawtypes")
    private void updateModulePoses(CommandSwerveDrivetrain sdsDriveBase)
    {
        Pose2d          modulePoses[] = new Pose2d[4], robotPose = getPose();

        Translation2d   moduleLocations[] = sdsDriveBase.getModuleLocations(), moduleLocation;

        SwerveModule    modules[] = sdsDriveBase.getModules();

        for (int i = 0; i < modules.length; i++)
        {
            moduleLocation = moduleLocations[i]
                .rotateBy(robotPose.getRotation())
                .plus(robotPose.getTranslation());

            modulePoses[i] = new Pose2d(moduleLocation, 
                                modules[i].getCurrentState().angle.plus(Rotation2d.fromDegrees(-getYaw180())));
        }

        field2d.getObject("Robot").setPose(robotPose);
        field2d.getObject("Swerve Modules").setPoses(modulePoses);
    }
}

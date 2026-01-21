package Team4450.Robot26;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.Properties;

import Team4450.Robot26.subsystems.SDS.TunerConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
	public static String		PROGRAM_NAME = "ORF26-01.09.26";

	public static Robot			robot;

	public static Properties	robotProperties;
	  
	public static boolean		isClone = false, isComp = false;
	    	
	public static DriverStation.Alliance	 alliance;
	public static int                        location, matchNumber;
	public static String					 eventName, gameMessage;
	    
    public static String                     functionMarker = "-".repeat(30);

    public static final double  ROBOT_PERIOD_SEC = .02;
    public static final int     ROBOT_PERIOD_MS = 20;

	// Non-drive base motor controller port assignments
    
    // Pneumatic valve controller port assignments.
    
    // CAMERAS 

    // The names of the cameras in the PhotonVision software

    // Limelight Constants:
	public static final double DONT_SEE_TAG_WAIT_TIME = 1;
	public static final double POSE_VALIDATION_TIME = 0.3;

    public static final int     REV_PDB = 20;
    public static final int     CTRE_CANDLE = 21;
    public static final int     PIGEON_ID = 1;
	
	// GamePad port assignments.
	public static final int		DRIVER_PAD = 0, UTILITY_PAD = 1;

    public static String LIMELIGHT_LEFT = "limelight-left";
    // Add limelight left offset
    public static String LIMELIGHT_RIGHT = "limelight-right";
    // Add limelight right offset

    public static double ROBOT_TO_QUEST_X = 0.304;
    public static double ROBOT_TO_QUEST_Y = 0;
    public static double ROBOT_TO_QUEST_Z = 0;

    public static int VISION_BUFFER_SIZE = 1;

    // Assume all field measurements are in meters
    // Field Limits (The Origin of the field should be the bottom left corner therefore all pose should be in +, +)
    public static double FIELD_MAX_X = 16.54;
    public static double FIELD_MAX_Y = 8.07;

    // HUB Positions (Center of the HUB)
    
    // THESE ARE ALL IN INCHES AS THAT IS WHAT THE FIELD IS DEFINED IN

    public static double HUB_BLUE_X = 4.625;
    public static double HUB_BLUE_Y = 4.034;
    // Red
    public static double HUB_RED_X = 9.375;
    public static double HUB_RED_Y = 4.034;

    // Blue
    // Comment out the not welded field
    // public static Pose2d HUB_BLUE_ANDYMARK_POSE = new Pose2d(4.611, 4.021, Rotation2d.kZero);
    public static Pose2d HUB_BLUE_WELDED_POSE = new Pose2d(4.625, 4.034, Rotation2d.kZero);
    // Red
    // public static Pose2d HUB_RED_ANDYMARK_POSE = new Pose2d(11.901, 4.021, Rotation2d.kZero);
    public static Pose2d HUB_RED_WELDED_POSE = new Pose2d(11.915, 4.034, Rotation2d.kZero);

    public static double NEUTRAL_BLUE_ZONE_BARRIER_X = 4.572;
    public static double NEUTRAL_RED_ZONE_BARRIER_X = 11.938;

    public static Pose2d FERRY_BLUE_OUTPOST_CORNER = new Pose2d(1.27, 0.635, Rotation2d.kZero);
    public static Pose2d FERRY_BLUE_BLANK_CORNER = new Pose2d(1.27, 7.407, Rotation2d.kZero);
    
    public static Pose2d FERRY_RED_OUTPOST_CORNER = new Pose2d(15.243, 7.407, Rotation2d.kZero);
    public static Pose2d FERRY_RED_BLANK_CORNER = new Pose2d(15.243, 0.635, Rotation2d.kZero);

    public static double FIELD_MIDDLE_Y = 4.021;

    public static double ROBOT_THROTTLE_KP = 0.012;
    public static double ROBOT_THROTTLE_KI = 0;
    public static double ROBOT_THROTTLE_KI_MAX = 0;
    public static double ROBOT_THROTTLE_KD = 0;

    public static double ROBOT_STRAFE_KP = 0.012;
    public static double ROBOT_STRAFE_KI = 0;
    public static double ROBOT_STRAFE_KI_MAX = 0;
    public static double ROBOT_STRAFE_KD = 0;

    public static double ROBOT_HEADING_KP = 0.04;
    public static double ROBOT_HEADING_KI = 0;
    public static double ROBOT_HEADING_KI_MAX = 0;
    public static double ROBOT_HEADING_KD = 0;
    // public static double ROBOT_HEADING_KF = 0;
    public static double ROBOT_HEADING_TOLERANCE_DEG = 0.5;
    // public static double ROBOT_HEADING_MAX_OUTPUT = 1;

    public static double FLYWHEEL_MAX_THEORETICAL_RPM = 4000;

    // Interpolation table
    public static double[] FLYWHEEL_SPEED_TABLE = {0.57, 0.595, 0.69, 0.715, 0.73, 0.82, 0.86};
    public static double[] FLYWHEEL_SPEED_DISTANCE_TABLE = {40, 56, 90, 95, 103, 127, 152};
    public static double[] FUEL_AIR_TIME_TABLE_SEC = {0.3, 0.35, 0.4, 0.45, 0.5, 0.55, 0.6};

    // ---------------- Turret motion profiling constants ---------------------------------
    // Turret defaults (internal units are degrees/sec and degrees/sec^2).
    // These are used as defaults; the runtime code exposes RPM-based tunables for convenience.
    // Max angular velocity for turret (degrees per second). Tune to your hardware limits.
    public static final double TURRET_MAX_VELOCITY_DEG_PER_SEC = 180.0;
    // Max angular acceleration for turret (degrees per second squared). Tune to your hardware limits.
    public static final double TURRET_MAX_ACCELERATION_DEG_PER_SEC2 = 360.0;
    // Default turret velocity/accel expressed in RPM units for dashboard convenience.
    // 1 rotation = 360 degrees, 1 RPM = 6 deg/sec
    public static final double TURRET_DEFAULT_MAX_VELOCITY_RPM = TURRET_MAX_VELOCITY_DEG_PER_SEC / 6.0; // 30 RPM
    public static final double TURRET_DEFAULT_MAX_ACCEL_RPMS = TURRET_MAX_ACCELERATION_DEG_PER_SEC2 / 6.0; // 60 RPM/s
    // Enable/disable acceleration smoothing (true = enabled)
    public static final boolean TURRET_ACCELERATION_ENABLED = true;
    // When within this many degrees, snap to setpoint and zero velocity.
    public static final double TURRET_ANGLE_TOLERANCE_DEG = 0.5;
    // -------------------------------------------------------------------------------------

    // Flywheel tuning defaults
    // Default target RPM for flywheel (used as a manual override/starting value)
    public static final double FLYWHEEL_DEFAULT_TARGET_RPM = 1845.0;
    // Default flywheel acceleration in RPM per second (used for ramping if implemented)
    public static final double FLYWHEEL_DEFAULT_ACCEL_RPMS = 20000.0;
    // Default open-loop start percent for flywheel when controlled by code only (0.0 - 1.0)
    public static final double FLYWHEEL_DEFAULT_START_PERCENT = 0.5; // 50% output

    // What is the LCD
	// LCD display line number constants showing class where the line is set.
	public static final int		LCD_1 = 1;	    // Robot, Auto Commands.
	public static final int		LCD_2 = 2;	    // Swerve Drive command.
	public static final int		LCD_3 = 3;	    // ShuffleBoard subsystem.
	public static final int		LCD_4 = 4;	    // ShuffleBoard subsystem.
	public static final int		LCD_5 = 5;	    // Autonomous commands.
	public static final int		LCD_6 = 6;	    // ShuffleBoard subsystem.
	public static final int		LCD_7 = 7;	    // ShuffleBoard subsystem.
	public static final int		LCD_8 = 8;	    // ShuffleBoard subsystem.
	public static final int		LCD_9 = 9;	    // ShuffleBoard subsystem.
	public static final int		LCD_10 = 10;	// ShuffleBoard subsystem.

    public static final class DriveConstants {
        // Driving Parameters - These are the maximum capable speeds of the robot.

        // Top speed determined by TunerX. Rotation speed reccommended by CTRE.
        // 2026 robot max speed is 5.29 m/s.
        public static double kMaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // top speed
        public static double kMaxAngularRate = RotationsPerSecond.of(1.0).in(RadiansPerSecond); // 1 rotation per second max angular velocity

        // Velocity dead bands applied in SDS code. Times max speed.
        public static final double  DRIVE_DEADBAND = 0.01, ROTATION_DEADBAND = 0.1;

        // Factors used to reduce robot max speed to levels desired for lab/demo operation.
        // The split below matches the rotation speed to drive speed. Needs to be tuned for
        // full weight robot.
        public static final double kDriveReductionPct = .50; // 50% of max linear speed.
        public static final double kRotationReductionPct = .70; // 70% of max rotational speed.

        // Factors used to slow robot speed for fine driving.
        public static final double kSlowModeLinearPct = .15; // 15% of max linear speed.
        public static final double kSlowModeRotationPct = .40; // 40% of max rotational speed.

        // Drive Motor ramp rate. Needs to be tuned for full weight robot.
        public static final double kDriveRampRate = .5; // 0 to 12v in .5 second.

        // Starting pose for sim. Is lower left corner (blue) or where we want sim robot to start.
        //public static final Pose2d	DEFAULT_STARTING_POSE = new Pose2d(7.473, .559, Rotation2d.kZero);
        public static final Pose2d	DEFAULT_STARTING_POSE = new Pose2d(0, 0, Rotation2d.kZero);
        public static final Pose3d	DEFAULT_STARTING_POSE_3D = new Pose3d(0, 0, 0, Rotation3d.kZero);
    }

  //-------------------- No student code above this line ------------------------------------------------------

}
;

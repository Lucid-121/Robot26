
package Team4450.Robot26;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.Properties;

import Team4450.Robot26.subsystems.SDS.TunerConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{
	public static String		PROGRAM_NAME = "ORF26-01.30.26";

	public static Robot			robot;

	public static Properties	robotProperties;
	  
	public static boolean		isClone = false, isComp = false, tracing = false;
	    	
	public static DriverStation.Alliance	 alliance;
	public static int                        location, matchNumber;
	public static String					 eventName, gameMessage;
	    
    public static String                     functionMarker = "-".repeat(30);

    public static final double  ROBOT_PERIOD_SEC = .02;
    public static final int     ROBOT_PERIOD_MS = 20;

	// Non-drive base motor controller port assignments
    
    // Pneumatic valve controller port assignments.
    
    // Cameras 

    // The names of the cameras in the PhotonVision software

    // Limelight Constants:
	public static final double DONT_SEE_TAG_WAIT_TIME = 1;
	public static final double POSE_VALIDATION_TIME = 0.3;

    // Misc port assignments.
    public static final int     REV_PDB = 20;
    public static final int     CTRE_CANDLE = 21;
    public static final int     PIGEON_ID = 1;

    // Canivore device name or "rio" if no canivore.
    public static String        CANIVORE_NAME = "canivore";
	
	// GamePad port assignments.
	public static final int		DRIVER_PAD = 0, UTILITY_PAD = 1;

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
    }

  //-------------------- No student code above this line ------------------------------------------------------

}
;
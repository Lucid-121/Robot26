package Team4450.Robot26.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import Team4450.Lib.LCD;
import Team4450.Lib.Util;
import Team4450.Robot26.subsystems.DriveBase;
import static Team4450.Robot26.Constants.*;
import Team4450.Robot26.utility.ConsoleEveryX;

public class DriveCommand extends Command 
{
    private final DriveBase         driveBase;

    private final DoubleSupplier    throttleSupplier;
    private final DoubleSupplier    strafeSupplier;
    private final DoubleSupplier    rotationXSupplier;
    private final DoubleSupplier    rotationYSupplier;
    private final XboxController    controller;

    public final PIDController     headingPID;

    public DriveCommand(DriveBase      driveBase,
                        DoubleSupplier throttleSupplier,
                        DoubleSupplier strafeSupplier,
                        DoubleSupplier rotationXSupplier,
                        DoubleSupplier rotationYSupplier,
                        XboxController controller, PIDController headingPID) 
    {
        Util.consoleLog();

        this.driveBase = driveBase;
        this.throttleSupplier = throttleSupplier;
        this.strafeSupplier = strafeSupplier;
        this.rotationXSupplier = rotationXSupplier;
        this.rotationYSupplier = rotationYSupplier;
        this.controller = controller;
        this.headingPID = headingPID;
        headingPID.setIntegratorRange(-ROBOT_HEADING_KI_MAX, ROBOT_HEADING_KI_MAX);
        headingPID.enableContinuousInput(-180, 180);
        headingPID.setTolerance(ROBOT_HEADING_TOLERANCE_DEG);

        addRequirements(driveBase);
    }

    @Override
    public void initialize()
    {
        Util.consoleLog();
    }

    @Override
    public void execute() 
    {
        LCD.printLine(2, "rx=%.3f  ry=%.3f  throttle=%.3f  strafe=%.3f  rotX=%.3f rotY=%.3f",
            controller.getRightX(),
            controller.getRightY(),
            throttleSupplier.getAsDouble(),
            strafeSupplier.getAsDouble(),
            rotationXSupplier.getAsDouble(),
            rotationYSupplier.getAsDouble()
        );

        LCD.printLine(3, "lx=%.3f  ly=%.3f", // yaw=%.3f",
            controller.getLeftX(),
            controller.getLeftY()
            //driveBase.getGyroRotation2d().getDegrees(),
            //driveBase.getGyroYaw() rich
        );

        // This is the default command for the DriveBase. When running in autonmous, the auto commands
        // require DriveBase, which preempts the default DriveBase command. However, if our auto code ends 
        // before end of auto period, then this drive command resumes and is feeding drivebase during remainder
        // of auto period. This was not an issue until the joystick drift problems arose, so the resumption of a 
        // driving command during auto had the robot driving randomly after our auto program completed. The if 
        // statment below prevents this.
        
        if (robot.isAutonomous()) return;

        Pose2d hubPosition;
        if (alliance == DriverStation.Alliance.Blue) {
            if (WELDED_FIELD) {
                hubPosition = new Pose2d(HUB_BLUE_WELDED_POSE.getX(), HUB_BLUE_WELDED_POSE.getY(), Rotation2d.kZero);
            } else {
                hubPosition = new Pose2d(HUB_BLUE_ANDYMARK_POSE.getX(), HUB_BLUE_ANDYMARK_POSE.getY(), Rotation2d.kZero);
            }
        } else {
            if (WELDED_FIELD) {
                hubPosition = new Pose2d(HUB_RED_WELDED_POSE.getX(), HUB_RED_WELDED_POSE.getY(), Rotation2d.kZero);
            } else {
                hubPosition = new Pose2d(HUB_RED_ANDYMARK_POSE.getX(), HUB_RED_ANDYMARK_POSE.getY(), Rotation2d.kZero);
            }
        }
        
        double targetHeading;
        if (rotationXSupplier.getAsDouble() == 0 && rotationYSupplier.getAsDouble() == 0 && alliance == DriverStation.Alliance.Blue) {
            if (driveBase.robotPose.getX() < NEUTRAL_BLUE_ZONE_BARRIER_X) {
                targetHeading = driveBase.getAngleToAim(hubPosition);
            } else {
                if (driveBase.robotPose.getY() < FIELD_MIDDLE_Y) {
                    targetHeading = driveBase.getAngleToAim(FERRY_BLUE_OUTPOST_CORNER);
                } else {
                    targetHeading = driveBase.getAngleToAim(FERRY_BLUE_BLANK_CORNER);
                }
            }
        } else if (rotationXSupplier.getAsDouble() == 0 && rotationYSupplier.getAsDouble() == 0 && alliance == DriverStation.Alliance.Red) {
            if (driveBase.robotPose.getX() > NEUTRAL_RED_ZONE_BARRIER_X) {
                targetHeading = driveBase.getAngleToAim(hubPosition);
            } else {
                if (driveBase.robotPose.getY() < FIELD_MIDDLE_Y) {
                    targetHeading = driveBase.getAngleToAim(FERRY_RED_BLANK_CORNER);
                } else {
                    targetHeading = driveBase.getAngleToAim(FERRY_RED_OUTPOST_CORNER);
                }
            }
        } else {
            targetHeading = Math.toDegrees(Math.atan2(rotationYSupplier.getAsDouble(), rotationXSupplier.getAsDouble()));
        }

        double rotation = headingPID.calculate(driveBase.getYaw180(), targetHeading);
        double throttle = throttleSupplier.getAsDouble();
        double strafe = strafeSupplier.getAsDouble();

        // Squaring input is one way to ramp JS inputs to reduce sensitivity.
        // Please do not square the headingPID
        
        // throttle = Util.squareInput(throttle);
        // strafe = Util.squareInput(strafe);
        // rotation = Util.squareInput(rotation);
        // rotation = Math.pow(rotation, 5);
        //
        ConsoleEveryX rotationLog = new ConsoleEveryX(200);
        rotationLog.update("Heading PID rotation output: " + String.valueOf(rotation));

        driveBase.drive(throttle, strafe, rotation);
    }

    @Override 
    public void end(boolean interrupted) {
        Util.consoleLog("interrupted=%b", interrupted);
    }
}

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
import Team4450.Robot26.subsystems.Drivebase;
import static Team4450.Robot26.Constants.*;
import Team4450.Robot26.utility.ConsoleEveryX;

public class PIDDriveCommand extends Command 
{
    private final Drivebase driveBase;

    public final PIDController throttlePID;
    public final PIDController strafePID;
    public final PIDController headingPID;

    public double targetX;
    public double targetY;
    public double targetHeading;

    public PIDDriveCommand(Drivebase driveBase, PIDController throttlePID, PIDController strafePID, PIDController headingPID, double targetX, double targetY, double targetHeading) 
    {
        Util.consoleLog();

        this.driveBase = driveBase;
        this.throttlePID = throttlePID;
        this.strafePID = strafePID;
        this.headingPID = headingPID;

        this.targetX = targetX;
        this.targetY = targetY;
        this.targetHeading = targetHeading;

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
        if (robot.isAutonomous()) return;
        
        double throttle = throttlePID.calculate(driveBase.getPose().getX(), targetX);
        double strafe = strafePID.calculate(driveBase.getPose().getY(), targetY);
        double rotation = headingPID.calculate(driveBase.getYaw180(), targetHeading);
        driveBase.drive(throttle, strafe, rotation);
    }

    @Override 
    public void end(boolean interrupted) {
        Util.consoleLog("interrupted=%b", interrupted);
    }
}

package Team4450.Robot26.subsystems;

import static Team4450.Robot26.Constants.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Turret extends SubsystemBase {
    // Requested target (degrees) — set by callers
    private double requestedAngleDeg = 0.0;
    // Commanded angle we are currently outputting to hardware (degrees)
    private double commandedAngleDeg = 0.0;
    // Current commanded angular velocity (deg/sec)
    private double commandedAngularVelocity = 0.0;
    // Tunable motion parameters (initialized from Constants but editable at runtime)
    // Internals use deg/sec and deg/sec^2. For convenience we expose RPM units on the dashboard
    // and convert to degrees internally (1 RPM = 6 deg/sec).
    private double turretMaxVelDegPerSec = TURRET_MAX_VELOCITY_DEG_PER_SEC;
    private double turretMaxAccelDegPerSec2 = TURRET_MAX_ACCELERATION_DEG_PER_SEC2;
    private boolean turretAccelEnabled = TURRET_ACCELERATION_ENABLED;
    // Flywheel runtime tunables (RPM and RPM/s units on dashboard)
    // (flywheel is currently controlled by TestSubsystem; no dashboard-driven flywheel tunables here)
    
    private final DriveBase driveBase;

    public Turret(DriveBase driveBase) {
        // initialize commanded angle to whatever a reasonable default is
        this.commandedAngleDeg = 0.0;
        this.requestedAngleDeg = 0.0;
        this.commandedAngularVelocity = 0.0;
        this.driveBase = driveBase;

        // Publish tuning values to SmartDashboard so they can be changed while testing.
        // Publish both RPM-based and internal values for clarity/editing.
        SmartDashboard.putNumber("Turret/MaxVelocityRPM", TURRET_DEFAULT_MAX_VELOCITY_RPM);
        SmartDashboard.putNumber("Turret/MaxAccelerationRPMperSec", TURRET_DEFAULT_MAX_ACCEL_RPMS);
        SmartDashboard.putBoolean("Turret/AccelEnabled", turretAccelEnabled);

    // Flywheel is controlled by TestSubsystem (code-controlled). No dashboard defaults here.
    }

    @Override
    public void periodic() {
    // Read potentially-updated tuning values from SmartDashboard (RPM-based entries).
    double hudTurretMaxVelRpm = SmartDashboard.getNumber("Turret/MaxVelocityRPM", TURRET_DEFAULT_MAX_VELOCITY_RPM);
    double hudTurretMaxAccelRpms = SmartDashboard.getNumber("Turret/MaxAccelerationRPMperSec", TURRET_DEFAULT_MAX_ACCEL_RPMS);
    turretAccelEnabled = SmartDashboard.getBoolean("Turret/AccelEnabled", turretAccelEnabled);


    // Convert RPM-based dashboard values to internal deg/sec units: 1 RPM = 360 deg / 60 sec = 6 deg/sec
    turretMaxVelDegPerSec = hudTurretMaxVelRpm * 6.0;
    turretMaxAccelDegPerSec2 = hudTurretMaxAccelRpms * 6.0;

    // Flywheel is controlled by TestSubsystem via Constants; no dashboard reads here.

        // Update the acceleration-limited motion profile for the turret every robot cycle.
        updateMotion(ROBOT_PERIOD_SEC);
    }

    public void aimTurret(Pose2d robotPosition) {
        setTargetAngle(getAngleToFaceGoalDegrees(robotPosition));

        double targetFlywheelSpeed = getNeededFlywheelSpeed(driveBase.getDistFromRobot(driveBase.getPoseToAim(getGoalPose())));
        setFlywheelSpeed(targetFlywheelSpeed);
    }

    public void setTargetAngle(double angleInSomething) {
        // Public API: callers request a new target angle in degrees. We store it and
        // the periodic loop will ramp the commanded output toward this target using
        // the acceleration/velocity limits in Constants.
        this.requestedAngleDeg = angleInSomething;
    }

    public void setFlywheelSpeed(double targetFlywheelSpeed) {
        // For future integration: apply a flywheel RPM setpoint. Currently a stub.
        applyFlywheelRpm(targetFlywheelSpeed);
    }

    public double getNeededFlywheelSpeed(double distToGoal) {
        double targetVelocity = interpolateFlywheelSpeedByDistance(distToGoal);
        return targetVelocity * FLYWHEEL_MAX_THEORETICAL_RPM; // Normalize the target velocity by the max theoretical
    }

    public Pose2d getGoalPose() {
        // If blue side
        if (alliance == Alliance.Blue) {
            return HUB_BLUE_WELDED_POSE;
        // If red side
        } else if (alliance == Alliance.Red) {
            return HUB_RED_WELDED_POSE;
        } else {
            return null; // Error
        }
    }

    public double getAngleToFaceGoalDegrees(Pose2d robotPosition) {
        // If blue side
        double xDiff = 0;
        double yDiff = 0;
        if (alliance == Alliance.Blue) {
            xDiff = HUB_BLUE_WELDED_POSE.getX() - robotPosition.getX();
            yDiff = HUB_BLUE_WELDED_POSE.getY() + robotPosition.getY();
        // If red side
        } else if (alliance == Alliance.Red) {
            xDiff = HUB_RED_WELDED_POSE.getX() - robotPosition.getX();
            yDiff = HUB_RED_WELDED_POSE.getY() - robotPosition.getY();
        } else {
            // Error
        }
        // Simplify this stuff
        return (Math.toDegrees(Math.atan(yDiff / xDiff)) + robotPosition.getRotation().getDegrees() - 180);
    }

    public double interpolateFlywheelSpeedByDistance(double distToGoal) {
        double lowerPoint = FLYWHEEL_SPEED_DISTANCE_TABLE[0];

        int lowerPointIndex = 0;

        double higherPoint = FLYWHEEL_SPEED_DISTANCE_TABLE[FLYWHEEL_SPEED_DISTANCE_TABLE.length - 1];
        int higherPointIndex = FLYWHEEL_SPEED_DISTANCE_TABLE.length - 1;

        double currentDistance;
        for (int i = FLYWHEEL_SPEED_DISTANCE_TABLE.length - 2; i > 0; i--){
            currentDistance = FLYWHEEL_SPEED_TABLE[i];
            if(currentDistance > distToGoal){
                if (currentDistance < higherPoint) {
                    higherPoint = currentDistance;
                    higherPointIndex = i;
                }
            }else if (currentDistance < distToGoal){
                if (currentDistance >= lowerPoint) {
                    lowerPoint = currentDistance;
                    lowerPointIndex = i;
                }
            }else if (currentDistance == distToGoal){
                return FLYWHEEL_SPEED_TABLE[i];
            }
        }
        double lowerSpeed = FLYWHEEL_SPEED_TABLE[lowerPointIndex];
        double higherSpeed = FLYWHEEL_SPEED_TABLE[higherPointIndex];

        return linearInterpolate(lowerSpeed, higherSpeed, (distToGoal - lowerPoint) / (higherPoint - lowerPoint));
    }

    public static double linearInterpolate(double point1, double point2, double percentageSplit) {
        return point1 + ((point2 - point1) * percentageSplit);
    }

    // --- Motion profiling helpers ----------------------------------------------------
    private void updateMotion(double dt) {
        if (dt <= 0) return;

        if (!turretAccelEnabled) {
            // No smoothing; jump straight to requested setpoint.
            commandedAngleDeg = requestedAngleDeg;
            commandedAngularVelocity = 0.0;
            applyHardwareAngle(commandedAngleDeg);
            return;
        }

        // Compute shortest-path angular error in degrees [-180,180)
        double error = wrapAngleDeg(requestedAngleDeg - commandedAngleDeg);

        // Desired velocity to close the error in one iteration (may be large); clamp to max velocity
    double desiredVel = clamp(error / dt, -turretMaxVelDegPerSec, turretMaxVelDegPerSec);

        // Limit acceleration: compute allowable change in velocity
    double maxDeltaV = turretMaxAccelDegPerSec2 * dt;
        double deltaV = desiredVel - commandedAngularVelocity;
        if (deltaV > maxDeltaV) deltaV = maxDeltaV;
        if (deltaV < -maxDeltaV) deltaV = -maxDeltaV;

        commandedAngularVelocity += deltaV;

        // Integrate angle
        commandedAngleDeg += commandedAngularVelocity * dt;

        // If we're very close to target, snap to it and zero velocity to avoid jitter.
        if (Math.abs(wrapAngleDeg(requestedAngleDeg - commandedAngleDeg)) <= TURRET_ANGLE_TOLERANCE_DEG) {
            commandedAngleDeg = requestedAngleDeg;
            commandedAngularVelocity = 0.0;
        }

        applyHardwareAngle(commandedAngleDeg);
    }

    // Convert a raw angle to the range [-180, 180)
    private static double wrapAngleDeg(double angle) {
        angle = angle % 360.0;
        if (angle < -180.0) angle += 360.0;
        if (angle >= 180.0) angle -= 360.0;
        return angle;
    }

    private static double clamp(double v, double lo, double hi) {
        if (v < lo) return lo;
        if (v > hi) return hi;
        return v;
    }

    // This method should perform the actual hardware call to move the turret to the
    // specified angle (in degrees). Replace the body with your motor controller call
    // converting degrees to whatever encoder/motor units you use.
    private void applyHardwareAngle(double angleDeg) {
        // Example placeholder:
        // turretMotor.setPositionDegrees(angleDeg);
        // For now, just leave a debug comment or log if needed.
        // System.out.println("Turret angle command: " + angleDeg);
    }

    // Flywheel hardware application stub. This will ramp target RPM with accel if you
    // want to implement a flywheel ramp. Right now it simply calls the placeholder.
    private void applyFlywheelRpm(double rpm) {
        // Replace with your motor controller speed command, e.g.:
        // flywheelMotor.setVelocityRpm(rpm);
    }

    // Public setters/getters for programmatic tuning if you prefer not to use SmartDashboard
    public void setTurretMaxVelocityRpm(double rpm) { turretMaxVelDegPerSec = rpm * 6.0; SmartDashboard.putNumber("Turret/MaxVelocityRPM", rpm); }
    public void setTurretMaxAccelerationRpms(double rpms) { turretMaxAccelDegPerSec2 = rpms * 6.0; SmartDashboard.putNumber("Turret/MaxAccelerationRPMperSec", rpms); }
    public double getTurretMaxVelocityRpm() { return turretMaxVelDegPerSec / 6.0; }
    public double getTurretMaxAccelerationRpms() { return turretMaxAccelDegPerSec2 / 6.0; }

    // Flywheel dashboard/setters removed; flywheel is code-controlled in TestSubsystem.
}

package Team4450.Robot26.subsystems;

import static Team4450.Robot26.Constants.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Shooter extends SubsystemBase {
    

    private final Drivebase drivebase;
    DigitalInput beamBreak;

    // Constants for launch calculations
    private static final double GRAVITY = 9.81;
    private static final double DESIRED_MAX_HEIGHT = 2.5; // meters (8.2 feet)
    private static final double GOAL_HEIGHT = 1.8288; // meters (6 feet)
    private static final double FLYWHEEL_HEIGHT = 0.5334; // meters (21 inches)
    private static final double CONVERSION_FACTOR_MPS_TO_RPM = 10000 / 47.93;

    public Shooter(Drivebase drivebase) {
        // initialize commanded angle to whatever a reasonable default is
        this.drivebase = drivebase;
        beamBreak = new DigitalInput(3);
    }

    @Override
    public void periodic() {
        // Flywheel is controlled by TestSubsystem via Constants; no dashboard reads here.
        
        //This line should be all that is needed when the flywheel should be spun up
        //updateLaunchValues(true);

        //Update the beam break sensors
        SmartDashboard.putBoolean("Beam Break", beamBreak.get());
    }

    public void updateLaunchValues(boolean interpolate){
        // Calculate distance to goal & diffs
        double xDiff = getGoalPose().getX() - drivebase.getPose().getX();
        double yDiff = getGoalPose().getY() - drivebase.getPose().getY();
        double distToGoal = Math.sqrt(Math.pow(xDiff, 2) + Math.pow(yDiff, 2));
        
        // Chose how to get the flywheel speed
        if (interpolate){
            getNeededFlywheelSpeed(distToGoal);
        } else {
            calculateLaunchValues(distToGoal);
        }
    }

    public void calculateLaunchValues(double distToGoal){
        double desiredHeight = distToGoal > 6 ? DESIRED_MAX_HEIGHT : distToGoal / 6;

        // Calculate time of flight and velocity vectors
        double verticalVel = Math.sqrt(2 * GRAVITY * (desiredHeight - FLYWHEEL_HEIGHT));
        double estimatedTime = (verticalVel + Math.sqrt(verticalVel - 2 * (GRAVITY) * (GOAL_HEIGHT - FLYWHEEL_HEIGHT))) / GRAVITY;
        double horizonalVel = distToGoal / estimatedTime;

        // Calculate hood angle and angle to face goal
        double hoodAngle = Math.atan(verticalVel / horizonalVel);
        //double angleToFaceGoal = Math.atan2(yDiff, xDiff); I think this is done in a different file
        
        // Calculate flywheel RPM needed
        double initialVel = Math.sqrt(Math.pow((verticalVel), 2) * Math.pow((horizonalVel), 2));
        double targetRpm = initialVel * CONVERSION_FACTOR_MPS_TO_RPM;
        
        // Set the flywheel Velocity & Hood angle
        setFlywheelSpeed(targetRpm);
        setHoodAngle(hoodAngle);
    }

    public void setFlywheelSpeed(double targetFlywheelSpeed) {
        // For future integration: apply a flywheel RPM setpoint. Currently a stub.

        // Whatever the harware call is here
        //setFlywheelSpeed(targetFlywheelSpeed);
    }

    public void setHoodAngle(double targetAngle){
        // Whatever the harware call is here
        //setHoodAngle(targetAngle);
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
        // Find the difference betweeen the robot and the goal
        double xDiff = getGoalPose().getX() - drivebase.getPose().getX();
        double yDiff = getGoalPose().getY() - drivebase.getPose().getY();

        // use atan2 to get teh correct angle to face goal & convert to degrees
        double angleToFaceGoal = Math.toDegrees(Math.atan(yDiff / xDiff));

        return ( angleToFaceGoal - robotPosition.getRotation().getDegrees());
    }

    // This method sets the lower and higher points to interpolate between 
    // Based on the hardcoded flywheel speed & distance tables and the robots current distance away from the goal
    public double interpolateFlywheelSpeedByDistance(double distToGoal) {
        
        double lowerPoint = FLYWHEEL_SPEED_DISTANCE_TABLE[0];
        int lowerPointIndex = 0;

        double higherPoint = FLYWHEEL_SPEED_DISTANCE_TABLE[FLYWHEEL_SPEED_DISTANCE_TABLE.length - 1];
        int higherPointIndex = FLYWHEEL_SPEED_DISTANCE_TABLE.length - 1;

        double currentDistance;

        for (int i = FLYWHEEL_SPEED_DISTANCE_TABLE.length - 2; i > 0; i--){
            currentDistance = FLYWHEEL_SPEED_TABLE[i];
            if(currentDistance > distToGoal){
                if (currentDistance <= higherPoint) {
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
}

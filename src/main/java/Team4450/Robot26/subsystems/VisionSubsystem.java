package Team4450.Robot26.subsystems;

import Team4450.Lib.Util;
import Team4450.Robot26.Constants;
import Team4450.Robot26.utility.RobotOrientation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.lang.Math;
import Team4450.Robot26.utility.VisionBuffer;
import Team4450.Robot26.utility.VisionPose;
import edu.wpi.first.math.geometry.Pose2d;

public class VisionSubsystem extends SubsystemBase {
    // Info from: https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltags
    //
    // TODO: We need to aim the Limelight upwards at an angle when the april tag is as centered as possible at 10-20 feet or so (Whatever the average sight distance will be)
    //
    // The tag collection family in "AprilTag Classic 36h11
    // The Limelight offset from robot pose must be set in the web view for each Limelight
    //
    // The Coordinate plane for Limelight is as follows
    // https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-coordinate-systems

    // Speed tips
    //
    // Increase detector downscale for insteased framerate without affecting 3D accuracy
    // Lower brightness and contrast values will improve pipeline framerate at the cost of range
    // If the Limelight has a large area of the frame that will never have a tag, then the area that the april tag algo will be reduced and result in huge performance
    //
    // MegaTag 2
    //
    // The heading of the robot is required to use this, Idealy pas a complete robot orientation and angular velocities
    // LimelightHelpers.SetRobotOrientation(robotYawInDegrees,0,0,0,0,0))
    // SetRobotOrientation assumes a centered (see the map generator) or blue-corner origin. CCW-positive, 0 degrees -> facing red alliance wall in FRC.
    //
    DriveBase drivebase;
    int i = 0;

    public VisionSubsystem(DriveBase drivebase) {

        // Need to add a null check for the cameras
        this.drivebase = drivebase;
        // Init Left and Right Limelight
        //
        // To use the Limelight 4 built in IMU to get even better MegaTag 2 updates
        // 
        // For each
        // Firstly, run SetRobotOrientation()
        // Next, run SetIMUMode() // Use mode 2 when enabled and mode 1 when disabled, so put this in a disable function
        //
        
        RobotOrientation rO = drivebase.getRobotOrientation(); // IDK if RobotOrientation works correctly, look there to see
        
        zeroLimelightIMU(rO);

        Util.consoleLog("Init Limelight Left");
        LimelightHelpers.SetRobotOrientation(Constants.LIMELIGHT_LEFT, rO.yaw, rO.yawRate, rO.pitch, rO.pitchRate, rO.roll, rO.rollRate);
        Util.consoleLog("Init Limelight Right");
        LimelightHelpers.SetRobotOrientation(Constants.LIMELIGHT_RIGHT, rO.yaw, rO.yawRate, rO.pitch, rO.pitchRate, rO.roll, rO.rollRate);
        // IMU mode 2 uses to Limelight 4 internal IMU
        LimelightHelpers.SetIMUMode(Constants.LIMELIGHT_LEFT, 2);
        LimelightHelpers.SetIMUMode(Constants.LIMELIGHT_RIGHT, 2);

    }

    @Override
    public void periodic() {
        // How to stpo this when disabled
        i++;
        boolean useLeftLimelight = true;
        boolean useRightLimelight = true;
        // Get latest pose estimage from each camera
        
        LimelightHelpers.PoseEstimate left_mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.LIMELIGHT_LEFT);
        // Pose2d left_mt2 = LimelightHelpers.getBotPose2d_wpiBlue(Constants.LIMELIGHT_LEFT);
        LimelightHelpers.PoseEstimate right_mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.LIMELIGHT_RIGHT);
        // Pose2d right_mt2 = LimelightHelpers.getBotPose2d_wpiBlue(Constants.LIMELIGHT_RIGHT);

        // If the angular velocity is greater than 720 degrees per second ignore the vision update
        //
        // IDK if this is yaw rate or what units this is in
        if (Math.abs(drivebase.gyro.pigeon.getAngularVelocityXDevice().getValueAsDouble()) > 720) {
            return;
        }

        // Get rid of any result that says we are out of the field

        // IDK what units the getX() return
        // IDK what units the getY() return
        
        if (left_mt2 != null) {
            if (i % 200 == 0) {
                // Util.consoleLog("Left" + String.valueOf(left_mt2.getX()));
                Util.consoleLog("Left Tags: " + String.valueOf(left_mt2.rawFiducials.length));
                if (Math.abs(left_mt2.pose.getX()) > Constants.FIELD_MAX_X) {
                    useLeftLimelight = false;
                    Util.consoleLog("Left Outside max x");
                }

                if (Math.abs(left_mt2.pose.getY()) > Constants.FIELD_MAX_Y) {
                    useLeftLimelight = false;
                    Util.consoleLog("Left Outside max y");
                }

                if (left_mt2.rawFiducials.length < 1) {
                    useLeftLimelight = false;
                    Util.consoleLog("Left not no good raw fids");
                }

                if (useLeftLimelight) {
                    Util.consoleLog("Add left vision");
                    drivebase.addVisionMeasurement(left_mt2.pose, left_mt2.timestampSeconds);
                }
            }
            if (Math.abs(left_mt2.pose.getX()) > Constants.FIELD_MAX_X) {
                useLeftLimelight = false;
            }

            if (Math.abs(left_mt2.pose.getY()) > Constants.FIELD_MAX_Y) {
                useLeftLimelight = false;
            }

            if (left_mt2.rawFiducials.length < 1) {
                useLeftLimelight = false;
            }

            if (useLeftLimelight) {
                Util.consoleLog("Add left vision");
                drivebase.addVisionMeasurement(left_mt2.pose, left_mt2.timestampSeconds);
            }
        }

        if (right_mt2 != null) {
            if (i % 200 == 0) {
                // Util.consoleLog("Right" + String.valueOf(right_mt2.getX()));
                Util.consoleLog("Right Tags: " + String.valueOf(right_mt2.rawFiducials.length));
            }
            if (Math.abs(right_mt2.pose.getX()) > Constants.FIELD_MAX_X) {
                useRightLimelight = false;
            }

            if (Math.abs(right_mt2.pose.getY()) > Constants.FIELD_MAX_Y) {
                useRightLimelight = false;
            }

            if (right_mt2.rawFiducials.length < 1) {
                useRightLimelight = false;
            }

            if (useRightLimelight) {
                drivebase.addVisionMeasurement(right_mt2.pose, right_mt2.timestampSeconds);
            }
        }


        // Get rid of the result if the yaw of the resulting pose is impossible
        //
        // I think the yaw is between -180 and 180 instead of 0 - 360
        // if (left_mt2.pose.getRotation().getDegrees() < 0 || left_mt2.pose.getRotation().getDegrees() > 360 || right_mt2.pose.getRotation().getDegrees() < 0 || right_mt2.pose.getRotation().getDegrees() > 360) {
        //     return;
        // }
        
    }

    public void zeroLimelightIMU(RobotOrientation rO) { // Set to IMU mode 0 to diable the internal limelight IMU
        // Setting to IMU mode 1 will use the setRobotOrientation to set the internal Limelight IMU
        LimelightHelpers.SetIMUMode(Constants.LIMELIGHT_LEFT, 1);
        LimelightHelpers.SetIMUMode(Constants.LIMELIGHT_RIGHT, 1);
        
        LimelightHelpers.SetRobotOrientation(Constants.LIMELIGHT_LEFT, rO.yaw, rO.yawRate, rO.pitch, rO.pitchRate, rO.roll, rO.rollRate);
        LimelightHelpers.SetRobotOrientation(Constants.LIMELIGHT_RIGHT, rO.yaw, rO.yawRate, rO.pitch, rO.pitchRate, rO.roll, rO.rollRate);
    }
}

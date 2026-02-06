package Team4450.Robot26.subsystems;

import Team4450.Robot26.Constants;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;
import Team4450.Lib.Util;
import Team4450.Robot26.utility.ConsoleEveryX;

public class QuestNavSubsystem extends SubsystemBase {
    QuestNav questNav;
    Transform3d ROBOT_TO_QUEST = new Transform3d(Constants.ROBOT_TO_QUEST.getX(), Constants.ROBOT_TO_QUEST.getY(), Constants.ROBOT_TO_QUEST.getZ(), Constants.ROBOT_TO_QUEST.getRotation());
    
    final Pose3d nullPose = new Pose3d(-1, -1, -1, Rotation3d.kZero);
    final Pose3d zeroPose = new Pose3d(0, 0, 0, Rotation3d.kZero);

    ConsoleEveryX questTestLogger = new ConsoleEveryX("Quest Test Logger", 100);
    ConsoleEveryX questLogger = new ConsoleEveryX("Quest Logger", 100);
    ConsoleEveryX limelightWarnLogger = new ConsoleEveryX("Limelight Warn Logger", 1000);

    PoseFrame[] poseFrames;

    /** Creates a new QuestNavSubsystem. */
    private Drivebase drivebase;
    private double resetTimer = 0; // Change this to a real timer at some point;
    public QuestNavSubsystem(Drivebase drivebase) {
        this.drivebase = drivebase;
        questNav = new QuestNav();

        this.resetTimer = 0;

        resetToZeroPose();
    }

    public void resetToZeroPose() {
        Pose3d questPose3d = zeroPose.transformBy(ROBOT_TO_QUEST);
        // Because the pose is set on the quest nav we will need to store an offset that is updated by the limelight in the drivebase class because we do not want to send constant updates to the questnav system
        questNav.setPose(questPose3d);
        System.out.println("QuestNav internal pose reset to: " + questPose3d.toString());
    }

    public Pose3d getQuestRobotPose() {
        return (poseFrames != null && poseFrames.length > 0) ?
            poseFrames[poseFrames.length - 1].questPose3d()
            .transformBy(ROBOT_TO_QUEST.inverse()) : nullPose;
    }

    public double getQTimeStamp() {
        return (poseFrames != null && poseFrames.length > 0) ?
            poseFrames[poseFrames.length - 1].dataTimestamp() : 0;
    }

    public double getQAppTimeStamp() {
        return (poseFrames != null && poseFrames.length > 0) ?
            poseFrames[poseFrames.length - 1].appTimestamp() : 0;
    }

    public Pose3d getQuestPose() {
        return (poseFrames != null && poseFrames.length > 0) ?
            poseFrames[poseFrames.length - 1].questPose3d() : nullPose;
    }

    public void resetQuestOdometry(Pose3d rP) {
        // Transform by the offset to get the Quest pose
        Pose3d questPose3d = rP.transformBy(ROBOT_TO_QUEST);

        // Send the reset operation
        questNav.setPose(questPose3d);
        Util.consoleLog("Quest Odometry Reset To: " + questPose3d.toString());
        Util.consoleLog("QRP: " + rP.toString());
    }

    public void resetTestPose() {
        questNav.setPose(drivebase.getPose3d());
    }

    @Override
    public void periodic() {
        // Make these smartDashboard stuff send less data, maybe it is smart to not send data it does not need to
        if (questNav.isConnected()) {
            // If the x or y difference from the robots current pose to the limelight estimate pose update the current quest estimate for the position
            SmartDashboard.putBoolean("Quest Connected", true);
            if (resetTimer > 200) {
                if (drivebase.limelightPoseEstimate.getX() > 0.2 || drivebase.limelightPoseEstimate.getY() > 0.2) {
                    Pose3d limelightEstimatePose = new Pose3d(drivebase.limelightPoseEstimate);
                    resetQuestOdometry(limelightEstimatePose);
                    Util.consoleLog("Updated quest odomety to pose: ", limelightEstimatePose.toString());
                    resetTimer = 0;
                }
            } else {
                resetTimer++;
            }
        } else {
            limelightWarnLogger.update("The Questnav is not found, force updating pose with limelight pose!");
            drivebase.forceAddLimelightMeasurement(drivebase.limelightPoseEstimate);
            SmartDashboard.putBoolean("Quest Connected", false);
        }

        if (questNav.isTracking()) {
            SmartDashboard.putBoolean("Quest Tracking", true);
        } else {
            SmartDashboard.putBoolean("Quest Tracking", false);
        }

        questTestLogger.update("Quest periodic");
        // This method will be called once per scheduler run
        questNav.commandPeriodic();

        // Update pose Frames
        poseFrames = questNav.getAllUnreadPoseFrames();
        // Display number of frames provided
        SmartDashboard.putNumber("qFrames", poseFrames.length);
        for (PoseFrame questFrame : poseFrames) {
            if (questNav.isTracking()) {
                questLogger.update(questFrame.questPose3d().toPose2d().toString());
                drivebase.addQuestMeasurement(questFrame.questPose3d().toPose2d(), questFrame.dataTimestamp());
            }
        }
    }
}

package Team4450.Robot26.subsystems;

import Team4450.Robot26.Constants.DriveConstants;
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
    Transform3d ROBOT_TO_QUEST = new Transform3d(Constants.ROBOT_TO_QUEST_X, Constants.ROBOT_TO_QUEST_Y, Constants.ROBOT_TO_QUEST_Z, Rotation3d.kZero);
    
    Pose3d robotPose = DriveConstants.DEFAULT_STARTING_POSE_3D;
    final Pose3d nullPose = new Pose3d(-1, -1, -1, Rotation3d.kZero);

    ConsoleEveryX questTestLogger = new ConsoleEveryX(100);
    ConsoleEveryX questLogger = new ConsoleEveryX(100);

    PoseFrame[] poseFrames;

    /** Creates a new QuestNavSubsystem. */
    private DriveBase drivebase;
    public QuestNavSubsystem(DriveBase drivebase) {
        this.drivebase = drivebase;
        questNav = new QuestNav();

        resetToZeroPose();
    }

    public void resetToZeroPose() {
        Pose3d questPose3d = robotPose.transformBy(ROBOT_TO_QUEST);
        questNav.setPose(questPose3d);
        System.out.println("****QRobot reset to zero pose: " + questPose3d.toString());
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

    @Override
    public void periodic() {
        questTestLogger.update("Quest periodic");
        if (questNav.isTracking()) { // Is connected was working as well but I would prefer that is tracking would work, but it has not been tested
            // This method will be called once per scheduler run
            questNav.commandPeriodic();

            // Update pose Frames
            poseFrames = questNav.getAllUnreadPoseFrames();
            // Display number of frames provided
            SmartDashboard.putNumber("qFrames", poseFrames.length);
            if(Constants.UPDATE_QUESTNAV) {
                for (PoseFrame questFrame : poseFrames) {
                    // The dataTimestamp is in seconds
                    questLogger.update(questFrame.questPose3d().toPose2d().toString());
                    drivebase.addQuestMeasurement(questFrame.questPose3d().toPose2d(), questFrame.dataTimestamp());
                }
            }
        }
    }
}

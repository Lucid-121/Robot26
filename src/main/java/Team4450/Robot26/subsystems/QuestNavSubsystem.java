package Team4450.Robot26.subsystems;

import static Team4450.Robot26.Constants.alliance;

import java.util.Objects;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import Team4450.Lib.Util;
import Team4450.Robot26.RobotContainer;
import Team4450.Robot26.Constants;
import Team4450.Robot26.Constants.DriveConstants;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;
import Team4450.Robot26.subsystems.LimelightHelpers;

public class QuestNavSubsystem extends SubsystemBase {
    private boolean limelightEnabled = true;
    private boolean allianceRed = false; // Set to true if the robot is on the red alliance
    QuestNav questNav;
    Transform3d ROBOT_TO_QUEST = new Transform3d(-0.32, -0.29, 0, Rotation3d.kZero); //Original was -0.32, -0.29
                                                                                     // Transform2d ROBOT_TO_QUEST = new Transform2d(0, 0, Rotation2d.kZero); //Use for characterization
    Pose3d robotPose = DriveConstants.DEFAULT_STARTING_POSE_3D;
    final Pose3d nullPose = new Pose3d(-1, -1, -1, Rotation3d.kZero);



    PoseFrame[] poseFrames;

    /** Creates a new QuestNavSubsystem. */
    private DriveBase drivebase;
    public QuestNavSubsystem(DriveBase drivebase) {
        this.drivebase = drivebase;
        questNav = new QuestNav();

        resetToZeroPose();

        allianceRed = alliance == Alliance.Red;
    }

    public void resetToZeroPose() {
        Pose3d questPose3d = robotPose.transformBy(ROBOT_TO_QUEST);
        questNav.setPose(questPose3d);
        System.out.println("****QRobot reset to zero pose: " + questPose3d.toString());
    }

    public Pose3d getQuestRobotPose() {

        return (poseFrames != null && poseFrames.length > 0) ?
            poseFrames[poseFrames.length - 1].questPose3d()
            .transformBy(ROBOT_TO_QUEST.inverse()) :
            nullPose;

        // return qPose;
    }

    public double getQTimeStamp() {
        return (poseFrames != null && poseFrames.length > 0) ?
            poseFrames[poseFrames.length - 1].dataTimestamp() :
            0;
    }

    public double getQAppTimeStamp() {
        return (poseFrames != null && poseFrames.length > 0) ?
            poseFrames[poseFrames.length - 1].appTimestamp() :
            0;
    }

    public Pose3d getQuestPose() {
        return (poseFrames != null && poseFrames.length > 0) ?
            poseFrames[poseFrames.length - 1].questPose3d() :
            nullPose;
    }

    public void resetQuestOdometry(Pose3d rP) {

        // Transform by the offset to get the Quest pose
        Pose3d questPose3d = rP.transformBy(ROBOT_TO_QUEST);

        // Send the reset operation
        questNav.setPose(questPose3d);
        System.out.println("Quest Odometry Reset To: " + questPose3d.toString());
        System.out.println("QRP: " + rP.toString());
    }

    @Override
    public void periodic() {
        // https://github.com/LimelightVision/limelight-examples/tree/main/java-wpilib/swerve-megatag-odometry
        // https://docs.limelightvision.io/docs/docs-limelight/getting-started/summary
        if (questNav.isTracking()) {
            // This method will be called once per scheduler run
            questNav.commandPeriodic();

            //update pose Frames
            poseFrames = questNav.getAllUnreadPoseFrames();
            // Display number of frames provided
            SmartDashboard.putNumber("qFrames", poseFrames.length);
            if(Constants.UPDATE_QUESTNAV) {
                for (PoseFrame questFrame : poseFrames) {
                    // Is the dataTimestamp in seconds
                    drivebase.addVisionMeasurement(questFrame.questPose3d().toPose2d(), questFrame.dataTimestamp());

                    // Transform by the mount pose to get your robot pose
                    // Pose2d robotPose = questPose3d.transformBy(ROBOT_TO_QUEST.inverse());

                    // You can put some sort of filtering here if you would like!

                    // Add the measurement to our estimator
                    // RobotContainer.driveBase.updateOdometryQuest(getQuestRobotPose(), timestamp);




                }
            }
        }
    }
}

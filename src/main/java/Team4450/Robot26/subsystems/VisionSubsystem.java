 package Team4450.Robot26.subsystems;

import static Team4450.Robot26.Constants.alliance;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    public VisionSubsystem() {
    }

    @Override
    public void periodic() {
        // What will the turret need to update over time
    }

        LimeLightInfo info = limelightLogic.logicBotPoseCM();
        if (info == null) {
            return null;
        }
        // If LimeLight result outside of the field ignore it
        double x = info.pose.getX(DistanceUnit.CM);
        double y = info.pose.getY(DistanceUnit.CM);
        double yaw = info.pose.getHeading(AngleUnit.DEGREES);
        // If outside the field x
        if (Math.abs(x) > 182.88) {
            robotContainer.telemetry.addData("Limelight Failed because", " x+");
            return null;
        }
        // If outside the field y
        if (Math.abs(y) > 182.88) {
            robotContainer.telemetry.addData("Limelight Failed because", " y+");
            return null;
        }
        // If yaw is impossible
//        if (yaw > 360 || yaw < 0) {
//            robotContainer.telemetry.addData("Limelight Failed because", " yaw");
//            return null;
//        }
        robotContainer.telemetry.addData("llyaw", yaw);
        // If the april tag is to far from the center x
        if (Math.abs(info.result.getTx()) > 12) {
            robotContainer.telemetry.addData("Limelight Failed because", " tx");
            return null;
        }
        // If the april tag is to far from the center y
        if (Math.abs(info.result.getTy()) > 12) {
            robotContainer.telemetry.addData("Limelight Failed because", " ty");
            return null;
        }
        // Maybe more?
        return info.pose;
    }
    private LimeLightInfo getGoodLimeLightInfo() {
        LimeLightInfo info = limelightLogic.logicBotPoseCM();
        if (info == null) {
            return null;
        }
        // If LimeLight result outside of the field ignore it
        double x = info.pose.getX(DistanceUnit.CM);
        double y = info.pose.getY(DistanceUnit.CM);
        double yaw = info.pose.getHeading(AngleUnit.DEGREES);
        // If outside the field x
        if (Math.abs(x) > 182.88) {
            robotContainer.telemetry.addData("Limelight Failed because", " x+");
            return null;
        }
        // If outside the field y
        if (Math.abs(y) > 182.88) {
            robotContainer.telemetry.addData("Limelight Failed because", " y+");
            return null;
        }
        // If yaw is impossible
//        if (yaw > 360 || yaw < 0) {
//            robotContainer.telemetry.addData("Limelight Failed because", " yaw");
//            return null;
//        }
        return info;
}

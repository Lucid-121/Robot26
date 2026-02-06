package Team4450.Robot26.subsystems;

import Team4450.Robot26.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.InvertedValue;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;

public class TestSubsystem extends SubsystemBase {

    // ---------------- Hardware ----------------

    private final TalonFX flywheelMotor =
            new TalonFX(Constants.FLYWHEEL_MOTOR_CAN_ID);

    // ---------------- State ----------------

    private double targetRpm = 0.0;
    private double currentRpm = 0.0;

    private final double maxRpm =
            Constants.FLYWHEEL_MAX_THEORETICAL_RPM;

    private boolean flywheelEnabled = false; // Button-controlled enable

    // Shuffleboard cached values
    private boolean sdInit = false;

    private double sd_kP, sd_kI, sd_kD;
    private double sd_kS, sd_kV, sd_kA;

    // ---------------- Constructor ----------------

    public TestSubsystem() {

        TalonFXConfiguration cfg = new TalonFXConfiguration();

        // Neutral + inversion
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        cfg.MotorOutput.Inverted =
                Constants.FLYWHEEL_INVERTED
                        ? InvertedValue.Clockwise_Positive
                        : InvertedValue.CounterClockwise_Positive;

        // Slot 0 PID
        cfg.Slot0.kP = Constants.FLYWHEEL_kP;
        cfg.Slot0.kI = Constants.FLYWHEEL_kI;
        cfg.Slot0.kD = Constants.FLYWHEEL_kD;

        // Slot 0 Feedforward (Talon internal)
        cfg.Slot0.kS = Constants.FLYWHEEL_kS;
        cfg.Slot0.kV = Constants.FLYWHEEL_kV;
        cfg.Slot0.kA = Constants.FLYWHEEL_kA;

        // Motion Magic acceleration limits
        if (Constants.FLYWHEEL_USE_MOTION_MAGIC) {
            cfg.MotionMagic.MotionMagicAcceleration =
                    Constants.FLYWHEEL_MOTION_ACCEL_RPMS / 60.0;
            cfg.MotionMagic.MotionMagicJerk =
                    Constants.FLYWHEEL_MOTION_JERK;
        }

        flywheelMotor.getConfigurator().apply(cfg);

        // ---------------- Shuffleboard Defaults ----------------

        SmartDashboard.putNumber(
                "Flywheel/TargetRPM",
                Constants.FLYWHEEL_TARGET_RPM);

        SmartDashboard.putNumber("Flywheel/kP", Constants.FLYWHEEL_kP);
        SmartDashboard.putNumber("Flywheel/kI", Constants.FLYWHEEL_kI);
        SmartDashboard.putNumber("Flywheel/kD", Constants.FLYWHEEL_kD);

        SmartDashboard.putNumber("Flywheel/kS", Constants.FLYWHEEL_kS);
        SmartDashboard.putNumber("Flywheel/kV", Constants.FLYWHEEL_kV);
        SmartDashboard.putNumber("Flywheel/kA", Constants.FLYWHEEL_kA);

        sd_kP = Constants.FLYWHEEL_kP;
        sd_kI = Constants.FLYWHEEL_kI;
        sd_kD = Constants.FLYWHEEL_kD;

        sd_kS = Constants.FLYWHEEL_kS;
        sd_kV = Constants.FLYWHEEL_kV;
        sd_kA = Constants.FLYWHEEL_kA;

        sdInit = true;
    }

    // ---------------- Periodic ----------------

    @Override
    public void periodic() {

        // -------- Velocity measurement --------

        double measuredRps =
                flywheelMotor.getRotorVelocity()
                        .refresh()
                        .getValueAsDouble();

        currentRpm = measuredRps * 60.0;

        // -------- Shuffleboard tuning --------

        targetRpm = SmartDashboard.getNumber(
                "Flywheel/TargetRPM",
                Constants.FLYWHEEL_TARGET_RPM);

        double kP = SmartDashboard.getNumber("Flywheel/kP", sd_kP);
        double kI = SmartDashboard.getNumber("Flywheel/kI", sd_kI);
        double kD = SmartDashboard.getNumber("Flywheel/kD", sd_kD);

        double kS = SmartDashboard.getNumber("Flywheel/kS", sd_kS);
        double kV = SmartDashboard.getNumber("Flywheel/kV", sd_kV);
        double kA = SmartDashboard.getNumber("Flywheel/kA", sd_kA);

        // Apply only if changed
        if (!sdInit ||
                kP != sd_kP || kI != sd_kI || kD != sd_kD ||
                kS != sd_kS || kV != sd_kV || kA != sd_kA) {

            TalonFXConfiguration cfg = new TalonFXConfiguration();

            cfg.Slot0.kP = kP;
            cfg.Slot0.kI = kI;
            cfg.Slot0.kD = kD;

            cfg.Slot0.kS = kS;
            cfg.Slot0.kV = kV;
            cfg.Slot0.kA = kA;

            cfg.MotionMagic.MotionMagicAcceleration =
                    Constants.FLYWHEEL_MOTION_ACCEL_RPMS / 60.0;

            flywheelMotor.getConfigurator().apply(cfg);

            sd_kP = kP;
            sd_kI = kI;
            sd_kD = kD;

            sd_kS = kS;
            sd_kV = kV;
            sd_kA = kA;

            sdInit = true;
        }

        // -------- Control request --------

        double targetRps = flywheelEnabled
                ? targetRpm / 60.0
                : 0.0;

        if (Constants.FLYWHEEL_USE_MOTION_MAGIC) {

            MotionMagicVelocityVoltage req =
                    new MotionMagicVelocityVoltage(targetRps)
                            .withSlot(Constants.FLYWHEEL_PID_SLOT);

            flywheelMotor.setControl(req);

        } else {

            VelocityVoltage req =
                    new VelocityVoltage(targetRps)
                            .withSlot(Constants.FLYWHEEL_PID_SLOT);

            flywheelMotor.setControl(req);
        }

        // -------- Telemetry --------

        double percent = currentRpm / maxRpm;

        SmartDashboard.putNumber(
                "Flywheel/MeasuredRPM",
                currentRpm);

        SmartDashboard.putNumber(
                "Flywheel/PercentOutApprox",
                percent);
    }

    // ---------------- Commands / Accessors ----------------

    public void start() {
        flywheelEnabled = true;
    }

    public void stop() {
        flywheelEnabled = false;
    }

    public void setTargetRpm(double rpm) {
        targetRpm = rpm;
    }

    public double getTargetRpm() {
        return targetRpm;
    }

    public double getCurrentRpm() {
        return currentRpm;
    }
}









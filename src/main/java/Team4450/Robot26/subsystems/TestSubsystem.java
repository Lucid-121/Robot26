package Team4450.Robot26.subsystems;

import Team4450.Robot26.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.VelocityVoltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TestSubsystem extends SubsystemBase {
    private TalonFX testMotor = new TalonFX(Constants.FLYWHEEL_MOTOR_CAN_ID);
    // Flywheel state (RPM units)
    private double targetRpm = 0.0;
    private double defaultRPM = Constants.FLYWHEEL_DEFAULT_TARGET_RPM;
    // most recent measured RPM (for accessor/telemetry)
    private double currentRpm = 0.0;
    // keep last measured velocity (rotations per second) to estimate accel for feedforward
    private double lastRps = 0.0;
    private double maxRpm = Constants.FLYWHEEL_MAX_THEORETICAL_RPM;
    // Cached SmartDashboard-tunable values
    private double sd_kP, sd_kI, sd_kD;
    private double sd_kS, sd_kV, sd_kA;
    private double sd_direction;
    private boolean sdInit = false;

    public TestSubsystem() {
        // Configure motor neutral mode and closed-loop gains for onboard control.
        testMotor.setNeutralMode(NeutralModeValue.Brake);

        SmartDashboard.putNumber("Flywheel/DefaultRPM", defaultRPM);

        // Configure TalonFX slot gains & motion settings
        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
    var slot0 = talonFXConfigs.Slot0;
    // Use grouped Flywheel constants for clarity
    slot0.kS = Constants.Flywheel.kS;
    slot0.kV = Constants.Flywheel.kV;
    slot0.kA = Constants.Flywheel.kA;
    slot0.kP = Constants.Flywheel.kP;
    slot0.kI = Constants.Flywheel.kI;
    slot0.kD = Constants.Flywheel.kD;

        // Motion Magic velocity configs (only used if you enable Motion Magic Velocity)
    talonFXConfigs.MotionMagic.MotionMagicAcceleration = Constants.Flywheel.MOTION_ACCEL_RPMS / 60.0; // convert RPM/s to RPS/s
    talonFXConfigs.MotionMagic.MotionMagicJerk = Constants.Flywheel.MOTION_JERK;

        // Apply initial configuration to the motor (blocking call)
        testMotor.getConfigurator().apply(talonFXConfigs);

        // Publish tunables to SmartDashboard so they can be adjusted at runtime.
        SmartDashboard.putNumber("Flywheel/kP", Constants.Flywheel.kP);
        SmartDashboard.putNumber("Flywheel/kI", Constants.Flywheel.kI);
        SmartDashboard.putNumber("Flywheel/kD", Constants.Flywheel.kD);

        SmartDashboard.putNumber("Flywheel/kS", Constants.Flywheel.kS);
        SmartDashboard.putNumber("Flywheel/kV", Constants.Flywheel.kV);
        SmartDashboard.putNumber("Flywheel/kA", Constants.Flywheel.kA);

        SmartDashboard.putNumber("Flywheel/Direction", Constants.FLYWHEEL_DIRECTION);
        SmartDashboard.putNumber("Flywheel/TargetRPM", Constants.Flywheel.DEFAULT_TARGET_RPM);

        // Cache initial SmartDashboard values
        sd_kP = Constants.Flywheel.kP; sd_kI = Constants.Flywheel.kI; sd_kD = Constants.Flywheel.kD;
        sd_kS = Constants.Flywheel.kS; sd_kV = Constants.Flywheel.kV; sd_kA = Constants.Flywheel.kA;
        sd_direction = Constants.FLYWHEEL_DIRECTION;
        sdInit = true;
    }

    @Override
    public void periodic() {
        double dt = Constants.ROBOT_PERIOD_SEC;

    // Read actual rotor velocity from the Talon (rotations per second)
    // getRotorVelocity() returns a StatusSignal<AngularVelocity> so extract a primitive
    var rotorVelSignal = testMotor.getRotorVelocity();
    // Refresh the signal value (non-blocking request for most recent value)
    rotorVelSignal.refresh();
    // Use StatusSignal's primitive extractor. getValueAsDouble() returns the value
    // in the sensor's base units (rotations per second for rotor velocity).
    double measuredRps = rotorVelSignal.getValueAsDouble();
    double measuredRpm = measuredRps * 60.0;

    // store for accessor/telemetry
    this.currentRpm = measuredRpm;

        // Read SmartDashboard tunables (allows live tuning)
        double kP = SmartDashboard.getNumber("Flywheel/kP", sd_kP);
        double kI = SmartDashboard.getNumber("Flywheel/kI", sd_kI);
        double kD = SmartDashboard.getNumber("Flywheel/kD", sd_kD);

        double kS = SmartDashboard.getNumber("Flywheel/kS", sd_kS);
        double kV = SmartDashboard.getNumber("Flywheel/kV", sd_kV);
        double kA = SmartDashboard.getNumber("Flywheel/kA", sd_kA);

        double direction = SmartDashboard.getNumber("Flywheel/Direction", sd_direction);

        // If any PID/feedforward changed, update the Talon slot configuration once.
        if (!sdInit || kP != sd_kP || kI != sd_kI || kD != sd_kD || kS != sd_kS || kV != sd_kV || kA != sd_kA) {
            TalonFXConfiguration cfg = new TalonFXConfiguration();
            var slot0 = cfg.Slot0;
            slot0.kS = kS;
            slot0.kV = kV;
            slot0.kA = kA;
            slot0.kP = kP;
            slot0.kI = kI;
            slot0.kD = kD;
            // keep any existing motion magic settings
            cfg.MotionMagic.MotionMagicAcceleration = Constants.Flywheel.MOTION_ACCEL_RPMS / 60.0;
            cfg.MotionMagic.MotionMagicJerk = Constants.Flywheel.MOTION_JERK;
            testMotor.getConfigurator().apply(cfg);

            sd_kP = kP; sd_kI = kI; sd_kD = kD;
            sd_kS = kS; sd_kV = kV; sd_kA = kA;
            sd_direction = direction;
            sdInit = true;
        }

        // Convert target RPM to rotations per second for control request
        // Apply a hardware direction multiplier so the constants can stay positive.
        double targetRps = (targetRpm / 60.0) * sd_direction;

        // Estimate acceleration (rps/s) using last measured value for feedforward
        double accelRpsPerSec = 0.0;
        if (dt > 0) accelRpsPerSec = (targetRps - lastRps) / dt;

        // Build a VelocityVoltage control request (uses slot 0 gains configured above)
    VelocityVoltage vRequest = new VelocityVoltage(0).withSlot(Constants.Flywheel.PID_SLOT)
        .withVelocity(targetRps);

        // Compute feedforward (Volts)
    double ffVolts = sd_kS * Math.signum(targetRps)
        + sd_kV * targetRps
        + sd_kA * accelRpsPerSec;
        vRequest = vRequest.withFeedForward(ffVolts);

        // Send the closed-loop request to the Talon (the device will maintain velocity onboard)
        testMotor.setControl(vRequest);

        // Telemetry
        double percent = 0.0;
        if (maxRpm > 0.0) percent = measuredRpm / maxRpm;
        if (percent > 1.0) percent = 1.0;
        if (percent < -1.0) percent = -1.0;

        SmartDashboard.putNumber("Flywheel/TargetRPM", targetRpm);
        defaultRPM = SmartDashboard.getNumber("Flywheel/DefaultRPM", defaultRPM);
        SmartDashboard.putNumber("Flywheel/MeasuredRPM", measuredRpm);
        SmartDashboard.putNumber("Flywheel/FeedForwardVolts", ffVolts);
        SmartDashboard.putNumber("Flywheel/PercentOutApprox", percent);

        // remember last measured rps for next tick
        lastRps = measuredRps;
    }


    public void start() {
        // Start by setting the target RPM from constants. periodic() will ramp the motor to it.
        this.targetRpm = defaultRPM/1.52;
    }

    public void stop() {
        // Ramp down to zero
        this.targetRpm = 0.0;
    }

    // Optional programmatic control
    public void setTargetRpm(double rpm) { this.targetRpm = rpm; }
    public double getTargetRpm() { return this.targetRpm; }
    public double getCurrentRpm() { return this.currentRpm; }
}

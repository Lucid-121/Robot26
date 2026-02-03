package Team4450.Robot26.subsystems;

import Team4450.Robot26.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.CANBus;

public class Intake extends SubsystemBase {

    // This motor is a Kraken x60
    private final TalonFX intakePivitMotor = new TalonFX(Constants.INTAKE_MOTOR_PIVIT_CAN_ID, new CANBus(Constants.CANIVORE_NAME));
    // This motor is a Kraken x44
    private final TalonFX intakeLeftMotor = new TalonFX(Constants.INTAKE_MOTOR_LEFT_CAN_ID, new CANBus(Constants.CANIVORE_NAME));
    // This motor is a Kraken x44
    private final TalonFX intakeRightMotor = new TalonFX(Constants.INTAKE_MOTOR_RIGHT_CAN_ID, new CANBus(Constants.CANIVORE_NAME));

    private boolean canPivit;
    private boolean canSpin;

    // This value is expected to be between 0 and 1
    private double pivitTargetPosition;
    // The format of this value is in rotations of the pivit motor
    private double pivitTargetPositionMotorPosition;

    // This value is expected to be between 0 and 1
    private double pivitCurrentPosition;
    // The format of this value is in rotations of the pivit motor
    private double pivitCurrentPositionMotorPosition;

    // I wish Java had errors as values
    public Intake() {
        this.canPivit = this.intakePivitMotor.isConnected();
        this.canSpin = this.intakeLeftMotor.isConnected() || this.intakeRightMotor.isConnected();

        // Assume the pivit starting position is 0
        this.pivitCurrentPosition = 0;
        this.pivitTargetPosition = 0;

        if (this.canPivit) {
            this.intakePivitMotor.setPosition(0);
        }

        // Convert to sendable
        SmartDashboard.putBoolean("Intake can Pivit", this.canPivit);
        SmartDashboard.putBoolean("Intake can Spin", this.canSpin);
    }

    @Override
    public void periodic() {
        this.pivitTargetPosition = SmartDashboard.getNumber("Pivit Position", 0);
        if (this.canPivit) {
            this.pivitTargetPositionMotorPosition = this.pivitPositionToMotorPosition(this.pivitTargetPosition);
            // Convert position input to rotations for the motor
            double power = SmartDashboard.getNumber("pivit MotorPower", Constants.INTAKE_PIVIT_MOTOR_POWER);
            
            if (this.pivitCurrentPositionMotorPosition <= this.pivitTargetPositionMotorPosition - Constants.INTAKE_PIVIT_TOLERENCE_MOTOR_ROTATIONS) {
                this.intakePivitMotor.set(power);
            } else if (this.pivitCurrentPositionMotorPosition >= this.pivitTargetPositionMotorPosition + Constants.INTAKE_PIVIT_TOLERENCE_MOTOR_ROTATIONS) {
                this.intakePivitMotor.set(-power);
            } else {
                this.intakePivitMotor.set(0);
            }

            this.pivitCurrentPositionMotorPosition = this.getPivitMotorPosition();
            this.pivitCurrentPosition = this.motorPositionToPivitPosition(this.pivitCurrentPositionMotorPosition);
            SmartDashboard.putNumber("Pivit current position", this.pivitCurrentPosition);
        }
    }

    // Linear interpolate the pivit position between zero and one with the motor rotations of up and down on the pivit
    public double pivitPositionToMotorPosition(double pivitPosition) {
        return Constants.INTAKE_PIVIT_MOTOR_POSITION_UP + ((Constants.INTAKE_PIVIT_MOTOR_POSITION_DOWN - Constants.INTAKE_PIVIT_MOTOR_POSITION_UP) * pivitPosition);
    }

    public double motorPositionToPivitPosition(double motorPosition) {
        return (motorPosition / Constants.INTAKE_PIVIT_GEAR_RATIO) * (1 / (Constants.INTAKE_PIVIT_POSITION_DOWN_DEGREES / 360));
    }

    public void startIntake() {
        if (canSpin) {
            intakeLeftMotor.set(1);
            intakeRightMotor.set(1);
        }
    }

    public void startIntakeWithSpeed(double speed) {
        if (canSpin) {
            intakeLeftMotor.set(speed);
            intakeRightMotor.set(speed);
        }
    }

    public void stopIntake() {
        if (canSpin) {
            intakeLeftMotor.set(0);
            intakeRightMotor.set(0);
        }
    }

    public double getIntakeRPM() {
        if (canSpin) {
            return intakeLeftMotor.getRotorVelocity(true).getValueAsDouble() * 60;
        } else {
            return -1;
        }
    }

    public double getIntakeCurrent() {
        if (canSpin) {
            return intakeLeftMotor.getSupplyCurrent(true).getValueAsDouble() + intakeRightMotor.getSupplyCurrent(true).getValueAsDouble();
        } else {
            return -1;
        }
    }

    public double getIntakeLeftMotorCurrent() {
        if (canSpin) {
            return intakeLeftMotor.getSupplyCurrent(true).getValueAsDouble();
        } else {
            return -1;
        }
    }

    public double getIntakeRightMotorCurrent() {
        if (canSpin) {
            return intakeRightMotor.getSupplyCurrent(true).getValueAsDouble();
        } else {
            return -1;
        }
    }

    public void setPivitMotorSpeed(double speed) {
        if (canPivit) {
            intakePivitMotor.set(speed);
        }
    }

    // The position input is between 0 and 1 with 0 being up and 1 being down
    public void setPivitMotorPosition(double position) {
        pivitTargetPosition = position;
    }

    public double getPivitMotorPosition() {
        if (canPivit) {
            return intakePivitMotor.getPosition(true).getValueAsDouble();
        } else {
            return -1;
        }
    }

    public double getPivitMotorCurrent() {
        if (canPivit) {
            return intakePivitMotor.getSupplyCurrent(true).getValueAsDouble();
        } else {
            return -1;
        }
    }
}

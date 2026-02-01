package Team4450.Robot26.subsystems;

import Team4450.Robot26.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.controls.DifferentialPositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;


public class Intake extends SubsystemBase {

    // This motor is a Kraken x60
    private final TalonFX intakePivitMotor = new TalonFX(Constants.INTAKE_MOTOR_PIVIT_CAN_ID);
    // This motor is a Kraken x44
    private final TalonFX intakeLeftMotor = new TalonFX(Constants.INTAKE_MOTOR_LEFT_CAN_ID);
    // This motor is a Kraken x44
    private final TalonFX intakeRightMotor = new TalonFX(Constants.INTAKE_MOTOR_RIGHT_CAN_ID);

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

    private DifferentialPositionVoltage pivitPositionVoltagePID = new DifferentialPositionVoltage(0, Constants.INTAKE_PIVIT_TOLERENCE_MOTOR_ROTATIONS);

    // I wish Java had errors as values
    public Intake() {
        canPivit = intakePivitMotor.isConnected();
        canSpin = intakeLeftMotor.isConnected() || intakeRightMotor.isConnected();
        // Assume the pivit starting position is 0
        pivitCurrentPosition = 0;
        pivitTargetPosition = 0;
    }

    @Override
    public void periodic() {
        if (canPivit) {
            this.pivitTargetPositionMotorPosition = pivitPositionToMotorPosition(pivitTargetPosition);
            // Convert position input to rotations for the motor
            // I think this is not the correct way to use this??
            // I think I only need to use with average position when it changes
            intakePivitMotor.setControl(pivitPositionVoltagePID.withAveragePosition(this.pivitTargetPositionMotorPosition));

            this.pivitCurrentPositionMotorPosition = getPivitPosition();
            this.pivitCurrentPosition = motorPositionToPivitPosition(this.pivitCurrentPositionMotorPosition);
            SmartDashboard.putNumber("Pivit position", this.pivitCurrentPosition);
        }
    }

    // Linear interpolate the pivit position between zero and one with the motor rotations of up and down on the pivit
    public double pivitPositionToMotorPosition(double pivitPosition) {
        return Constants.INTAKE_PIVIT_MOTOR_POSITION_UP + ((Constants.INTAKE_PIVIT_MOTOR_POSITION_DOWN - Constants.INTAKE_PIVIT_MOTOR_POSITION_UP) * pivitPosition);
    }

    public double motorPositionToPivitPosition(double motorPosition) {
        return motorPosition / Constants.INTAKE_PIVIT_GEAR_RATIO;
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

    public double getIntakeVoltage() {
        if (canSpin) {
            return intakeLeftMotor.getSupplyVoltage(true).getValueAsDouble() + intakeRightMotor.getSupplyVoltage(true).getValueAsDouble();
        } else {
            return -1;
        }
    }

    public double getIntakeLeftMotorVoltage() {
        if (canSpin) {
            return intakeLeftMotor.getSupplyVoltage(true).getValueAsDouble();
        } else {
            return -1;
        }
    }

    public double getIntakeRightMotorVoltage() {
        if (canSpin) {
            return intakeRightMotor.getSupplyVoltage(true).getValueAsDouble();
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

    public double getPivitPosition() {
        // Need to convert
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

    public double getPivitMotorVoltage() {
        if (canPivit) {
            return intakePivitMotor.getSupplyVoltage(true).getValueAsDouble();
        } else {
            return -1;
        }
    }
}

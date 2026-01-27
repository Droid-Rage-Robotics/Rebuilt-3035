package frc.robot.subsystems.drive;

import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.utility.motor.MotorConstants.Direction;
import lombok.Getter;

public class SwerveModuleConstants {
    public enum POD{
        FL("Front Left"),
        BL("Back Left"),
        FR("Front Right"),
        BR("Back Right");

        @Getter private final String name;

        private POD(String name) {
            this.name=name;
        }
    }
    
    public int driveMotorId;
    public int turnMotorId;
    public int encoderId;

    public Direction driveMotorDirection;
    public Direction turnMotorDirection;
    public SensorDirectionValue encoderDirection;

    public boolean driveMotorIsEnabled;
    public boolean turnMotorIsEnabled;

    public double encoderOffsetRad;

    public Subsystem subsystem;
    public POD podName;

    public SwerveModuleConstants withDriveMotorId(int driveMotorId) {
        this.driveMotorId=driveMotorId;
        return this;
    }

    public SwerveModuleConstants withTurnMotorId(int turnMotorId) {
        this.turnMotorId=turnMotorId;
        return this;
    }

    public SwerveModuleConstants withEncoderId(int encoderId) {
        this.encoderId = encoderId;
        return this;
    }

    public SwerveModuleConstants withDriveMotorDirection(Direction driveMotorDirection) {
        this.driveMotorDirection = driveMotorDirection;
        return this;
    }

    public SwerveModuleConstants withTurnMotorDirection(Direction turnMotorDirection) {
        this.turnMotorDirection = turnMotorDirection;
        return this;
    }

    public SwerveModuleConstants withEncoderDirection(SensorDirectionValue encoderDirection) {
        this.encoderDirection = encoderDirection;
        return this;
    }

    public SwerveModuleConstants withDriveMotorIsEnabled(boolean driveMotorIsEnabled) {
        this.driveMotorIsEnabled = driveMotorIsEnabled;
        return this;
    }

    public SwerveModuleConstants withTurnMotorIsEnabled(boolean turnMotorIsEnabled) {
        this.turnMotorIsEnabled = turnMotorIsEnabled;
        return this;
    }

    public SwerveModuleConstants withEncoderOffsetRad(double encoderOffsetRad) {
        this.encoderOffsetRad = encoderOffsetRad;
        return this;
    }

    public SwerveModuleConstants withSubsystem(Subsystem subsystem) {
        this.subsystem=subsystem;
        return this;
    }

    public SwerveModuleConstants withPodName(POD podName) {
        this.podName=podName;
        return this;
    }

}

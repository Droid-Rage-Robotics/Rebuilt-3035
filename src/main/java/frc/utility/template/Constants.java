package frc.utility.template;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;

public class Constants {
    public static sealed interface SubsystemConstants
        permits ArmConstants, ElevatorConstants, FlywheelConstants, TurretConstants {
        String name();
        int mainNum();
    }

    public static enum EncoderType {
        INTEGRATED,
        ABSOLUTE,
        EXTERNAL
    }

    public static final class ArmConstants implements SubsystemConstants {
        public String name = "Arm";
        public int mainNum = 0;

        public EncoderType encoderType = EncoderType.INTEGRATED;

        public Angle minAngle = Degrees.of(-90);
        public Angle maxAngle = Degrees.of(90);
        public Angle resetAngle = Degrees.zero();

        public AngularVelocity maxVelocity = RotationsPerSecond.of(1);
        public AngularAcceleration maxAcceleration = RotationsPerSecondPerSecond.of(1);
        public double maxJerk = 0.0;

        public double gearRatio = 1.0;
        public double offset = 0.0;

        public double kP, kI, kD;
        public double kS, kG, kV, kA;

        @Override
        public String name() {
            return name;
        }

        @Override
        public int mainNum() {
            return mainNum;
        }

        public ArmConstants withName(String name) {
            this.name = name;
            return this;
        }

        public ArmConstants withMainNum(int mainNum) {
            this.mainNum = mainNum;
            return this;
        }

        public ArmConstants withEncoderType(EncoderType encoderType) {
            this.encoderType = encoderType;
            return this;
        }

        public ArmConstants withLimits(Angle minAngle, Angle maxAngle) {
            this.minAngle = minAngle;
            this.maxAngle = maxAngle;
            return this;
        }

        public ArmConstants withResetAngle(Angle resetAngle) {
            this.resetAngle = resetAngle;
            return this;
        }

        public ArmConstants withMotionMagic(
                AngularVelocity maxVelocity,
                AngularAcceleration maxAcceleration,
                double maxJerk
        ) {
            this.maxVelocity = maxVelocity;
            this.maxAcceleration = maxAcceleration;
            this.maxJerk = maxJerk;
            return this;
        }

        public ArmConstants withGearRatio(double gearRatio) {
            this.gearRatio = gearRatio;
            return this;
        }

        public ArmConstants withPID(double kP, double kI, double kD) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            return this;
        }

        public ArmConstants withFeedforward(double kS, double kG, double kV, double kA) {
            this.kS = kS;
            this.kG = kG;
            this.kV = kV;
            this.kA = kA;
            return this;
        }
    }

    public static final class TurretConstants implements SubsystemConstants {
        public String name = "Turret";
        public int mainNum = 0;

        public EncoderType encoderType = EncoderType.INTEGRATED;

        public Angle minAngle = Degrees.of(-90);
        public Angle maxAngle = Degrees.of(90);
        public Angle resetAngle = Degrees.zero();

        public AngularVelocity maxVelocity = RotationsPerSecond.of(1);
        public AngularAcceleration maxAcceleration = RotationsPerSecondPerSecond.of(1);
        public double maxJerk = 0.0;

        public double gearRatio = 1.0;
        public double offset = 0.0;

        public double kP, kI, kD;
        public double kS, kV, kA;

        @Override
        public String name() {
            return name;
        }

        @Override
        public int mainNum() {
            return mainNum;
        }

        public TurretConstants withName(String name) {
            this.name = name;
            return this;
        }

        public TurretConstants withMainNum(int mainNum) {
            this.mainNum = mainNum;
            return this;
        }

        public TurretConstants withEncoderType(EncoderType encoderType) {
            this.encoderType = encoderType;
            return this;
        }

        public TurretConstants withLimits(Angle minAngle, Angle maxAngle) {
            this.minAngle = minAngle;
            this.maxAngle = maxAngle;
            return this;
        }

        public TurretConstants withResetAngle(Angle resetAngle) {
            this.resetAngle = resetAngle;
            return this;
        }

        public TurretConstants withMotionMagic(
                AngularVelocity maxVelocity,
                AngularAcceleration maxAcceleration,
                double maxJerk
        ) {
            this.maxVelocity = maxVelocity;
            this.maxAcceleration = maxAcceleration;
            this.maxJerk = maxJerk;
            return this;
        }

        public TurretConstants withGearRatio(double gearRatio) {
            this.gearRatio = gearRatio;
            return this;
        }

        public TurretConstants withPID(double kP, double kI, double kD) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            return this;
        }

        public TurretConstants withFeedforward(double kS, double kV, double kA) {
            this.kS = kS;
            this.kV = kV;
            this.kA = kA;
            return this;
        }
    }

    public static final class ElevatorConstants implements SubsystemConstants {
        public String name = "Elevator";
        public int mainNum = 0;

        public EncoderType encoderType = EncoderType.INTEGRATED;

        public Distance minDistance = Meters.zero();
        public Distance maxDistance = Meters.of(1);
        public Distance resetPosition = Meters.zero();

        public LinearVelocity maxVelocity = MetersPerSecond.of(1);
        public LinearAcceleration maxAcceleration = MetersPerSecondPerSecond.of(1);

        /**
         * CTRE Motion Magic jerk is still configured in motor rotations/sec^3,
         * so this stays double unless you make a custom linear jerk unit.
         */
        public double maxJerk = 0.0;

        /**
         * Elevator mechanism conversion.
         * Example: meters of elevator travel per motor rotation.
         */
        public double metersPerMotorRotation = 1.0;

        public double kP, kI, kD;
        public double kS, kG, kV, kA;

        @Override
        public String name() {
            return name;
        }

        @Override
        public int mainNum() {
            return mainNum;
        }

        public ElevatorConstants withName(String name) {
            this.name = name;
            return this;
        }

        public ElevatorConstants withMainNum(int mainNum) {
            this.mainNum = mainNum;
            return this;
        }

        public ElevatorConstants withEncoderType(EncoderType encoderType) {
            this.encoderType = encoderType;
            return this;
        }

        public ElevatorConstants withLimits(Distance minDistance, Distance maxDistance) {
            this.minDistance = minDistance;
            this.maxDistance = maxDistance;
            return this;
        }

        public ElevatorConstants withResetPosition(Distance resetPosition) {
            this.resetPosition = resetPosition;
            return this;
        }

        public ElevatorConstants withMotionMagic(
                LinearVelocity maxVelocity,
                LinearAcceleration maxAcceleration,
                double maxJerk
        ) {
            this.maxVelocity = maxVelocity;
            this.maxAcceleration = maxAcceleration;
            this.maxJerk = maxJerk;
            return this;
        }

        public ElevatorConstants withMetersPerMotorRotation(double metersPerMotorRotation) {
            this.metersPerMotorRotation = metersPerMotorRotation;
            return this;
        }

        public ElevatorConstants withPID(double kP, double kI, double kD) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            return this;
        }

        public ElevatorConstants withFeedforward(double kS, double kG, double kV, double kA) {
            this.kS = kS;
            this.kG = kG;
            this.kV = kV;
            this.kA = kA;
            return this;
        }
    }

    public static final class FlywheelConstants implements SubsystemConstants {
        public String name = "Flywheel";
        public int mainNum = 0;

        public AngularVelocity minVelocity = RotationsPerSecond.zero();
        public AngularVelocity maxVelocity = RotationsPerSecond.of(100);

        public AngularAcceleration maxAcceleration = RotationsPerSecondPerSecond.of(100);
        public double maxJerk = 0.0;

        public double gearRatio = 1.0;

        public double kP, kI, kD;
        public double kS, kV, kA;

        @Override
        public String name() {
            return name;
        }

        @Override
        public int mainNum() {
            return mainNum;
        }

        public FlywheelConstants withName(String name) {
            this.name = name;
            return this;
        }

        public FlywheelConstants withMainNum(int mainNum) {
            this.mainNum = mainNum;
            return this;
        }

        public FlywheelConstants withVelocityLimits(
                AngularVelocity minVelocity,
                AngularVelocity maxVelocity
        ) {
            this.minVelocity = minVelocity;
            this.maxVelocity = maxVelocity;
            return this;
        }

        public FlywheelConstants withGearRatio(double gearRatio) {
            this.gearRatio = gearRatio;
            return this;
        }

        public FlywheelConstants withPID(double kP, double kI, double kD) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            return this;
        }

        public FlywheelConstants withFeedforward(double kS, double kV, double kA) {
            this.kS = kS;
            this.kV = kV;
            this.kA = kA;
            return this;
        }
    }
}

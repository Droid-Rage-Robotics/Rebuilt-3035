package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import lombok.Getter;

public class SwerveConfig {
    public static class ModuleConstants {
        public static final Distance WHEEL_DIAMETER = Inches.of(4);
        // public static final double DRIVE_MOTOR_GEAR_RATIO = GearRatio.R3.getConversionFactor();
        // public static final double TURN_MOTOR_GEAR_RATIO = GearRatio.TURN.getConversionFactor();

        // public static final double DRIVE_ENCODER_ROT_2_METER = DRIVE_MOTOR_GEAR_RATIO * Math.PI * WHEEL_DIAMETER.in(Meters);
        // public static final double DRIVE_ENCODER_RPM_2_METER_PER_SEC = DRIVE_ENCODER_ROT_2_METER / 60;
        public static final double READINGS_PER_REVOLUTION = 1;//4096

        //Used for the CANCoder
        public static final double TURN_ENCODER_ROT_2_RAD = 2 * Math.PI / READINGS_PER_REVOLUTION;
        public static final double TURN_ENCODER_ROT_2_RAD_SEC = TURN_ENCODER_ROT_2_RAD/60;

        /* Current Limits */
        public static final double DRIVE_SUPPLY_CURRENT_LIMIT = 35; //MA:40
        public static final double DRIVE_STATOR_CURRENT_LIMIT = 75; //MA:80
        public static final double TURN_SUPPLY_CURRENT_LIMIT = 35; //MA:40 //2025=80
        //SUPERNURDS had STATOR at 60, no supply
    }

    public static final PIDConstants TRANSLATIONAL_PID = new PIDConstants(7,0,0);

    public static final PIDConstants THETA_PID = new PIDConstants(5,0,0);
    
    public enum Speed {
        TURBO(1, 1),
        NORMAL(1.6, 1.6),//3.5, 1 //1,.4
        SLOW(.2, 0.25),
        SUPER_SLOW(0.05, 0.05),
        ;
        private final double translationalValue;
        private final double angularValue;
        private Speed(double translationalSpeed, double angularSpeed) {
            this.translationalValue = translationalSpeed;
            this.angularValue = angularSpeed;
        }
        public double getTranslationalSpeed() {
            return translationalValue;
        }
        public double getAngularSpeed() {
            return angularValue;
        }
    }

    public enum GearRatio {
        R1(7.03),
        R2(6.03),
        R3(5.27),
        TURN(26.09);

        @Getter private final double conversionFactor;
        @Getter private final double gearRatio;

        private GearRatio(double gearRatio) {
            this.gearRatio=gearRatio;
            this.conversionFactor=(1.0/gearRatio);
        }
    }

    public enum DriveOptions { 
        IS_FIELD_ORIENTED(true),
        IS_SQUARED_INPUTS(true),
        IS_POSE_UPDATED(true)
        ;
        private final boolean value;
        private DriveOptions(boolean value) {
            this.value = value;
        } 
        public boolean get(){
            return value;
        }
    }
    
    public static final Distance WHEEL_RADIUS = Inches.of(2);
    
    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    @Getter private final Rotation2d blueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    @Getter private final Rotation2d redAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);


    private static final Slot0Configs steerGains = new Slot0Configs()
        .withKP(110).withKI(0).withKD(0)
        // .withKS(0.35738).withKV(2.4411).withKA(0.18145)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

    private static final Slot0Configs driveGains = new Slot0Configs()
        .withKP(0.79827).withKI(0).withKD(0)
        .withKS(0.14055).withKV(1.929);

    // The closed-loop output type to use for the steer motors;
    // This affects the PID/FF gains for the steer motors
    private static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
    // The closed-loop output type to use for the drive motors;
    // This affects the PID/FF gains for the drive motors
    private static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;

    // The type of motor used for the drive motor
    private static final DriveMotorArrangement kDriveMotorType = DriveMotorArrangement.TalonFX_Integrated;
    // The type of motor used for the drive motor
    private static final SteerMotorArrangement kSteerMotorType = SteerMotorArrangement.TalonFX_Integrated;

    // The remote sensor feedback type to use for the steer motors;
    // When not Pro-licensed, Fused*/Sync* automatically fall back to Remote*
    private static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.RemoteCANcoder;

    // The stator current at which the wheels start to slip;
    // This needs to be tuned to your individual robot
    private static final Current kSlipCurrent = Amps.of(70);

    // Initial configs for the drive and steer motors and the azimuth encoder; these cannot be null.
    // Some configs will be overwritten; check the `with*InitialConfigs()` API documentation.
    private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(Amps.of(ModuleConstants.DRIVE_SUPPLY_CURRENT_LIMIT))
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimit(Amps.of(ModuleConstants.DRIVE_STATOR_CURRENT_LIMIT))
                .withStatorCurrentLimitEnable(true)   
        );
    private static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                // Swerve azimuth does not require much torque output, so we can set a relatively low
                // stator current limit to help avoid brownouts without impacting performance.
                .withSupplyCurrentLimit(Amps.of(ModuleConstants.TURN_SUPPLY_CURRENT_LIMIT))
                .withSupplyCurrentLimitEnable(true)
                // .withStatorCurrentLimit(Amps.of(DriveConstants.ModuleConstants.TURN_SUPPLY_CURRENT_LIMIT))
                // .withStatorCurrentLimitEnable(true)
        );
    private static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();
    // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
    private static final Pigeon2Configuration pigeonConfigs = null;

    // CAN bus that the devices are located on;
    // All swerve devices must share the same CAN bus
    public static final CANBus kCANBus = new CANBus("drive", "./logs/example.hoot");

    public static final LinearVelocity ATTAINABLE_MAX_SPEED = MetersPerSecond.of(4.47);
    public static final AngularVelocity ATTAINABLE_MAX_SPEED_ANG = RadiansPerSecond.of(2 * (2 * Math.PI));

    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot
    private static final double kCoupleRatio = 3.375;

    private static final double kDriveGearRatio = 5.2734375;
    private static final double kSteerGearRatio = 26.09090909090909;

    private static final boolean kInvertLeftSide = false;
    private static final boolean kInvertRightSide = true;

    private static final int kPigeonId = 13;

    // These are only used for simulation
    private static final MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.01);
    private static final MomentOfInertia kDriveInertia = KilogramSquareMeters.of(0.01);
    // Simulated voltage necessary to overcome friction
    private static final Voltage kSteerFrictionVoltage = Volts.of(0.2);
    private static final Voltage kDriveFrictionVoltage = Volts.of(0.2);

    @Getter private SwerveDrivetrainConstants drivetrainConstants;

    @Getter
    private SwerveModuleConstantsFactory<
                    TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
            constantCreator;

    // Front Left
    private static final int kFrontLeftDriveMotorId = 12;
    private static final int kFrontLeftSteerMotorId = 10;
    private static final int kFrontLeftEncoderId = 11;
    private static final Angle kFrontLeftEncoderOffset = Rotations.of(0.1806640625);
    private static final boolean kFrontLeftSteerMotorInverted = false;
    private static final boolean kFrontLeftEncoderInverted = false;

    private static final Distance kFrontLeftXPos = Inches.of(10);
    private static final Distance kFrontLeftYPos = Inches.of(10);

    // Front Right
    private static final int kFrontRightDriveMotorId = 3;
    private static final int kFrontRightSteerMotorId = 1;
    private static final int kFrontRightEncoderId = 2;
    private static final Angle kFrontRightEncoderOffset = Rotations.of(0.04052734375);
    private static final boolean kFrontRightSteerMotorInverted = false;
    private static final boolean kFrontRightEncoderInverted = false;

    private static final Distance kFrontRightXPos = Inches.of(10);
    private static final Distance kFrontRightYPos = Inches.of(-10);

    // Back Left
    private static final int kBackLeftDriveMotorId = 9;
    private static final int kBackLeftSteerMotorId = 7;
    private static final int kBackLeftEncoderId = 8;
    private static final Angle kBackLeftEncoderOffset = Rotations.of(0.081787109375);
    private static final boolean kBackLeftSteerMotorInverted = false;
    private static final boolean kBackLeftEncoderInverted = false;

    private static final Distance kBackLeftXPos = Inches.of(-10);
    private static final Distance kBackLeftYPos = Inches.of(10);

    // Back Right
    private static final int kBackRightDriveMotorId = 6;
    private static final int kBackRightSteerMotorId = 4;
    private static final int kBackRightEncoderId = 5;
    private static final Angle kBackRightEncoderOffset = Rotations.of(-0.133544921875);
    private static final boolean kBackRightSteerMotorInverted = false;
    private static final boolean kBackRightEncoderInverted = false;

    private static final Distance kBackRightXPos = Inches.of(-10);
    private static final Distance kBackRightYPos = Inches.of(-10);
        
    @Getter
    private SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
            frontLeft;

    @Getter
    private SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
            frontRight;

    @Getter
    private SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
            backLeft;

    @Getter
    private SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
            backRight;

    private SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
                    []
            modules;

    @SuppressWarnings("unchecked")
    public SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
            [] getModules() {
        if (frontLeft != null && frontRight != null && backLeft != null && backRight != null) {
            modules = new SwerveModuleConstants[] {frontLeft, frontRight, backLeft, backRight};
        } else {
            throw new IllegalStateException("One or more SwerveModuleConstants are null");
        }
        return modules;
    }

    public SwerveConfig() {
        updateConfig();
    }

    public SwerveConfig updateConfig() {
        drivetrainConstants = new SwerveDrivetrainConstants()
            .withCANBusName(kCANBus.getName())
            .withPigeon2Id(kPigeonId)
            .withPigeon2Configs(pigeonConfigs);


        constantCreator = new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
            .withDriveMotorGearRatio(kDriveGearRatio)
            .withSteerMotorGearRatio(kSteerGearRatio)
            .withCouplingGearRatio(kCoupleRatio)
            .withWheelRadius(WHEEL_RADIUS)
            .withSteerMotorGains(steerGains)
            .withDriveMotorGains(driveGains)
            .withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
            .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
            .withSlipCurrent(kSlipCurrent)
            .withSpeedAt12Volts(ATTAINABLE_MAX_SPEED)
            .withDriveMotorType(kDriveMotorType)
            .withSteerMotorType(kSteerMotorType)
            .withFeedbackSource(kSteerFeedbackType)
            .withDriveMotorInitialConfigs(driveInitialConfigs)
            .withSteerMotorInitialConfigs(steerInitialConfigs)
            .withEncoderInitialConfigs(encoderInitialConfigs)
            .withSteerInertia(kSteerInertia)
            .withDriveInertia(kDriveInertia)
            .withSteerFrictionVoltage(kSteerFrictionVoltage)
            .withDriveFrictionVoltage(kDriveFrictionVoltage);
        
        frontLeft = constantCreator.createModuleConstants(
            kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId, kFrontLeftEncoderOffset,
            kFrontLeftXPos, kFrontLeftYPos, kInvertLeftSide, kFrontLeftSteerMotorInverted, kFrontLeftEncoderInverted
        );

        frontRight = constantCreator.createModuleConstants(
            kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId, kFrontRightEncoderOffset,
            kFrontRightXPos, kFrontRightYPos, kInvertRightSide, kFrontRightSteerMotorInverted, kFrontRightEncoderInverted
        );

        backLeft = constantCreator.createModuleConstants(
            kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset,
            kBackLeftXPos, kBackLeftYPos, kInvertLeftSide, kBackLeftSteerMotorInverted, kBackLeftEncoderInverted
        );
        
        backRight = constantCreator.createModuleConstants(
            kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId, kBackRightEncoderOffset,
            kBackRightXPos, kBackRightYPos, kInvertRightSide, kBackRightSteerMotorInverted, kBackRightEncoderInverted
        );
        
        return this;
    }
}

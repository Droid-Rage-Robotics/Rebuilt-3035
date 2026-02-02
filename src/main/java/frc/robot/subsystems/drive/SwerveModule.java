package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.Supplier;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.DroidRageConstants;
import frc.robot.subsystems.drive.SwerveDriveConstants.SwerveDriveConfig;
import frc.robot.subsystems.drive.SwerveModuleConstants.POD;
import frc.utility.encoder.CANcoderEx;
import frc.utility.motor.MotorConstants.Direction;
import frc.utility.motor.MotorConstants;
import frc.utility.motor.TalonEx;
import lombok.Getter;

public class SwerveModule implements Sendable {
    public static class Constants {
        public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4);
        public static final double DRIVE_MOTOR_GEAR_RATIO = 1/6.75;//(50.0 / 16.0) * (16.0 / 28.0) * (45.0 / 15.0)=5.35714285714
        public static final double TURN_MOTOR_GEAR_RATIO = 1/ 21.42;

        public static final double DRIVE_ENCODER_ROT_2_METER = DRIVE_MOTOR_GEAR_RATIO * Math.PI * WHEEL_DIAMETER_METERS;
        public static final double DRIVE_ENCODER_RPM_2_METER_PER_SEC = DRIVE_ENCODER_ROT_2_METER / 60;
        public static final double READINGS_PER_REVOLUTION = 1;//4096

        //Used for the CANCoder
        public static final double TURN_ENCODER_ROT_2_RAD = 2 * Math.PI / READINGS_PER_REVOLUTION;
        public static final double TURN_ENCODER_ROT_2_RAD_SEC = TURN_ENCODER_ROT_2_RAD/60;

        //0.5 Change this to make the robot turn the turn motor as fast as possible
        //If strafing, the robot drifts to he front/back, then increase
                //.115


        public static final double DRIVE_SUPPLY_CURRENT_LIMIT = 35;//50, 40
        public static final double DRIVE_STATOR_CURRENT_LIMIT = 75;   //90, 80
        public static final double TURN_SUPPLY_CURRENT_LIMIT = 80;
    }

    private final MotorConstants driveMotorConstants;
    private final MotorConstants turnMotorConstants;

    @Getter private final TalonEx driveMotor;
    @Getter private final TalonEx turnMotor;

    private final CANcoderEx turnEncoder;

    private final PIDController turningPIDController;
    private final SimpleMotorFeedforward driveFeedforward;

    private final Subsystem subsystem;
    private final POD pod;

    private final StructPublisher<Rotation2d> turnPositionPub;
    private final StructPublisher<SwerveModuleState> moduleStatePub;
    private final StructPublisher<SwerveModulePosition> modulePositionPub;
    private final DoublePublisher driveVelocityPub;


    private SwerveModule(SwerveModuleConstants constants) {
        this.subsystem=constants.subsystem;
        this.pod=constants.podName;

        driveMotorConstants = new MotorConstants()
            .withDeviceId(constants.driveMotorId)
            .withCANBus(DroidRageConstants.driveCanBus)
            .withDirection(constants.driveMotorDirection)
            .withIdleMode(NeutralModeValue.Brake)
            .withConversionFactor(Constants.DRIVE_ENCODER_ROT_2_METER)
            .withSubsystem(subsystem)
            .withIsEnabled(constants.driveMotorIsEnabled)
            .withSupplyCurrentLimit(Constants.DRIVE_SUPPLY_CURRENT_LIMIT)
            .withStatorCurrentLimit(Constants.DRIVE_STATOR_CURRENT_LIMIT);

        driveMotor = TalonEx.createWithConstants(driveMotorConstants);

        turnMotorConstants = new MotorConstants()
            .withDeviceId(constants.turnMotorId)
            .withCANBus(DroidRageConstants.driveCanBus)
            .withDirection(constants.turnMotorDirection)
            .withIdleMode(NeutralModeValue.Coast)
            .withConversionFactor(Constants.TURN_ENCODER_ROT_2_RAD)
            .withSubsystem(subsystem)
            .withIsEnabled(constants.turnMotorIsEnabled)
            .withSupplyCurrentLimit(Constants.TURN_SUPPLY_CURRENT_LIMIT);

        turnMotor = TalonEx.createWithConstants(driveMotorConstants);

        turnEncoder = CANcoderEx.create(constants.encoderId, DroidRageConstants.driveCanBus)
            .withDirection(SensorDirectionValue.CounterClockwise_Positive)
            .withMagnetOffset(constants.encoderOffsetRad/Constants.TURN_ENCODER_ROT_2_RAD)
            .withAbsoluteSensorDiscontinuityPoint(0.5);

        turningPIDController = new PIDController(SwerveDriveConfig.TURN_KP.getValue(), 0.0, 0.0);
        turningPIDController.enableContinuousInput(-Math.PI, Math.PI);// Was -Math.PI, Math.PI but changed to 0 and 2PI
        
        driveFeedforward = new SimpleMotorFeedforward(SwerveDriveConfig.DRIVE_KS.getValue(),
                SwerveDriveConfig.DRIVE_KV.getValue());

        resetDriveEncoder();

        String baseTopic = "SwerveModules/" + pod.getName();

        var nt = NetworkTableInstance.getDefault();
        turnPositionPub = nt.getStructTopic(baseTopic + "/TurnPosition", Rotation2d.struct).publish();
        moduleStatePub = nt.getStructTopic(baseTopic + "/State", SwerveModuleState.struct).publish();
        modulePositionPub = nt.getStructTopic(baseTopic + "/Position", SwerveModulePosition.struct).publish();
        driveVelocityPub = nt.getDoubleTopic(baseTopic + "/DriveVelocity").publish();
    }

    public void updateTelemetry() {
        turnPositionPub.set(new Rotation2d(getTurningPosition()));
        moduleStatePub.set(getState());
        modulePositionPub.set(getPosition());
        driveVelocityPub.set(getDriveVelocity());
    }
    

    public static SwerveModule createWithConstants(SwerveModuleConstants constants) {
        return new SwerveModule(constants);
    }

    public double getDrivePos() {
        return driveMotor.getPosition().in(Rotations);
    }

    public String getPod() {
        return pod.toString();
    }
    
    public double getTurningPosition() {
        return turnEncoder.getAbsolutePosition().in(Radians);
    }

    public double getDriveVelocity(){
        return driveMotor.getVelocity().in(RotationsPerSecond);
    }

    public void resetDriveEncoder(){
        driveMotor.resetEncoder(0);
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePos(), new Rotation2d(getTurningPosition()));
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setState(SwerveModuleState state) {
        SwerveModuleState desiredState = state;
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        desiredState.optimize(getState().angle);
        driveMotor.setPower(state.speedMetersPerSecond / SwerveDriveConstants.SwerveDriveConfig.PHYSICAL_MAX_SPEED_METERS_PER_SECOND.getValue());
        turnMotor.setPower((turningPIDController.calculate(getTurningPosition(), desiredState.angle.getRadians()))*1);
    }

    public void setFeedforwardState(SwerveModuleState state) {
        SwerveModuleState desiredState = state;
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        desiredState.optimize(getState().angle);
        driveMotor.setVoltage(driveFeedforward.calculate(state.speedMetersPerSecond));
        turnMotor.setPower(turningPIDController.calculate(getTurningPosition(), desiredState.angle.getRadians()));
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty(getPod(), () -> getTurningPosition(), null);
    }

    /* ---------------- Utility ---------------- */

    public void stop(){
        driveMotor.setPower(0);
        turnMotor.setPower(0);
    }

    public void coastMode() {
        driveMotor.withIdleMode(NeutralModeValue.Coast);
        turnMotor.withIdleMode(NeutralModeValue.Coast);
    }

    public void brakeMode() {
        driveMotor.withIdleMode(NeutralModeValue.Brake);
        turnMotor.withIdleMode(NeutralModeValue.Brake);
    }

    public void brakeAndCoastMode() {
        driveMotor.withIdleMode(NeutralModeValue.Brake);
        turnMotor.withIdleMode(NeutralModeValue.Coast);
    }

    public void getTurnVoltage(){
        turnMotor.getVoltage();
    }

    public void setTurnMotorIsEnabled(boolean isEnabled){
        turnMotor.withIsEnabled(isEnabled);
    }
    
    public void setDriveMotorIsEnabled(boolean isEnabled) {
        driveMotor.withIsEnabled(isEnabled);
    }

    
}
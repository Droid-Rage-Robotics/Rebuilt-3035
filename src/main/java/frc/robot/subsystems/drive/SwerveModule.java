package frc.robot.subsystems.drive;

import java.util.function.Supplier;

import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.DroidRageConstants;
import frc.robot.subsystems.drive.SwerveDriveConstants.SwerveDriveConfig;
import frc.utility.encoder.CANcoderEx;
import frc.utility.motor.MotorBase.Direction;
import frc.utility.motor.MotorBase.ZeroPowerMode;
import frc.utility.motor.TalonEx;
import lombok.Getter;

public class SwerveModule implements Sendable {
    public enum POD{
        FL,
        BL,
        FR,
        BR
    }

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

    @Getter private TalonEx driveMotor;
    @Getter private TalonEx turnMotor;

    // private CANcoder turnEncoder;
    private CANcoderEx turnEncoder;
    // private CANcoderConfiguration encoderConfig = new CANcoderConfiguration();

    private PIDController turningPIDController;
    private SimpleMotorFeedforward driveFeedforward;

    private Subsystem subsystem;
    private SwerveModule.POD podName;

    private SwerveModule() {}

    public static SwerveModule create() {
        return new SwerveModule();
    }

    public SwerveModule withSubsystem(Subsystem subsystem, SwerveModule.POD pod) {
        this.podName=pod;
        this.subsystem=subsystem;
        return this;
    }

    public SwerveModule withDriveMotor(int driveMotorId, Direction direction, boolean isEnabled) {
        driveMotor = TalonEx.create(driveMotorId, DroidRageConstants.driveCanBus)
            .withDirection(direction)
            .withIdleMode(ZeroPowerMode.Brake)
            .withConversionFactor(Constants.DRIVE_ENCODER_ROT_2_METER)
            .withSubsystem(subsystem)
            .withIsEnabled(isEnabled)
            .withSupplyCurrentLimit(Constants.DRIVE_SUPPLY_CURRENT_LIMIT)
            .withStatorCurrentLimit(Constants.DRIVE_STATOR_CURRENT_LIMIT);
        return this;
    }

    public SwerveModule withTurnMotor(int turnMotorId, Direction direction, boolean isEnabled) {
        turnMotor = TalonEx.create(turnMotorId, DroidRageConstants.driveCanBus)
            .withDirection(direction)
            .withIdleMode(ZeroPowerMode.Coast)
            .withConversionFactor(Constants.TURN_ENCODER_ROT_2_RAD)
            .withSubsystem(subsystem)
            .withIsEnabled(isEnabled)
            .withSupplyCurrentLimit(Constants.TURN_SUPPLY_CURRENT_LIMIT);
        return this;
    }

    public SwerveModule withEncoder(int absoluteEncoderId, Supplier<Double> absoluteEncoderOffsetRad, SensorDirectionValue direction) {
        turnEncoder = CANcoderEx.create(absoluteEncoderId, DroidRageConstants.driveCanBus)
            .withDirection(SensorDirectionValue.CounterClockwise_Positive)
            .withMagnetOffset(absoluteEncoderOffsetRad.get()/Constants.TURN_ENCODER_ROT_2_RAD)
            .withAbsoluteSensorDiscontinuityPoint(0.5);
        
        turningPIDController = new PIDController(SwerveDriveConfig.TURN_KP.getValue(), 0.0, 0.0);
        turningPIDController.enableContinuousInput(-Math.PI, Math.PI);// Was -Math.PI, Math.PI but changed to 0 and 2PI
        
        driveFeedforward = new SimpleMotorFeedforward(SwerveDriveConfig.DRIVE_KS.getValue(),
                SwerveDriveConfig.DRIVE_KV.getValue());

        resetDriveEncoder();
        return this;
    }

    public double getDrivePos() {
        return driveMotor.getPosition();
    }

    public String getPodName() {
        return podName.toString();
    }
    
    public double getTurningPosition() {
        return turnEncoder.getAbsolutePosition()*Constants.TURN_ENCODER_ROT_2_RAD;
    }

    public double getDriveVelocity(){
        return driveMotor.getVelocity();
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

    // private SlewRateLimiter speedLimiter = new SlewRateLimiter()

    // public SwerveModuleState clampStateForHeight(SwerveModuleState state) {
    //     double scale = getHeightScale();
    
    //     double maxSpeed = Constants.PHYSICAL_MAX_SPEED_METERS_PER_SECOND * scale;
    
    //     // Limit speed
    //     double limitedSpeed = MathUtil.clamp(
    //         state.speedMetersPerSecond,
    //         -maxSpeed,
    //         maxSpeed
    //     );
    
    //     return new SwerveModuleState(limitedSpeed, state.angle);
    // }

    // private double lastSpeed = 0;

    // public double clampAcceleration(double desiredSpeed) {
    //     double scale = getHeightScale();

    //     // double maxAccel = Constants.PHYSICAL_MAX_ACCELERATION_ * scale;

    //     double maxAccel = 0;

    //     double dt = 0.02; // 20ms loop

    //     double maxDelta = maxAccel * dt;

    //     double delta = desiredSpeed - lastSpeed;

    //     if (delta > maxDelta) delta = maxDelta;
    //     if (delta < -maxDelta) delta = -maxDelta;

    //     lastSpeed += delta;
    //     return lastSpeed;
    // }

    

    // public void setFeedforwardStated(SwerveModuleState state) {
    //     SwerveModuleState desiredState = state;
    //     if (Math.abs(state.speedMetersPerSecond) < 0.001) {
    //         stop();
    //         return;
    //     }
        
    //     desiredState.optimize(getState().angle);

        

    //     driveMotor.setVoltage(driveFeedforward.calculate(state.speedMetersPerSecond));
    //     turnMotor.setPower(turningPIDController.calculate(getTurningPosition(), desiredState.angle.getRadians()));
    // }


    public void stop(){
        driveMotor.setPower(0);
        turnMotor.setPower(0);
    }

    public void coastMode() {
        driveMotor.withIdleMode(ZeroPowerMode.Coast);
        turnMotor.withIdleMode(ZeroPowerMode.Coast);
    }

    public void brakeMode() {
        driveMotor.withIdleMode(ZeroPowerMode.Brake);
        turnMotor.withIdleMode(ZeroPowerMode.Brake);
    }

    public void brakeAndCoastMode() {
        driveMotor.withIdleMode(ZeroPowerMode.Brake);
        turnMotor.withIdleMode(ZeroPowerMode.Coast);
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

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty(getPodName(), () -> getTurningPosition(), null);
    }
}
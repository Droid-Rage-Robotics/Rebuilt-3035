package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import java.util.concurrent.atomic.AtomicReference;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.DroidRageConstants;
import frc.robot.subsystems.drive.DriveConstants.GearRatio;
import frc.robot.subsystems.drive.DriveConstants.ModuleConstants;
import frc.robot.subsystems.drive.DriveConstants.SwerveDriveConfig;
import frc.robot.subsystems.drive.SwerveModuleConstants.POD;
import frc.utility.encoder.CANcoderEx;
import frc.utility.motor.MotorConstants;
import frc.utility.motor.TalonEx;
import lombok.Getter;

public class SwerveModule implements Sendable {
    private final MotorConstants driveMotorConstants;
    private final MotorConstants turnMotorConstants;

    @Getter private final TalonEx driveMotor;
    @Getter private final TalonEx turnMotor;

    private final CANcoderEx turnEncoder;

    private final PIDController turningPIDController;
    
    private final PIDController drivePIDController;
    private final SimpleMotorFeedforward driveFeedforward;

    private final Subsystem subsystem;
    private final POD pod;

    private final StructPublisher<SwerveModuleState> moduleStatePub;
    private final StructPublisher<SwerveModulePosition> modulePositionPub;

    private final AtomicReference<Double> driveVoltage = new AtomicReference<Double>(0.0);
    private final AtomicReference<Double> turnVoltage = new AtomicReference<Double>(0.0);


    private static final DCMotor driveMotorModel = DCMotor.getKrakenX60(1);
    private static final DCMotor turnMotorModel = DCMotor.getKrakenX44(1);

    private final DCMotorSim driveSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(driveMotorModel, 0.025, DriveConstants.GearRatio.R3.getGearRatio()),
        driveMotorModel
    );
  
    private final DCMotorSim turnSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(turnMotorModel, 0.004, DriveConstants.GearRatio.TURN.getGearRatio()),
        turnMotorModel
    );


    private SwerveModule(SwerveModuleConstants constants) {
        this.subsystem=constants.subsystem;
        this.pod=constants.podName;

        driveMotorConstants = new MotorConstants()
            .withDeviceId(constants.driveMotorId)
            .withCANBus(DroidRageConstants.driveCanBus)
            .withDirection(constants.driveMotorDirection)
            .withIdleMode(NeutralModeValue.Brake)
            .withConversionFactor(ModuleConstants.DRIVE_ENCODER_ROT_2_METER)
            .withSubsystem(subsystem)
            .withIsEnabled(constants.driveMotorIsEnabled)
            .withSupplyCurrentLimit(ModuleConstants.DRIVE_SUPPLY_CURRENT_LIMIT)
            .withStatorCurrentLimit(ModuleConstants.DRIVE_STATOR_CURRENT_LIMIT);

        driveMotor = TalonEx.createWithConstants(driveMotorConstants);

        turnMotorConstants = new MotorConstants()
            .withDeviceId(constants.turnMotorId)
            .withCANBus(DroidRageConstants.driveCanBus)
            .withDirection(constants.turnMotorDirection)
            .withIdleMode(NeutralModeValue.Coast)
            .withConversionFactor(ModuleConstants.TURN_ENCODER_ROT_2_RAD)
            .withSubsystem(subsystem)
            .withIsEnabled(constants.turnMotorIsEnabled)
            .withSupplyCurrentLimit(ModuleConstants.TURN_SUPPLY_CURRENT_LIMIT);

        turnMotor = TalonEx.createWithConstants(turnMotorConstants);

        turnEncoder = CANcoderEx.create(constants.encoderId, DroidRageConstants.driveCanBus)
            .withDirection(SensorDirectionValue.CounterClockwise_Positive)
            .withMagnetOffset(constants.encoderOffsetRad/ModuleConstants.TURN_ENCODER_ROT_2_RAD)
            .withAbsoluteSensorDiscontinuityPoint(0.5);

        turningPIDController = new PIDController(SwerveDriveConfig.TURN_KP.getValue(), 0.0, 0.0);
        turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
        
        
        drivePIDController = new PIDController(0, 0, 0);
        driveFeedforward = new SimpleMotorFeedforward(
            SwerveDriveConfig.DRIVE_KS.getValue(),
            SwerveDriveConfig.DRIVE_KV.getValue()
        );

        resetDriveEncoder();

        String baseTopic = "SwerveModules/" + pod.getName();

        var nt = NetworkTableInstance.getDefault().getTable("Drivetrain");
        moduleStatePub = nt.getStructTopic(baseTopic + "/State", SwerveModuleState.struct).publish();
        modulePositionPub = nt.getStructTopic(baseTopic + "/Position", SwerveModulePosition.struct).publish();
    }

    public void simulationPeriodic() {
        var batteryVoltage = RobotController.getBatteryVoltage();
        
        driveMotor.getSimState().setSupplyVoltage(batteryVoltage);
        turnMotor.getSimState().setSupplyVoltage(batteryVoltage);
        turnEncoder.getSimState().setSupplyVoltage(batteryVoltage);
        
        driveSim.setInputVoltage(MathUtil.clamp(driveVoltage.get(), -batteryVoltage, batteryVoltage));
        turnSim.setInputVoltage(MathUtil.clamp(turnVoltage.get(), -batteryVoltage, batteryVoltage));
        
        driveSim.update(DroidRageConstants.LOOP_PERIOD_SECS);
        turnSim.update(DroidRageConstants.LOOP_PERIOD_SECS);

        driveMotor.getSimState().setRawRotorPosition(driveSim.getAngularPosition());
        turnMotor.getSimState().setRawRotorPosition(turnSim.getAngularPosition());
        turnEncoder.getSimState().setRawPosition(turnSim.getAngularPosition().times(GearRatio.TURN.getConversionFactor()));

        driveMotor.getSimState().setRotorVelocity(driveSim.getAngularVelocity());
        turnMotor.getSimState().setRotorVelocity(turnSim.getAngularVelocity());
        turnEncoder.getSimState().setVelocity(turnSim.getAngularVelocity().times(GearRatio.TURN.getConversionFactor()));

    }

    public void updateTelemetry() {
        moduleStatePub.set(getState());
        modulePositionPub.set(getPosition());
    }
    

    public static SwerveModule createWithConstants(SwerveModuleConstants constants) {
        return new SwerveModule(constants);
    }

    public Distance getDrivePosition() {
        return Meters.of(driveMotor.getPosition().in(Radians) * ModuleConstants.WHEEL_DIAMETER.in(Meters)/2);
    }

    public String getPod() {
        return pod.toString();
    }
    
    public double getTurningPosition() {
        return turnEncoder.getAbsolutePosition().in(Radians);
    }

    public Rotation2d getRotation2d() {
        return turnEncoder.getRotation2d();
    }

    public LinearVelocity getDriveVelocity(){
        return MetersPerSecond.of(driveMotor.getVelocity().in(RadiansPerSecond) * ModuleConstants.WHEEL_DIAMETER.in(Meters)/2);
    }

    public void resetDriveEncoder(){
        driveMotor.resetEncoder(0);
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), getRotation2d());
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveVelocity(), getRotation2d());
    }

    // public void setState(SwerveModuleState state) {
    //     SwerveModuleState desiredState = state;
    //     if (Math.abs(state.speedMetersPerSecond) < 0.001) {
    //         stop();
    //         return;
    //     }
    //     desiredState.optimize(getState().angle);
    //     driveMotor.setPower(state.speedMetersPerSecond / DriveConstants.SwerveDriveConfig.PHYSICAL_MAX_SPEED_METERS_PER_SECOND.getValue());
    //     turnMotor.setPower((turningPIDController.calculate(getTurningPosition(), desiredState.angle.getRadians()))*1);
    // }

    public void setFeedforwardState(SwerveModuleState state) {
        var currentState = getState();
        
        SwerveModuleState desiredState = state;
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        desiredState.optimize(getState().angle);
        
        driveVoltage.set(
            drivePIDController.calculate(currentState.speedMetersPerSecond, desiredState.speedMetersPerSecond)
            +driveFeedforward.calculate(desiredState.speedMetersPerSecond));
        
        turnVoltage.set(
            turningPIDController.calculate(currentState.angle.getRadians(), desiredState.angle.getRadians())
        );
        
        driveMotor.setVoltage(driveVoltage.get());
        turnMotor.setVoltage(turnVoltage.get());
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
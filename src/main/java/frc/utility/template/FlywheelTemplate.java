package frc.utility.template;

import static edu.wpi.first.units.Units.*;

import java.util.concurrent.atomic.AtomicReference;
import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.utility.TelemetryUtils;
import frc.utility.TelemetryUtils.Dashboard;
import frc.utility.TelemetryUtils.TelemetryUpdater;
import frc.utility.devices.motor.MotorConstants;
import frc.utility.devices.motor.TalonEx;

public class FlywheelTemplate extends SubsystemBase implements Dashboard, TelemetryUpdater {
    private final TalonEx[] motors;
    private final double maxSpeed;
    private final double minSpeed;
    private final int mainNum;
    private final String name;
    private final SysIdRoutine sysIdRoutine;
    private boolean sysIdActive = false;

    private final boolean isEnabled;

    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
    private final VoltageOut voltageOut = new VoltageOut(0);
    private final AtomicReference<AngularVelocity> targetVelocity = new AtomicReference<AngularVelocity>(RotationsPerSecond.zero());

    public FlywheelTemplate(
        boolean isEnabled,
        SubsystemConstants constants,
        MotorConstants...motorConstants
    ){
        this.isEnabled=isEnabled;
        this.maxSpeed=constants.maxVelocity.in(RotationsPerSecond);
        this.minSpeed=constants.minVelocity.in(RotationsPerSecond);
        this.mainNum=constants.mainNum;
        this.name=constants.name;

        this.motors = new TalonEx[motorConstants.length];
        
        for (MotorConstants m_motorConstants : motorConstants) {
            m_motorConstants.subsystem=this;
            m_motorConstants.isEnabled=isEnabled;
        }

        for (int i = 0; i < motorConstants.length; i++) {
            this.motors[i] = TalonEx.createWithConstants(motorConstants[i]);
        }

        for (int i = 0; i < motors.length; i++) {
            if (i != mainNum) {
                motors[i].getMotor().setControl(
                    new Follower(motors[mainNum].getMotor().getDeviceID(), motorConstants[i].alignment)
                );
            }
        }

        var motorConfig = motors[mainNum].getConfig();

        motorConfig.Feedback.SensorToMechanismRatio=constants.gearRatio;
        
        motorConfig.Slot0.kP = constants.kP;
        motorConfig.Slot0.kI = constants.kI;
        motorConfig.Slot0.kD = constants.kD;
        motorConfig.Slot0.kV = constants.kV;
        motorConfig.Slot0.kA = constants.kA;
        motorConfig.Slot0.kS = constants.kS;

        motors[mainNum].getMotor().getConfigurator().apply(motorConfig);

        TelemetryUtils.registerTelemetry(this);

        sysIdRoutine = getSysIdRoutine();
    }

    /* ---------------- Dashboard ---------------- */
    
    @Override
    public void elasticInit() {}

    @Override
    public void updateTelemetry() {
        Logger.recordOutput(name + "/Target Velocity", getTargetVelocity());
        Logger.recordOutput(name +"/Current Velocity", getVelocity());
        Logger.recordOutput(name + "/Applied Voltage", getVoltage());
        Logger.recordOutput(name + "/Torque Current", getCurrent());
    }

    @Override
    public void practiceWriters() {}

    @Override
    public void alerts() {}

    /* ---------------- Periodic Control Loop ---------------- */
    
    @Override
    public void periodic() {
        if (!sysIdActive) {
            motors[mainNum].setControl(velocityRequest.withVelocity(targetVelocity.get()));
        }
    }

    /* ---------------- Commands ---------------- */

    public Command setTargetVelocityCommand(AngularVelocity target){
        return new InstantCommand(() -> setTargetVelocity(target));
    }

    /* ---------------- Manual Goal Control ---------------- */

    public void setTargetVelocity(AngularVelocity target) {
        double clamped = MathUtil.clamp(
            target.in(RotationsPerSecond), 
            minSpeed, 
            maxSpeed);
        
        targetVelocity.set(RotationsPerSecond.of(clamped));
    }

    public AngularVelocity getTargetVelocity() {
        return targetVelocity.get();
    }

    public AngularVelocity getVelocityError() {
        return RotationsPerSecond.of(motors[mainNum].getMotor().getClosedLoopError().getValueAsDouble());
    }

    /* ---------------- Sensor Access ---------------- */

    public Angle getCurrentAngle() {
        return motors[mainNum].getPosition();
    }

    public AngularVelocity getVelocity() {
        return motors[mainNum].getVelocity();
    }
    
    public double getVoltage() {
        return motors[mainNum].getVoltage();
    }

    public Current getCurrent() {
        return motors[mainNum].getStatorCurrent();
    }

    /* ---------------- Motor Control ---------------- */
    
    protected void setVoltage(double voltage) {
        if (isEnabled) {
            // for (TalonEx motor: motors) {
            //     motor.setVoltage(voltage);
            // }
            motors[mainNum].setControl(voltageOut.withOutput(voltage));
        }
    }

    public void setVoltage(Voltage voltage) {
        if (isEnabled) {
            // for (TalonEx motor: motors) {
            //     motor.setVoltage(voltage);
            // }
            motors[mainNum].setControl(voltageOut.withOutput(voltage));
        }
    }

    /* ---------------- SysId ---------------- */

    private SysIdRoutine getSysIdRoutine() {
        return new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                Volts.of(9),
                Seconds.of(10),
                null
            ), 
            new SysIdRoutine.Mechanism(
                (voltage) -> {
                    motors[mainNum].setControl(voltageOut.withOutput(voltage));
                },
                (log) -> {
                    log.motor("motor")
                        .voltage(Volts.of(getVoltage()))
                        .angularPosition(this.getCurrentAngle())
                        .angularVelocity(this.getVelocity());
                }, 
                this
            )
        );
    }

    public Command getSysIdCommand() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> sysIdActive = true),
            sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward),
            new WaitCommand(10),
            sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse),
            new WaitCommand(10),
            sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward),
            new WaitCommand(10),
            sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse),
            new WaitCommand(10),
            new InstantCommand(() -> sysIdActive = false)
        );
    }

    /* ---------------- Utility ---------------- */

    public TalonEx getMotor() {
        return motors[mainNum];
    }

    public TalonEx[] getAllMotor() {
        return motors;
    }

    public boolean atSetpoint() {
        return (Math.abs(getVelocityError().in(RotationsPerSecond)) < 5);
    }
}
package frc.utility.template;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.utility.TelemetryUtils;
import frc.utility.TelemetryUtils.Dashboard;
import frc.utility.motor.TalonEx;
import frc.utility.motor.MotorConstants;

public class FlywheelTemplate extends SubsystemBase implements Dashboard {
    private final TalonEx[] motors;
    private final PIDController controller;
    private final SimpleMotorFeedforward feedforward;
    private final double maxSpeed;
    private final double minSpeed;
    private final double conversionFactor;
    private final int mainNum;
    private final String name;
    private final SysIdRoutine sysIdRoutine;
    private boolean sysIdActive = false;

    private final boolean isEnabled;

    public FlywheelTemplate(
        boolean isEnabled,
        PIDController controller,
        SimpleMotorFeedforward feedforward,
        SubsystemConstants constants,
        MotorConstants...motorConstants
    ){
        this.isEnabled=isEnabled;
        this.controller=controller;
        this.feedforward=feedforward;
        this.maxSpeed=constants.upperLimit;
        this.minSpeed=constants.lowerLimit;
        this.conversionFactor=constants.conversionFactor;
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

        TelemetryUtils.registerDashboard(this);

        sysIdRoutine = getSysIdRoutine();
    }

    /* ---------------- Dashboard ---------------- */
    
    @Override
    public void elasticInit() {
        SmartDashboard.putData(name, this);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Target Speed", controller::getSetpoint, null);
        builder.addDoubleProperty("Current Speed", () -> this.getVelocity().in(RotationsPerSecond), null);
        builder.addDoubleProperty("Applied Voltage", this::getVoltage, null);
        builder.addDoubleProperty("Torque Current", () -> getCurrent().in(Amp), null);
    }

    @Override
    public void practiceWriters() {}

    @Override
    public void alerts() {}

    /* ---------------- Periodic Control Loop ---------------- */
    
    @Override
    public void periodic() {
        if (!sysIdActive) {
            setVoltage(
                controller.calculate(getVelocity().in(RotationsPerSecond), controller.getSetpoint())
                +feedforward.calculate(controller.getSetpoint())
            );
        }
    }

    /* ---------------- Commands ---------------- */

    public Command setTargetVelocityCommand(double target){
        return runOnce(() -> setTargetVelocity(target));
    }

    /* ---------------- Manual Goal Control ---------------- */

    public void setTargetVelocity(double target) {
        double clamped = MathUtil.clamp(
            target, 
            minSpeed, 
            maxSpeed);
        
        controller.setSetpoint(clamped);
    }

    public double getTargetVelocity(){
        return controller.getSetpoint();
    }

    /* ---------------- Sensor Access ---------------- */

    public Angle getCurrentAngle() {
        return motors[mainNum].getPosition().times(conversionFactor);
    }

    public AngularVelocity getVelocity() {
        return motors[mainNum].getVelocity().times(conversionFactor);
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
            for (TalonEx motor: motors) {
                motor.setVoltage(voltage);
            }
        }
    }

    public void setVoltage(Voltage voltage) {
        if (isEnabled) {
            for (TalonEx motor: motors) {
                motor.setVoltage(voltage);
            }
        }
    }

    /* ---------------- SysId ---------------- */

    private SysIdRoutine getSysIdRoutine() {
        motors[mainNum].getMotor().getVelocity().setUpdateFrequency(100);
        return new SysIdRoutine(
            new SysIdRoutine.Config(), 
            new SysIdRoutine.Mechanism(
                this::setVoltage,
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
            new WaitCommand(3),
            sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse),
            new WaitCommand(3),
            sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward),
            new WaitCommand(3),
            sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse),
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
        return controller.atSetpoint();
    }
}

// isElementIn = this::(getTargetPosition() - getEncoderPosition() > 40);

    // private final Sendable isElementIn =  new Sendable() {
    //     @Override
    //     public void initSendable(SendableBuilder builder) {
    //         builder.setSmartDashboardType("Boolean Box");
    //         builder.addBooleanProperty("Is Element In", () -> (getTargetPosition() - getVelocity() > 40), null);
    //     }
    // };
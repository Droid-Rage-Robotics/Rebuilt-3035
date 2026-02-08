package frc.utility.template;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.utility.TelemetryUtils.Dashboard;
import frc.utility.encoder.CANcoderEx;
import frc.utility.encoder.EncoderConstants;
import frc.utility.motor.TalonEx;
import frc.utility.motor.MotorConstants;
import frc.utility.template.SubsystemConstants.EncoderType;

public class ElevatorTemplate extends SubsystemBase implements Dashboard {
    private final TalonEx[] motors;
    private final Optional<CANcoderEx> encoder;
    private final ProfiledPIDController controller;
    private final ElevatorFeedforward feedforward;

    private final double maxPosition;
    private final double minPosition;
    private final double conversionFactor;
    private final int mainNum;

    private final boolean isEnabled;

    private Distance goalPosition = Meters.zero();
    
    public ElevatorTemplate(
        boolean isEnabled,
        ProfiledPIDController controller,
        ElevatorFeedforward feedforward,
        SubsystemConstants constants,
        EncoderConstants encoderConstants,
        MotorConstants... motorConstants
    ) {
        this.mainNum=constants.mainNum;
        this.controller=controller;
        this.feedforward=feedforward;
        this.maxPosition=constants.lowerLimit;
        this.minPosition=constants.upperLimit;
        this.conversionFactor=constants.conversionFactor;
        this.isEnabled=isEnabled;

        if (constants.encoderType == EncoderType.ABSOLUTE) {
            if (encoderConstants == null) {
                throw new NullPointerException("Encoder constants required for absolute encoder");
            }
            this.encoder = Optional.of(CANcoderEx.createWithConstants(encoderConstants));
        } else {
            this.encoder = Optional.empty();
        }

        this.motors = new TalonEx[motorConstants.length];
        
        for (MotorConstants m_motorConstants : motorConstants) {
            m_motorConstants.subsystem=this;
            m_motorConstants.isEnabled=isEnabled;
        }

        for (int i = 0; i < motorConstants.length; i++) {
            this.motors[i] = TalonEx.createWithConstants(motorConstants[i]);
        }
    }

    /* ---------------- Dashboard ---------------- */

    @Override
    public void elasticInit() {
        SmartDashboard.putData(getName(), this);
        SmartDashboard.putData(getName() + "/Reset Encoder", runOnce(this::resetEncoder));
    }

    @Override public void practiceWriters() {}
    @Override public void alerts() {}

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Goal Position (inches)", () -> getGoalPosition().in(Inches), null);
        builder.addDoubleProperty("Current Position (inches)", () -> getPosition().in(Inches), null);
        builder.addDoubleProperty("Position Setpoint (inches)", () -> Units.metersToInches(getPositionSetpoint()), null);
        builder.addDoubleProperty("Velocity Setpoint (m/s)", this::getVelocitySetpoint, null);
        builder.addDoubleProperty("Current Velocity (m/s)", () -> getVelocity().in(MetersPerSecond), null);
        builder.addDoubleProperty("Applied Voltage", this::getVoltage, null);
        builder.addDoubleProperty("Position Error (deg)", () -> Math.toDegrees(controller.getPositionError()), null);
    }

    /* ---------------- Periodic Control Loop ---------------- */

    @Override
    public void periodic() {
        double meter = getPosition().in(Meters);

        double pidOut = controller.calculate(meter);
        var setpoint = controller.getSetpoint();

        double ffOut = feedforward.calculate(setpoint.velocity);

        setVoltage(pidOut + ffOut);
    }

    @Override
    public void simulationPeriodic() {
        periodic();
    }

    /* ---------------- Commands ---------------- */

    public Command setTargetPositionCommand(double inches) {
        return new InstantCommand(() -> setTargetPositionInches(inches), this);
    }

    /* ---------------- Manual Goal Control ---------------- */

    public void setTargetPositionInches(double inches) {
        setGoalPosition(Inches.of(inches));
    }

    public void setGoalPosition(Distance position) {
        double clamped = MathUtil.clamp(
            position.in(Meters),
            minPosition,
            maxPosition
        );

        goalPosition = Distance.ofBaseUnits(clamped, Meters);
        controller.setGoal(clamped);
    }

    public Distance getGoalPosition() {
        return goalPosition;
    }

    public double getVelocitySetpoint() {
        return Math.toDegrees(controller.getSetpoint().velocity);
    }

    public double getPositionSetpoint() {
        return Math.toDegrees(controller.getSetpoint().position);
    }

    
    /* ---------------- Sensor Access ---------------- */

    public Distance getPosition() {
        return Meters.of(encoder
            .map(enc -> enc.getAbsolutePosition().in(Radians) * conversionFactor)
            .orElse(motors[mainNum].getPosition().in(Radians) * conversionFactor)
        );
    }

    public LinearVelocity getVelocity() {
        return MetersPerSecond.of(encoder
            .map(enc -> enc.getVelocity().in(RadiansPerSecond) * conversionFactor)
            .orElse(motors[mainNum].getVelocity().in(RadiansPerSecond) * conversionFactor));
    }
    
    public double getVoltage() {
        return motors[mainNum].getVoltage();
    }

    /* ---------------- Motor Control ---------------- */

    public void setVoltage(double voltage) {
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
    
    public void resetEncoder() {
        for (TalonEx motor: motors) {
            motor.resetEncoder(0);
        }
    }

    /* ---------------- SysId ---------------- */

    private SysIdRoutine getSysIdRoutine() {
        return new SysIdRoutine(new SysIdRoutine.Config(), 
            new SysIdRoutine.Mechanism(
                (voltage) -> {
                    // Only apply voltage if within safe bounds
                    double currentPos = getPosition().in(Meters);
                    if (currentPos >= minPosition || currentPos <= maxPosition) {
                        setVoltage(voltage);
                    } else {
                        setVoltage(0); // Stop if at limits
                    }
                }, 
                (log) -> {
                    log.motor("motor")
                        .voltage(Volts.of(getVoltage()))
                        .linearPosition(this.getPosition())
                        .linearVelocity(this.getVelocity());
                }, 
                this
            )
        );
    }

    public Command getSysIdCommand() {
        return new SequentialCommandGroup(
            getSysIdRoutine().quasistatic(SysIdRoutine.Direction.kForward)
                .until(this::isAtUpperLimit),
            new WaitCommand(0.1),
            getSysIdRoutine().quasistatic(SysIdRoutine.Direction.kReverse)
                .until(this::isAtLowerLimit),
            new WaitCommand(0.1),
            getSysIdRoutine().dynamic(SysIdRoutine.Direction.kForward)
                .until(this::isAtUpperLimit),
            new WaitCommand(0.1),
            getSysIdRoutine().dynamic(SysIdRoutine.Direction.kReverse)
                .until(this::isAtLowerLimit)
        );
    }

    /* ---------------- Utility ---------------- */
    
    public TalonEx getMotor(){
        return motors[mainNum];
    }
    
    public TalonEx[] getAllMotor() {
        return motors;
    }

    public boolean atGoal(){
        return controller.atGoal();
    }

    private boolean isAtUpperLimit() {
        return getPosition().in(Meters) >= maxPosition - 0.05; // 0.05 cm buffer
    }

    private boolean isAtLowerLimit() {
        return getPosition().in(Meters) <= minPosition + 0.05; // 0.05 cm buffer
    }
}
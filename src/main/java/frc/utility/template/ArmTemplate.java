package frc.utility.template;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.utility.TelemetryUtils;
import frc.utility.TelemetryUtils.Dashboard;
import frc.utility.TelemetryUtils.TelemetryUpdater;
import frc.utility.io.ArmIO;
import frc.utility.io.ArmIOInputsAutoLogged;
import frc.utility.template.Constants.ArmConstants;


public class ArmTemplate extends SubsystemBase implements Dashboard, TelemetryUpdater {
    private final ArmIO io;
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

    private final double minAngleRad;
    private final double maxAngleRad;
    private final Angle resetAngle;
    // private final SubsystemConstants constants;

    private final boolean isEnabled;
    private double goalAngleRad = 0.0;

    public boolean controlLoopEnabled = true;

    private final String name;
    private final Alert motorDisconnectedAlert;
    private final Alert tempAlert;

    public ArmTemplate(
            boolean isEnabled,
            ArmConstants constants,
            ArmIO io
    ) {
        // this.constants = constants;
        this.name = constants.name;
        this.minAngleRad = constants.minAngle.in(Radians);
        this.maxAngleRad = constants.maxAngle.in(Radians);
        this.resetAngle = constants.resetAngle;
        this.isEnabled = isEnabled;
        this.io = io;

        motorDisconnectedAlert =
            new Alert(name + " motor disconnected", AlertType.kWarning);
        tempAlert =
            new Alert(name + " motor temperature high", AlertType.kWarning);

        TelemetryUtils.registerDashboard(this);
        TelemetryUtils.registerTelemetry(this);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs(name, inputs);

        motorDisconnectedAlert.set(!inputs.motorConnected);
        tempAlert.set(inputs.tempCelsius > 70.0);

        if (controlLoopEnabled) {
            io.setGoalAngleRad(goalAngleRad);
        }
    }

    @Override
    public void simulationPeriodic() {
        periodic();
    }

    @Override
    public void elasticInit() {
        SmartDashboard.putData(name + "/Reset Encoder", resetEncoderCommand());
    }

    @Override
    public void practiceWriters() {}

    @Override
    public void alerts() {}

    @Override
    public void updateTelemetry() {
        Logger.recordOutput(name + "/Goal Angle", Math.toDegrees(goalAngleRad));
        Logger.recordOutput(name + "/Current Angle", Math.toDegrees(inputs.positionRad));
        Logger.recordOutput(name + "/Position Setpoint", Math.toDegrees(inputs.closedLoopReferenceRad));
        Logger.recordOutput(name + "/Velocity Setpoint", inputs.closedLoopReferenceVelocityRadPerSec / (2.0 * Math.PI));
        Logger.recordOutput(name + "/Current Velocity", inputs.velocityRadPerSec / (2.0 * Math.PI));
        Logger.recordOutput(name + "/Applied Voltage", inputs.appliedVolts);
        Logger.recordOutput(name + "/Position Error", Math.toDegrees(inputs.closedLoopErrorRad));
    }

    public Command setTargetPositionCommand(Angle goalAngle) {
        return new InstantCommand(() -> setGoalAngle(goalAngle), this);
    }

    public void setGoalAngle(Angle angle) {
        double clamped = MathUtil.clamp(
            angle.in(Radians),
            minAngleRad,
            maxAngleRad
        );

        goalAngleRad = clamped;
    }

    public ArmIOInputsAutoLogged getInputs() {
        io.updateInputs(inputs);
        return inputs;
    }

    public Angle getGoalAngle() {
        return Radians.of(goalAngleRad);
    }

    public Angle getPositionSetpoint() {
        return Radians.of(inputs.closedLoopReferenceRad);
    }

    public AngularVelocity getVelocitySetpoint() {
        return RadiansPerSecond.of(inputs.closedLoopReferenceVelocityRadPerSec);
    }

    public Angle getPositionError() {
        return Radians.of(inputs.closedLoopErrorRad);
    }

    public Angle getCurrentAngle() {
        return Radians.of(inputs.positionRad);
    }

    public AngularVelocity getVelocity() {
        return RadiansPerSecond.of(inputs.velocityRadPerSec);
    }

    public Voltage getVoltage() {
        return Volts.of(inputs.appliedVolts);
    }

    public void setVoltage(double voltage) {
        if (isEnabled) {
            io.setVoltage(voltage);
        }
    }

    public void setVoltage(Voltage voltage) {
        if (isEnabled) {
            io.setVoltage(voltage);
        }
    }

    public void resetEncoder() {
        io.resetEncoder(resetAngle);
    }

    public void resetEncoder(Angle resetAngle) {
        io.resetEncoder(resetAngle);
    }

    public Command resetEncoderCommand() {
        return new InstantCommand(this::resetEncoder).ignoringDisable(true);
    }

    public Command resetEncoderCommand(Angle resetAngle) {
        return new InstantCommand(() -> resetEncoder(resetAngle)).ignoringDisable(true);
    }

    public SysIdRoutine getSysIdRoutine() {
        return new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                Volts.of(0.5),
                null,
                null
            ),
            new SysIdRoutine.Mechanism(
                this::setVoltage,
                log -> log.motor("motor")
                    .voltage(Volts.of(inputs.appliedVolts))
                    .angularPosition(Radians.of(inputs.positionRad))
                    .angularVelocity(RadiansPerSecond.of(inputs.velocityRadPerSec)),
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

    public boolean atGoal() {
        return Math.abs(Math.toDegrees(inputs.closedLoopErrorRad)) < 5.0;
    }

    private boolean isAtUpperLimit() {
        return inputs.positionRad >= maxAngleRad - 0.05;
    }

    private boolean isAtLowerLimit() {
        return inputs.positionRad <= minAngleRad + 0.05;
    }

    public Command disableControlLoop() {
        return new InstantCommand(() -> controlLoopEnabled = false, this);
    }

    public Command enableControlLoop() {
        return new InstantCommand(() -> controlLoopEnabled = true, this);
    }
}

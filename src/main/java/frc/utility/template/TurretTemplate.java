package frc.utility.template;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
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
import frc.utility.io.TurretIO;
import frc.utility.io.TurretIOInputsAutoLogged;
import frc.utility.template.Constants.TurretConstants;

public class TurretTemplate extends SubsystemBase implements Dashboard, TelemetryUpdater {
    private final TurretIO io;
    private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

    // private final TurretConstants constants;

    private final double minAngleRad;
    private final double maxAngleRad;
    private final Angle resetAngle;

    private final String name;
    private boolean sysIdActive = false;

    private double goalAngleRad = 0.0;

    public TurretTemplate(
            TurretConstants constants,
            TurretIO io
    ) {
        // this.constants = constants;
        this.io = io;

        this.name = constants.name;
        this.minAngleRad = constants.minAngle.in(Radians);
        this.maxAngleRad = constants.maxAngle.in(Radians);
        this.resetAngle = constants.resetAngle;

        TelemetryUtils.registerDashboard(this);
        TelemetryUtils.registerTelemetry(this);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs(name, inputs);

        if (!sysIdActive) {
            io.setPositionRad(goalAngleRad);
        }
    }

    @Override
    public void simulationPeriodic() {}

    @Override
    public void elasticInit() {
        SmartDashboard.putData(name + "/Reset Encoder", resetEncoderCommand());
    }

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

    @Override public void practiceWriters() {}
    @Override public void alerts() {}

    public Command setTargetPositionCommand(Angle goalAngle) {
        return new InstantCommand(() -> setGoalAngle(goalAngle), this);
    }

    public void setGoalAngle(Angle angle) {
        goalAngleRad = clampToLegalTurretAngleRad(angle.in(Radians));
    }

    private double clampToLegalTurretAngleRad(double angle) {
        double angleRad = MathUtil.inputModulus(
            angle,
            0.0,
            2.0 * Math.PI
        );

        double clamped;

        if (angleRad > maxAngleRad) {
            double distToMax = angleRad - maxAngleRad;
            double distToMin = (2.0 * Math.PI) - angleRad + minAngleRad;

            clamped = distToMax <= distToMin
                ? maxAngleRad
                : minAngleRad;
        } else {
            clamped = MathUtil.clamp(angleRad, minAngleRad, maxAngleRad);
        }

        return clamped;
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
        setVoltage(Volts.of(voltage));
    }

    public void setVoltage(Voltage voltage) {
        io.setVoltage(voltage);
    }

    public void resetEncoder() {
        io.resetEncoder(resetAngle);
        setGoalAngle(resetAngle);
    }

    public void resetEncoder(Angle resetAngle) {
        io.resetEncoder(resetAngle);
        setGoalAngle(resetAngle);
    }

    public Command resetEncoderCommand() {
        return new InstantCommand(this::resetEncoder)
            .ignoringDisable(true);
    }

    public Command resetEncoderCommand(Angle resetAngle) {
        return new InstantCommand(() -> resetEncoder(resetAngle))
            .ignoringDisable(true);
    }

    private SysIdRoutine createSysIdRoutine() {
        return new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.6).per(Second),
                Volts.of(3),
                null,
                null
            ),
            new SysIdRoutine.Mechanism(
                voltage -> {
                    double currentAngle = inputs.positionRad;

                    if (currentAngle >= minAngleRad && currentAngle <= maxAngleRad) {
                        setVoltage(voltage);
                    } else {
                        setVoltage(Volts.zero());
                    }
                },
                log -> log.motor("motor")
                    .voltage(getVoltage())
                    .angularPosition(getCurrentAngle())
                    .angularVelocity(getVelocity()),
                this
            )
        );
    }

    public Command getSysIdCommand() {
        SysIdRoutine sysIdRoutine = createSysIdRoutine();

        return new SequentialCommandGroup(
            new InstantCommand(() -> sysIdActive = true, this),

            sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward)
                .until(this::isAtUpperLimit),
            new WaitCommand(0.1),

            sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse)
                .until(this::isAtLowerLimit),
            new WaitCommand(0.1),

            sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward)
                .until(this::isAtUpperLimit),
            new WaitCommand(0.1),

            sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse)
                .until(this::isAtLowerLimit),

            new InstantCommand(() -> sysIdActive = false, this)
        );
    }

    public boolean atGoal() {
        return Math.abs(Math.toDegrees(inputs.closedLoopErrorRad)) < 5.0;
    }

    private boolean isAtUpperLimit() {
        return inputs.positionRad
            >= maxAngleRad - Units.degreesToRadians(20.0);
    }

    private boolean isAtLowerLimit() {
        return inputs.positionRad
            <= minAngleRad + Units.degreesToRadians(20.0);
    }
}

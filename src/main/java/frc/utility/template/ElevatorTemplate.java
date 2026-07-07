package frc.utility.template;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
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
import frc.utility.io.ElevatorIO;
import frc.utility.io.ElevatorIOInputsAutoLogged;
import frc.utility.template.Constants.ElevatorConstants;

public class ElevatorTemplate extends SubsystemBase implements Dashboard, TelemetryUpdater {
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    private final ElevatorConstants constants;
    private final String name;
    private final SysIdRoutine sysIdRoutine;

    private boolean sysIdActive = false;

    private double goalPositionMeters = 0.0;

    public ElevatorTemplate(
            ElevatorConstants constants,
            ElevatorIO io
    ) {
        this.constants = constants;
        this.name = constants.name;
        this.io = io;

        TelemetryUtils.registerDashboard(this);
        TelemetryUtils.registerTelemetry(this);

        sysIdRoutine = createSysIdRoutine();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs(name, inputs);

        if (!sysIdActive) {
            io.setPositionMeters(goalPositionMeters);
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
    public void updateTelemetry() {
        Logger.recordOutput(name + "/Goal Position", goalPositionMeters);
        Logger.recordOutput(name + "/Current Position", inputs.positionMeters);
        Logger.recordOutput(name + "/Position Setpoint", inputs.closedLoopReferenceMeters);
        Logger.recordOutput(name + "/Velocity Setpoint", inputs.closedLoopReferenceVelocityMetersPerSec);
        Logger.recordOutput(name + "/Current Velocity", inputs.velocityMetersPerSec);
        Logger.recordOutput(name + "/Applied Voltage", inputs.appliedVolts);
        Logger.recordOutput(name + "/Position Error", inputs.closedLoopErrorMeters);
    }

    @Override public void practiceWriters() {}
    @Override public void alerts() {}

    public Command setTargetPositionCommand(Distance value) {
        return new InstantCommand(() -> setTargetPosition(value), this);
    }

    public void setTargetPosition(Distance position) {
        double clampedMeters = MathUtil.clamp(
            position.in(Meters),
            constants.minDistance.in(Meters),
            constants.maxDistance.in(Meters)
        );

        goalPositionMeters = clampedMeters;
    }

    public Distance getGoalPosition() {
        return Meters.of(goalPositionMeters);
    }

    public Distance getPosition() {
        return Meters.of(inputs.positionMeters);
    }

    public LinearVelocity getVelocity() {
        return MetersPerSecond.of(inputs.velocityMetersPerSec);
    }

    public Voltage getVoltage() {
        return Volts.of(inputs.appliedVolts);
    }

    public Distance getPositionSetpoint() {
        return Meters.of(inputs.closedLoopReferenceMeters);
    }

    public LinearVelocity getVelocitySetpoint() {
        return MetersPerSecond.of(inputs.closedLoopReferenceVelocityMetersPerSec);
    }

    public Distance getPositionError() {
        return Meters.of(inputs.closedLoopErrorMeters);
    }

    public void setVoltage(Voltage voltage) {
        io.setVoltage(voltage);
    }

    public void setVoltage(double voltage) {
        io.setVoltage(Volts.of(voltage));
    }

    public void resetEncoder() {
        io.resetEncoder(Meters.zero());
        setTargetPosition(Meters.zero());
    }

    public Command resetEncoderCommand() {
        return new InstantCommand(this::resetEncoder)
            .ignoringDisable(true);
    }

    private SysIdRoutine createSysIdRoutine() {
        return new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                this::setVoltage,
                log -> log.motor("motor")
                    .voltage(getVoltage())
                    .linearPosition(getPosition())
                    .linearVelocity(getVelocity()),
                this
            )
        );
    }

    public Command getSysIdCommand() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> sysIdActive = true, this),
            sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward),
            new WaitCommand(0.1),
            sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse),
            new WaitCommand(0.1),
            sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward),
            new WaitCommand(0.1),
            sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse),
            new InstantCommand(() -> sysIdActive = false, this)
        );
    }

    public boolean atGoal() {
        return Math.abs(inputs.closedLoopErrorMeters) < 0.03;
    }
}

package frc.utility.template;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.utility.TelemetryUtils;
import frc.utility.TelemetryUtils.Dashboard;
import frc.utility.TelemetryUtils.TelemetryUpdater;
import frc.utility.io.FlywheelIO;
import frc.utility.io.FlywheelIOInputsAutoLogged;
import frc.utility.template.Constants.FlywheelConstants;

public class FlywheelTemplate extends SubsystemBase implements Dashboard, TelemetryUpdater {
    private final FlywheelIO io;
    private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

    private final double maxSpeedRotationsPerSecond;
    private final double minSpeedRotationsPerSecond;
    private final String name;

    private final SysIdRoutine sysIdRoutine;
    private boolean sysIdActive = false;

    private double targetVelocityRotationsPerSecond = 0.0;

    public FlywheelTemplate(
            FlywheelConstants constants,
            FlywheelIO io
    ) {
        this.maxSpeedRotationsPerSecond = constants.maxVelocity.in(RotationsPerSecond);
        this.minSpeedRotationsPerSecond = constants.minVelocity.in(RotationsPerSecond);
        this.name = constants.name;
        this.io = io;

        TelemetryUtils.registerTelemetry(this);

        sysIdRoutine = createSysIdRoutine();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs(name, inputs);

        if (!sysIdActive) {
            io.setVelocityRotationsPerSecond(targetVelocityRotationsPerSecond);
        }
    }

    @Override
    public void elasticInit() {}

    @Override
    public void updateTelemetry() {
        Logger.recordOutput(name + "/Target Velocity", targetVelocityRotationsPerSecond);
        Logger.recordOutput(name + "/Current Velocity", inputs.velocityRotationsPerSecond);
        Logger.recordOutput(name + "/Applied Voltage", inputs.appliedVolts);
        Logger.recordOutput(name + "/Torque Current", inputs.torqueCurrentAmps);
        Logger.recordOutput(name + "/Velocity Error", inputs.closedLoopErrorRotationsPerSecond);
    }

    @Override
    public void practiceWriters() {}

    @Override
    public void alerts() {}

    public Command setTargetVelocityCommand(AngularVelocity target) {
        return new InstantCommand(() -> setTargetVelocity(target), this);
    }

    public void setTargetVelocity(AngularVelocity target) {
        double clampedRps = MathUtil.clamp(
            target.in(RotationsPerSecond),
            minSpeedRotationsPerSecond,
            maxSpeedRotationsPerSecond
        );

        targetVelocityRotationsPerSecond = clampedRps;
    }

    public AngularVelocity getTargetVelocity() {
        return RotationsPerSecond.of(targetVelocityRotationsPerSecond);
    }

    public double getTargetVelocityRotationsPerSecond() {
        return targetVelocityRotationsPerSecond;
    }

    public AngularVelocity getVelocityError() {
        return RotationsPerSecond.of(inputs.closedLoopErrorRotationsPerSecond);
    }

    public Angle getCurrentAngle() {
        return Rotations.of(inputs.positionRotations);
    }

    public AngularVelocity getVelocity() {
        return RotationsPerSecond.of(inputs.velocityRotationsPerSecond);
    }

    public Voltage getVoltage() {
        return Volts.of(inputs.appliedVolts);
    }

    public Current getCurrent() {
        return Amps.of(inputs.statorCurrentAmps);
    }

    public void setVoltage(Voltage voltage) {
        io.setVoltage(voltage);
    }

    public void setVoltage(double voltage) {
        setVoltage(Volts.of(voltage));
    }

    private SysIdRoutine createSysIdRoutine() {
        return new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                Volts.of(9),
                Seconds.of(10),
                null
            ),
            new SysIdRoutine.Mechanism(
                this::setVoltage,
                log -> log.motor("motor")
                    .voltage(getVoltage())
                    .angularPosition(getCurrentAngle())
                    .angularVelocity(getVelocity()),
                this
            )
        );
    }

    public Command getSysIdCommand() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> sysIdActive = true, this),
            sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward),
            new WaitCommand(10),
            sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse),
            new WaitCommand(10),
            sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward),
            new WaitCommand(10),
            sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse),
            new WaitCommand(10),
            new InstantCommand(() -> sysIdActive = false, this)
        );
    }

    public boolean atSetpoint() {
        return Math.abs(inputs.closedLoopErrorRotationsPerSecond) < 5.0;
    }
}

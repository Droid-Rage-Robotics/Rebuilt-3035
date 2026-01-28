package frc.robot.SysID;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.utility.motor.TalonEx;

public class SysID {
    /**
     * Determines what measurements will be used in SysId calculations
     */
    public enum Measurement {
        /**
         * The SysId measurements will be in Rotations/RotationsPerSecond
         */
        ANGLE,
        /**
         * The SysId measurements will be in Inches/InchesPerSecond
         */
        DISTANCE
    }

    // Mutable holders for unit-safe values (shared across all modules)
    private final MutVoltage appliedVoltage = Volts.mutable(0);
    private final MutAngle angle = Radians.mutable(0);
    private final MutDistance distance = Inches.mutable(0);
    private final MutAngularVelocity angularVelocity = RadiansPerSecond.mutable(0);
    private final MutLinearVelocity linearVelocity = InchesPerSecond.mutable(0);
    
    private SysIdRoutine routine;

    /**
     * Constructs an instance of the {@link SysID} class with a routine
     * for one motor. Download logs from FileZilla.
     * 
     * @param motor the motors to be tested
     * @param subsystem the subsystem of the motors
     * @param unit sets the unit to be used: {@code Measurement.ANGLE} or {@code Measurement.DISTANCE}
     */
    public SysID(TalonEx motor, Subsystem subsystem, Measurement unit) {
        switch(unit){
        case ANGLE:
            routine = new SysIdRoutine(new SysIdRoutine.Config(), new SysIdRoutine.Mechanism(motor::setVoltage, log -> {
            log.motor("motor")
                .voltage(appliedVoltage.mut_replace(motor.getVoltage(), Volts))
                .angularPosition(angle.mut_replace(motor.getPosition(), Rotations))
                .angularVelocity(angularVelocity.mut_replace(motor.getVelocity(), RotationsPerSecond));
            }, subsystem)
        );
        case DISTANCE:
            routine = new SysIdRoutine(new SysIdRoutine.Config(), new SysIdRoutine.Mechanism(motor::setVoltage, log -> {
            log.motor("motor")
                .voltage(appliedVoltage.mut_replace(motor.getVoltage(), Volts))
                .linearPosition(distance.mut_replace(motor.getPosition(), Inches))
                .linearVelocity(linearVelocity.mut_replace(motor.getVelocity(), InchesPerSecond));
            }, subsystem)
        );
        }
    }

    /**
     * Constructs an instance of the {@link SysID} class with a routine
     * for CTRE motors ONLY. It uses {@link SignalLogger} to log data
     * instead of {@link DataLogManager}. Download logs from Phoenix
     * Tuner instead of FileZilla.
     * 
     * @param motor the TalonEx to run SysId on
     * @param subsystem the subsystem of the motor
     */
    public SysID(TalonEx motor, Subsystem subsystem) {
        routine = new SysIdRoutine(
            new SysIdRoutine.Config(
                null, // Use default ramp rate (1 V/s)
                Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
                null, // Use default timeout (10 s)
                (state) -> SignalLogger.writeString("state", state.toString()) // Log state with Phoenix SignalLogger class
            ),
            new SysIdRoutine.Mechanism(motor::setVoltage, null, subsystem)
        );
    }

    /**
     * Constructs an instance of the {@link SysID} class with a routine
     * for two motors. Download logs from FileZilla.
     * 
     * @param motor the motors to be tested
     * @param subsystem the subsystem of the motors
     * @param unit sets the unit to be used: {@code Measurement.ANGLE} or {@code Measurement.DISTANCE}
     */
    public SysID(TalonEx[] motor, Subsystem subsystem, Measurement unit) {
        switch(unit){
        case ANGLE:
            routine = new SysIdRoutine(new SysIdRoutine.Config(), new SysIdRoutine.Mechanism((motor[0]::setVoltage), log -> {
            log.motor("motor0")
                .voltage(appliedVoltage.mut_replace(motor[0].getVoltage() * RobotController.getBatteryVoltage(), Volts))
                .angularPosition(angle.mut_replace(motor[0].getPosition(), Rotations))
                .angularVelocity(angularVelocity.mut_replace(motor[0].getVelocity(), RotationsPerSecond));
            log.motor("motor1")
                .voltage(appliedVoltage.mut_replace(motor[1].getVoltage() * RobotController.getBatteryVoltage(), Volts))
                .angularPosition(angle.mut_replace(motor[1].getPosition(), Rotations))
                .angularVelocity(angularVelocity.mut_replace(motor[1].getVelocity(), RotationsPerSecond));
            }, subsystem)
        );
        case DISTANCE:
            routine = new SysIdRoutine(new SysIdRoutine.Config(), new SysIdRoutine.Mechanism(motor[0]::setVoltage, log -> {
            log.motor("shooter-wheel")
                .voltage(appliedVoltage.mut_replace(motor[0].getVoltage() * RobotController.getBatteryVoltage(), Volts))
                .linearPosition(distance.mut_replace(motor[0].getPosition(), Inches))
                .linearVelocity(linearVelocity.mut_replace(motor[0].getVelocity(), InchesPerSecond));
            log.motor("motor1")
                .voltage(appliedVoltage.mut_replace(motor[1].getVoltage() * RobotController.getBatteryVoltage(), Volts))
                .linearPosition(distance.mut_replace(motor[1].getPosition(), Inches))
                .linearVelocity(linearVelocity.mut_replace(motor[1].getVelocity(), InchesPerSecond));
            }, subsystem)
        );
        }
    }

    /**
     * Returns a command to run a quasistatic test in the specified direction.
     *
     * <p>The command will call the `drive` and `log` callbacks supplied at routine construction once
     * per iteration. Upon command end or interruption, the `drive` callback is called with a value of
     * 0 volts.
     *
     * @param direction The direction in which to run the test.
     * @return A command to run the test.
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return routine.quasistatic(direction);
    }

    /**
     * Returns a command to run a dynamic test in the specified direction.
     *
     * <p>The command will call the `drive` and `log` callbacks supplied at routine construction once
     * per iteration. Upon command end or interruption, the `drive` callback is called with a value of
     * 0 volts.
     *
     * @param direction The direction in which to run the test.
     * @return A command to run the test.
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return routine.dynamic(direction);
    }
}

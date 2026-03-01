package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.DroidRageConstants;
import frc.utility.motor.MotorConstants;
import frc.utility.motor.MotorConstants.Direction;
import frc.utility.template.FlywheelTemplate;
import frc.utility.template.SubsystemConstants;
import frc.utility.template.SubsystemConstants.EncoderType;

public class IntakeWheel extends FlywheelTemplate {
    private static final SubsystemConstants constants = new SubsystemConstants()
        .withConversionFactor(1)
        .withEncoderType(EncoderType.INTEGRATED)
        .withMinVelocity(RotationsPerSecond.of(-100))
        .withMaxVelocity(RotationsPerSecond.of(100))
        .withName("Intake")
        .withOffset(0)
        .withMainNum(0);

    private static final MotorConstants motorConstants = new MotorConstants() 
        .withDeviceId(15)
        .withCANBus(DroidRageConstants.driveCanBus)
        .withDirection(Direction.Forward)
        .withIdleMode(NeutralModeValue.Brake)
        .withConversionFactor( 1)
        .withMotorType(MotorType.KrakenX60)
        .withStatorCurrentLimit(25)
        .withSupplyCurrentLimit(25);

    public IntakeWheel(boolean isEnabled) {
        super(isEnabled,
            new PIDController(0.015741, 0, 0), 
            new SimpleMotorFeedforward(0.50353, 0.12171, 0.0036507), 
            constants, 
            motorConstants);
    }
}

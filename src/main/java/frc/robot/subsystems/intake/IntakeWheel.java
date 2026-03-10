package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.DroidRageConstants;
import frc.robot.subsystems.intake.Intake.IntakeValue;
import frc.utility.devices.motor.MotorConstants;
import frc.utility.devices.motor.MotorConstants.Direction;
import frc.utility.template.FlywheelTemplate;
import frc.utility.template.SubsystemConstants;
import frc.utility.template.SubsystemConstants.EncoderType;

public class IntakeWheel extends FlywheelTemplate {
    private static final SubsystemConstants constants = new SubsystemConstants()
        .withConversionFactor(1)
        .withPID(0.015741, 0, 0)
        .withFeedforward(0.50353, 0.12171, 0.0036507)
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
        .withIdleMode(NeutralModeValue.Coast)
        .withConversionFactor( 1)
        .withMotorType(MotorType.KrakenX60);
        // .withStatorCurrentLimit(65)//50
        // .withSupplyCurrentLimit(65);//50

    public IntakeWheel(boolean isEnabled) {
        super(isEnabled, constants, motorConstants);
    }

    public Command setTargetVelocityCommand(IntakeValue.WheelVelocity target) {
        return super.setTargetVelocityCommand(target.getVelocity());
    }
}

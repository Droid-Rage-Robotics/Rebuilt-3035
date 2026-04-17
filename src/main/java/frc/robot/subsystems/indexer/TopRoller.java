package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.DroidRageConstants;
import frc.utility.devices.motor.MotorConstants;
import frc.utility.devices.motor.MotorConstants.Direction;
import frc.utility.template.FlywheelTemplate;
import frc.utility.template.SubsystemConstants;
import frc.utility.template.SubsystemConstants.EncoderType;

public class TopRoller extends FlywheelTemplate {
    private static final SubsystemConstants constants = new SubsystemConstants()
        .withGearRatio(1.0)
        .withPID(0.013449, 0, 0)
        .withFeedforward(0.2067, 0.11912, 0.0032711)
        .withEncoderType(EncoderType.INTEGRATED)
        .withMinVelocity(RotationsPerSecond.of(-150))
        .withMaxVelocity(RotationsPerSecond.of(150))
        .withName("Top Roller")
        .withOffset(0)
        .withMainNum(0);
    
    private static final MotorConstants rightMotor = new MotorConstants()
        .withDeviceId(17)
        .withCANBus(DroidRageConstants.driveCanBus)
        .withDirection(Direction.Reversed)
        .withIdleMode(NeutralModeValue.Coast)
        .withSupplyCurrentLimit(80)
        .withStatorCurrentLimit(70); 

    public TopRoller(boolean isEnabled) {
        super(isEnabled, constants, rightMotor);
    }
}

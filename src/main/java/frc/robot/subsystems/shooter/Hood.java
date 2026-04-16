package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.DroidRageConstants;
import frc.utility.devices.motor.MotorConstants;
import frc.utility.devices.motor.MotorConstants.Direction;
import frc.utility.template.ArmTemplate;
import frc.utility.template.SubsystemConstants;
import frc.utility.template.SubsystemConstants.EncoderType;

public class Hood extends ArmTemplate {
    private static final SubsystemConstants constants = new SubsystemConstants()
        .withPID(20, 0, 20)
        .withFeedforward(0.4334, 0, 0.28114, 2.0731)
        .withMaxVelocity(RotationsPerSecond.of(1))
        .withMaxAcceleration(RotationsPerSecondPerSecond.of(0.5))
        .withGearRatio(4.0 * (155.0/9.0))
        .withEncoderType(EncoderType.INTEGRATED)
        .withMinAngle(Degrees.zero())
        .withMaxAngle(Degrees.of(28))//28
        .withName("Hood")
        .withOffset(0)
        .withMainNum(0);

    private static final MotorConstants motorConstants = new MotorConstants() 
        .withDeviceId(19)
        .withCANBus(DroidRageConstants.rioCanBus)
        .withDirection(Direction.Reversed)
        .withIdleMode(NeutralModeValue.Brake)
        .withConversionFactor( 1)
        .withSupplyCurrentLimit(45)
        .withStatorCurrentLimit(50);

    public Hood(boolean isEnabled) {
        super(isEnabled, constants, null, motorConstants);
    }

    @Override
    public void periodic() {
        super.periodic();

        if (getCurrentAngle().in(Degrees)<0) {
            resetEncoder(0);
        }
    }
}
package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.DroidRageConstants;
import frc.utility.DRAreaManager;
import frc.utility.devices.motor.MotorConstants;
import frc.utility.devices.motor.MotorConstants.Direction;
import frc.utility.template.ArmTemplate;
import frc.utility.template.SubsystemConstants;
import frc.utility.template.SubsystemConstants.EncoderType;

public class Hood extends ArmTemplate {
    private static final SubsystemConstants constants = new SubsystemConstants()
        .withPID(20, 0, 20)
        .withFeedforward(0.4334, 0, 0.28114, 2.2) // kA 2.0731
        .withMaxVelocity(RotationsPerSecond.of(1))
        .withMaxAcceleration(RotationsPerSecondPerSecond.of(2))
        .withGearRatio(4.0 * (155.0/9.0))
        .withEncoderType(EncoderType.INTEGRATED)
        .withMinAngle(Degrees.zero())
        .withMaxAngle(Degrees.of(28))
        .withName("Hood")
        .withResetAngle(Degrees.zero());

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

        DRAreaManager.inBetween().whileTrue(setTargetPositionCommand(Degrees.zero()));
    }

    @Override
    public void periodic() {
        super.periodic();

        if (getCurrentAngle().in(Degrees)<0) {
            resetEncoder(Rotation.zero());
        }
    }
}
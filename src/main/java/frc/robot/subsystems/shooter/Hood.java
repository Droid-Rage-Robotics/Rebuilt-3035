package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.DroidRageConstants;
import frc.utility.DRAreaManager;
import frc.utility.devices.motor.MotorConstants;
import frc.utility.io.ArmIOTalonFX;
import frc.utility.template.ArmTemplate;
import frc.utility.template.Constants.ArmConstants;
import frc.utility.template.Constants.EncoderType;

public class Hood extends ArmTemplate {
    private static final ArmConstants constants = new ArmConstants()
        .withPID(20, 0, 20)
        .withFeedforward(0.4334, 0, 0.28114, 2.2) // kA 2.0731
        .withMotionMagic(RotationsPerSecond.of(1), RotationsPerSecondPerSecond.of(2), 0)
        .withGearRatio(4.0 * (155.0/9.0))
        .withEncoderType(EncoderType.INTEGRATED)
        .withLimits(Degrees.zero(), Degrees.of(28))
        .withName("Hood")
        .withResetAngle(Degrees.zero());

    private static final MotorConstants motorConstants = new MotorConstants() 
        .withDeviceId(19)
        .withCANBus(DroidRageConstants.rioCanBus)
        .withDirection(InvertedValue.CounterClockwise_Positive)
        .withIdleMode(NeutralModeValue.Brake)
        .withConversionFactor( 1)
        .withSupplyCurrentLimit(45)
        .withStatorCurrentLimit(50);

    public Hood(boolean isEnabled) {
        super(isEnabled, constants, new ArmIOTalonFX(isEnabled, constants, null, motorConstants));

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
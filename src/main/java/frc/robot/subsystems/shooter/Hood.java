package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.DroidRageConstants;
import frc.utility.motor.MotorConstants;
import frc.utility.motor.MotorConstants.Direction;
import frc.utility.template.ArmTemplate;
import frc.utility.template.SubsystemConstants;
import frc.utility.template.SubsystemConstants.EncoderType;

public class Hood extends ArmTemplate {
    private static final SubsystemConstants constants = new SubsystemConstants()
        .withConversionFactor(1.0/(4.0 * (155.0/9.0)))
        .withEncoderType(EncoderType.INTEGRATED)
        .withMinAngle(Degrees.zero())
        .withMaxAngle(Degrees.of(28))
        .withName("Hood")
        .withOffset(0)
        .withMainNum(0);

    private static final MotorConstants motorConstants = new MotorConstants() 
        .withDeviceId(19)
        .withCANBus(DroidRageConstants.rioCanBus)
        .withDirection(Direction.Reversed)
        .withIdleMode(NeutralModeValue.Brake)
        .withConversionFactor( 1)
        .withSupplyCurrentLimit(70)
        .withStatorCurrentLimit(70);

    public Hood(boolean isEnabled) {
        super(isEnabled, 
            new ProfiledPIDController(15, 0, 0,
            new TrapezoidProfile.Constraints(0.5, 0.5)), 
            new ArmFeedforward(0.4334, 0,0.28114, 2.0731), 
            constants, 
            null, 
            motorConstants);
    }

    @Override
    public void periodic() {
        super.periodic();

        if (getCurrentAngle().getDegrees()<0) {
            resetEncoder(Degrees.of(0));
        }
    }
}

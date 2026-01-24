package frc.robot.subsystems.shooter;

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
        .withConversionFactor(1)
        .withEncoderType(EncoderType.ABSOLUTE)
        .withLowerLimit(0)
        .withUpperLimit(0)
        .withName("Hood")
        .withOffset(0)
        .withMainNum(0);

    private static final MotorConstants motorConstants = new MotorConstants() 
        .withCANBus(DroidRageConstants.rioCanBus)
        .withDirection(Direction.Forward)
        .withIdleMode(NeutralModeValue.Brake)
        .withConversionFactor( 1)
        .withSupplyCurrentLimit(70)
        .withStatorCurrentLimit(70);

    public Hood(boolean isEnabled) {
        super(isEnabled, 
            new ProfiledPIDController(0, 0, 0,
            new TrapezoidProfile.Constraints(0, 0)), 
            new ArmFeedforward(0, 0, 0), 
            constants, 
            null, 
            motorConstants);
    }
}

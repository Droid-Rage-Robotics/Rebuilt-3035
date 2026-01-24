package frc.robot.subsystems;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.DroidRageConstants;
import frc.utility.motor.MotorConstants;
import frc.utility.motor.MotorConstants.Direction;
import frc.utility.template.FlywheelTemplate;
import frc.utility.template.SubsystemConstants;
import frc.utility.template.SubsystemConstants.EncoderType;

public class Indexer extends FlywheelTemplate{
    private static final SubsystemConstants constants = new SubsystemConstants()
        .withConversionFactor(1)
        .withEncoderType(EncoderType.INTEGRATED)
        .withLowerLimit(0)
        .withUpperLimit(0)
        .withName("Indexer")
        .withOffset(0)
        .withMainNum(0);
    
    private static final MotorConstants motorConstants = new MotorConstants() 
        .withCANBus(DroidRageConstants.rioCanBus)
        .withDirection(Direction.Forward)
        .withIdleMode(NeutralModeValue.Brake)
        .withConversionFactor( 1)
        .withSupplyCurrentLimit(70)
        .withStatorCurrentLimit(70);

    public Indexer(boolean isEnabled) {
        super(isEnabled,
            new PIDController(0, 0, 0), 
            new SimpleMotorFeedforward(0, 0, 0), 
            constants, 
            motorConstants);
    }
}

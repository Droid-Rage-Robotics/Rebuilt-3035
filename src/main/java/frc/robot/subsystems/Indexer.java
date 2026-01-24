package frc.robot.subsystems;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.DroidRageConstants;
import frc.utility.motor.MotorConstants;
import frc.utility.motor.MotorConstants.Direction;
import frc.utility.template.FlywheelTemplate;

public class Indexer extends FlywheelTemplate{
    private static final MotorConstants motorConstants = new MotorConstants() 
        .withCANBus(DroidRageConstants.rioCanBus)
        .withDirection(Direction.Forward)
        .withIdleMode(NeutralModeValue.Brake)
        .withConversionFactor( 1)
        .withSupplyCurrentLimit(70)
        .withStatorCurrentLimit(70);

    public Indexer(boolean isEnabled) {
        super(
            0,
            new PIDController(0, 0, 0), 
            new SimpleMotorFeedforward(0, 0, 0), 
            0, 
            0, 
            1, 
            "Indexer", 
            isEnabled,
            motorConstants);
    }
}

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.DroidRageConstants;
import frc.robot.subsystems.vision.Vision;
import frc.utility.motor.MotorConstants;
import frc.utility.motor.MotorConstants.Direction;
import frc.utility.template.TurretTemplate;

public class Turret extends TurretTemplate {    
    private static final MotorConstants motorConstants = new MotorConstants() 
        .withCANBus(DroidRageConstants.rioCanBus)
        .withDirection(Direction.Forward)
        .withIdleMode(NeutralModeValue.Brake)
        .withConversionFactor( 1)
        .withSupplyCurrentLimit(70)
        .withStatorCurrentLimit(70);
    
    public Turret(boolean isEnabled, Vision vision) {
        super(0, vision.getLeftLimelight(), 
            new ProfiledPIDController(0, 0, 0, 
            new TrapezoidProfile.Constraints(0, 0)), 
            new SimpleMotorFeedforward(0, 0, 0), 
            0, 
            0, 
            1, 
            0, 
            isEnabled, motorConstants);
    }
}
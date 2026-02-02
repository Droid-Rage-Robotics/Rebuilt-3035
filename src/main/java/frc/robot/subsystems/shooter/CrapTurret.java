package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.DroidRageConstants;
import frc.utility.motor.MotorConstants;
import frc.utility.motor.MotorConstants.Direction;
import frc.utility.template.SubsystemConstants;
import frc.utility.template.SubsystemConstants.EncoderType;
import frc.utility.template.TurretTemplate;

public class CrapTurret extends TurretTemplate {
    private static final SubsystemConstants constants = new SubsystemConstants()
        .withConversionFactor(1)
        .withEncoderType(EncoderType.INTEGRATED)
        .withLowerLimit(-90) //Degree
        .withUpperLimit(90) //Degree
        .withName("Turret")
        .withOffset(0)
        .withMainNum(0);
    
    private static final MotorConstants motorConstants = new MotorConstants() 
        .withDeviceId(14)
        .withCANBus(DroidRageConstants.rioCanBus)
        .withDirection(Direction.Forward)
        .withIdleMode(NeutralModeValue.Brake)
        .withSupplyCurrentLimit(70)
        .withStatorCurrentLimit(70);
    
    public CrapTurret(boolean isEnabled) {
        super(isEnabled, 
            new ProfiledPIDController(1, 0, 0, 
            new TrapezoidProfile.Constraints(1, 1)), 
            new SimpleMotorFeedforward(0.01, 0.014, 0), 
            constants, 
            null,            
            motorConstants);
    }
}

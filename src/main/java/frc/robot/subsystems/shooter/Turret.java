package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.DroidRageConstants;
import frc.robot.subsystems.vision.Vision;
import frc.utility.encoder.EncoderConstants;
import frc.utility.motor.MotorConstants;
import frc.utility.motor.MotorConstants.Direction;
import frc.utility.template.SubsystemConstants;
import frc.utility.template.SubsystemConstants.EncoderType;
import frc.utility.template.TurretTemplate;

public class Turret extends TurretTemplate {    
    private static final SubsystemConstants constants = new SubsystemConstants()
        .withConversionFactor(1)
        .withEncoderType(EncoderType.ABSOLUTE)
        .withLowerLimit(0) //Degree
        .withUpperLimit(0) //Degree
        .withName("Turret")
        .withOffset(0)
        .withMainNum(0);
    
    private static final MotorConstants motorConstants = new MotorConstants() 
        .withCANBus(DroidRageConstants.rioCanBus)
        .withDirection(Direction.Forward)
        .withIdleMode(NeutralModeValue.Brake)
        .withConversionFactor( 1)
        .withSupplyCurrentLimit(70)
        .withStatorCurrentLimit(70);
    
    private static final EncoderConstants encoderConstants = new EncoderConstants()
        .withDeviceId(0)    
        .withCANBus(DroidRageConstants.rioCanBus)
        .withConversionFactor(1)
        .withDirection(SensorDirectionValue.Clockwise_Positive);
    
    public Turret(boolean isEnabled) {
        super(isEnabled, 
            new ProfiledPIDController(0, 0, 0, 
            new TrapezoidProfile.Constraints(0, 0)), 
            new SimpleMotorFeedforward(0, 0, 0), 
            constants, 
            encoderConstants,            
            motorConstants);
    }
}
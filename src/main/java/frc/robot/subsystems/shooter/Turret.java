package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.DroidRageConstants;
import frc.utility.encoder.EncoderConstants;
import frc.utility.motor.MotorConstants;
import frc.utility.motor.MotorConstants.Direction;
import frc.utility.template.SubsystemConstants;
import frc.utility.template.SubsystemConstants.EncoderType;
import frc.utility.template.TurretTemplate;

public class Turret extends TurretTemplate {
    private static final SubsystemConstants constants = new SubsystemConstants()
        .withConversionFactor(3.0/50.0)
        .withEncoderType(EncoderType.INTEGRATED)
        .withMinAngle(Degrees.of(-135))
        .withMaxAngle(Degrees.of(135))
        .withName("Turret")
        .withOffset(0)
        .withMainNum(0);
    
    private static final MotorConstants motorConstants = new MotorConstants() 
        .withDeviceId(18)
        .withCANBus(DroidRageConstants.rioCanBus)
        .withDirection(Direction.Forward)
        .withIdleMode(NeutralModeValue.Brake)
        .withSupplyCurrentLimit(70)
        .withStatorCurrentLimit(70);
    
    private static final EncoderConstants encoderConstants = new EncoderConstants()
        .withDeviceId(3)    
        .withCANBus(DroidRageConstants.rioCanBus)
        .withDirection(SensorDirectionValue.Clockwise_Positive);
    
    public Turret(boolean isEnabled) {
        super(isEnabled, 
            new ProfiledPIDController(10, 0, 0, 
            new TrapezoidProfile.Constraints(1, 1)), 
            new SimpleMotorFeedforward(0.11055, 1.6667, 0.15809), 
            constants, 
            encoderConstants,            
            motorConstants);
    }
}
package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.DroidRageConstants;
import frc.utility.motor.MotorConstants;
import frc.utility.motor.MotorConstants.Direction;
import frc.utility.template.FlywheelTemplate;
import frc.utility.template.SubsystemConstants;
import frc.utility.template.SubsystemConstants.EncoderType;

public class ShooterWheel extends FlywheelTemplate {
    private static final SubsystemConstants constants = new SubsystemConstants()
        .withConversionFactor(1)
        .withEncoderType(EncoderType.INTEGRATED)
        .withLowerLimit(-60)
        .withUpperLimit(60)
        .withName("ShooterWheel")
        .withOffset(0)
        .withMainNum(0);
    
    private static final MotorConstants rightMotor = new MotorConstants() 
        .withDeviceId(4)
        .withCANBus(DroidRageConstants.rioCanBus)
        .withDirection(Direction.Forward)
        .withIdleMode(NeutralModeValue.Brake)
        .withConversionFactor( 1);

    private static final MotorConstants leftMotor = new MotorConstants() 
        .withDeviceId(5)
        .withCANBus(DroidRageConstants.rioCanBus)
        .withDirection(Direction.Reversed)
        .withIdleMode(NeutralModeValue.Brake)
        .withConversionFactor( 1);

    public ShooterWheel(boolean isEnabled) {
        super(isEnabled,
            new PIDController(0.035014, 0, 0), 
            new SimpleMotorFeedforward(0.083079, 0.11738, 0.0072731), 
            constants, 
            rightMotor, leftMotor);
    }
}

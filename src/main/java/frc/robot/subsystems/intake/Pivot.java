package frc.robot.subsystems.intake;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.DroidRageConstants;
import frc.utility.encoder.EncoderConstants;
import frc.utility.motor.MotorConstants;
import frc.utility.motor.MotorConstants.Direction;
import frc.utility.template.ArmTemplate;
import frc.utility.template.SubsystemConstants;
import frc.utility.template.SubsystemConstants.EncoderType;

public class Pivot extends ArmTemplate {
    private static final SubsystemConstants constants = new SubsystemConstants()
        .withConversionFactor(1)
        .withEncoderType(EncoderType.ABSOLUTE)
        .withLowerLimit(0)
        .withUpperLimit(0)
        .withName("Pivot")
        .withOffset(0)
        .withMainNum(0);

    private static final MotorConstants motorConstants = new MotorConstants()
        .withDeviceId(0)
        .withCANBus(DroidRageConstants.rioCanBus)
        .withConversionFactor(1)
        .withDirection(Direction.Forward)
        .withIdleMode(NeutralModeValue.Brake)
        .withStatorCurrentLimit(70)
        .withSupplyCurrentLimit(70);
    
    private static final EncoderConstants encoderConstants = new EncoderConstants()
        .withDeviceId(0)    
        .withCANBus(DroidRageConstants.rioCanBus)
        .withConversionFactor(1)
        .withDirection(SensorDirectionValue.Clockwise_Positive);

    public Pivot(boolean isEnabled) {
        super(isEnabled, 
            new ProfiledPIDController(0, 0, 0,
            new TrapezoidProfile.Constraints(0, 0)), 
            new ArmFeedforward(0, 0, 0), 
            constants, 
            encoderConstants, 
            motorConstants);
    }
}
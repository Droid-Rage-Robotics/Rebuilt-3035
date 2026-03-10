package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.DroidRageConstants;
import frc.utility.devices.motor.MotorConstants;
import frc.utility.devices.motor.MotorConstants.Direction;
import frc.utility.template.FlywheelTemplate;
import frc.utility.template.SubsystemConstants;
import frc.utility.template.SubsystemConstants.EncoderType;

public class ShooterWheel extends FlywheelTemplate {
    private static final SubsystemConstants constants = new SubsystemConstants()
        .withPID(0.043, 0, 0) //0.035014
        .withFeedforward(0.085, 0.11738, 0.0072731) //ks: 0.083079
        .withConversionFactor(1)
        .withEncoderType(EncoderType.INTEGRATED)
        .withMinVelocity(RotationsPerSecond.of(-60))
        .withMaxVelocity(RotationsPerSecond.of(75))
        .withName("ShooterWheel")
        .withOffset(0)
        .withMainNum(0);
    
    private static final MotorConstants rightMotor = new MotorConstants() // LEADER
        .withDeviceId(20)
        .withCANBus(DroidRageConstants.rioCanBus)
        .withDirection(Direction.Reversed)
        .withIdleMode(NeutralModeValue.Coast)
        .withConversionFactor( 1)
        .withSupplyCurrentLimit(80)
        .withStatorCurrentLimit(80);

    private static final MotorConstants leftMotor = new MotorConstants() // FOLLOWER
        .withDeviceId(21)
        .withCANBus(DroidRageConstants.rioCanBus)
        // .withDirection(Direction.Forward)
        .withMotorAlignment(MotorAlignmentValue.Opposed)
        .withIdleMode(NeutralModeValue.Coast)
        .withConversionFactor( 1)
        .withSupplyCurrentLimit(80)
        .withStatorCurrentLimit(80);

    public ShooterWheel(boolean isEnabled) {
        super(isEnabled, constants, rightMotor, leftMotor);
    }
}

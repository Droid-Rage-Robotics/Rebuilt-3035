package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.DroidRageConstants;
import frc.utility.devices.motor.MotorConstants;
import frc.utility.io.FlywheelIOTalonFX;
import frc.utility.template.FlywheelTemplate;
import frc.utility.template.Constants.FlywheelConstants;

public class ShooterWheel extends FlywheelTemplate {
    private static final FlywheelConstants constants = new FlywheelConstants()
        .withPID(0.046955, 0, 0) //0.035014
        .withFeedforward(0.10211, 0.11705, 0.010939) //ks: 0.083079
        // .withConversionFactor(1)
        // .withEncoderType(EncoderType.INTEGRATED)
        .withVelocityLimits(RotationsPerSecond.of(-60), RotationsPerSecond.of(200))
        .withName("ShooterWheel")
        // .withOffset(0)
        .withMainNum(0);
    
    private static final MotorConstants rightMotor = new MotorConstants() // LEADER
        .withDeviceId(20)
        .withCANBus(DroidRageConstants.rioCanBus)
        .withDirection(InvertedValue.CounterClockwise_Positive)
        .withIdleMode(NeutralModeValue.Coast)
        .withSupplyCurrentLimit(55)//80
        .withStatorCurrentLimit(60);//80

    private static final MotorConstants leftMotor = new MotorConstants() // FOLLOWER
        .withDeviceId(21)
        .withCANBus(DroidRageConstants.rioCanBus)
        .withMotorAlignment(MotorAlignmentValue.Opposed)
        .withIdleMode(NeutralModeValue.Coast)
        .withSupplyCurrentLimit(80)
        .withStatorCurrentLimit(80);

    public ShooterWheel(boolean isEnabled) {
        super(constants, new FlywheelIOTalonFX(isEnabled, constants, rightMotor, leftMotor));
    }
}

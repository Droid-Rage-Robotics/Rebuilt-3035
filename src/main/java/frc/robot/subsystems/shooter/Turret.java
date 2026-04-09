package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import frc.robot.DroidRageConstants;
import frc.utility.devices.encoder.EncoderConstants;
import frc.utility.devices.motor.MotorConstants;
import frc.utility.devices.motor.MotorConstants.Direction;
import frc.utility.template.SubsystemConstants;
import frc.utility.template.SubsystemConstants.EncoderType;
import frc.utility.template.TurretTemplate;

public class Turret extends TurretTemplate {
    private static final SubsystemConstants constants = new SubsystemConstants()
        // .withPID(100, 0, 10)
        .withPID(70, 0, 0.5)
        .withFeedforward(0.19682, 4.0022, 0.14262)
        // .withMaxVelocity(RotationsPerSecond.of(20))
        // .withMaxAcceleration(RotationsPerSecondPerSecond.of(500))
        // .withMaxJerk(0)
        .withGearRatio(24.0)
        .withEncoderType(EncoderType.EXTERNAL)
        // .withMinAngle(Degrees.of(-145))
        // .withMaxAngle(Degrees.of(185))
        .withMinAngle(Degrees.of(-11))//-23
        .withMaxAngle(Degrees.of(303)) //335
        .withName("Turret")
        .withOffset(0)
        .withMainNum(0);
    
    private static final MotorConstants motorConstants = new MotorConstants() 
        .withDeviceId(24)
        .withCANBus(DroidRageConstants.rioCanBus)
        .withDirection(Direction.Reversed)
        .withIdleMode(NeutralModeValue.Brake)
        .withSupplyCurrentLimit(15)
        .withStatorCurrentLimit(30);
    
    private static final EncoderConstants encoderConstants = new EncoderConstants()
        .withDeviceId(22)    
        .withCANBus(DroidRageConstants.rioCanBus)
        .withDirection(SensorDirectionValue.CounterClockwise_Positive);
    
    public Turret(boolean isEnabled) {
        super(isEnabled, constants, encoderConstants, motorConstants);
    }
}
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
        .withPID(10, 0, 0)
        .withFeedforward(0.11055, 1.6667, 0.15809)
        .withMaxVelocity(RotationsPerSecond.of(7.5))
        .withMaxAcceleration(RotationsPerSecondPerSecond.of(7.5))
        .withGearRatio(20.0)
        .withEncoderType(EncoderType.EXTERNAL)
        .withMinAngle(Degrees.of(-140))
        .withMaxAngle(Degrees.of(185))
        .withName("Turret")
        .withOffset(0)
        .withMainNum(0);
    
    private static final MotorConstants motorConstants = new MotorConstants() 
        .withDeviceId(24)
        .withCANBus(DroidRageConstants.rioCanBus)
        .withDirection(Direction.Forward)
        .withIdleMode(NeutralModeValue.Brake)
        .withSupplyCurrentLimit(30)
        .withStatorCurrentLimit(30);
    
    private static final EncoderConstants encoderConstants = new EncoderConstants()
        .withDeviceId(22)    
        .withCANBus(DroidRageConstants.rioCanBus)
        .withDirection(SensorDirectionValue.Clockwise_Positive);
    
    public Turret(boolean isEnabled) {
        super(isEnabled, constants, encoderConstants, motorConstants);
    }
}
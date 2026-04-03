package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.DroidRageConstants;
import frc.utility.devices.motor.MotorConstants;
import frc.utility.devices.motor.MotorConstants.Direction;
import frc.utility.template.FlywheelTemplate;
import frc.utility.template.SubsystemConstants;
import frc.utility.template.SubsystemConstants.EncoderType;

public class BottomRollers extends FlywheelTemplate {
    private static final SubsystemConstants constants = new SubsystemConstants()
        .withGearRatio(3.0)
        .withPID(0.017256, 0, 0)
        .withFeedforward(0.27325,0.35848,0.0072889)
        .withEncoderType(EncoderType.INTEGRATED)
        .withMinVelocity(RotationsPerSecond.of(-150))
        .withMaxVelocity(RotationsPerSecond.of(150))
        .withName("Bottom Rollers")
        .withOffset(0)
        .withMainNum(0);

    private static final MotorConstants leftMotor = new MotorConstants() 
        .withDeviceId(14)
        .withCANBus(DroidRageConstants.driveCanBus)
        .withDirection(Direction.Forward)
        .withIdleMode(NeutralModeValue.Coast)
        .withStatorCurrentLimit(80)//100
        .withSupplyCurrentLimit(50);//100
    
    public BottomRollers(boolean isEnabled) {
        super(isEnabled, constants, leftMotor);
    }
}

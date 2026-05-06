package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.DroidRageConstants;
import frc.utility.devices.motor.MotorConstants;
import frc.utility.io.FlywheelIOTalonFX;
import frc.utility.template.FlywheelTemplate;
import frc.utility.template.Constants.FlywheelConstants;

public class BottomRollers extends FlywheelTemplate {
    private static final FlywheelConstants constants = new FlywheelConstants()
        .withGearRatio(3.0)
        .withPID(0.017256, 0, 0)
        .withFeedforward(0.27325,0.35848,0.0072889)
        .withVelocityLimits(RotationsPerSecond.of(-150), RotationsPerSecond.of(150))
        .withName("Bottom Rollers")
        .withMainNum(0);

    private static final MotorConstants leftMotor = new MotorConstants() 
        .withDeviceId(14)
        .withCANBus(DroidRageConstants.driveCanBus)
        .withDirection(InvertedValue.Clockwise_Positive)
        .withIdleMode(NeutralModeValue.Coast)
        .withStatorCurrentLimit(80)//100
        .withSupplyCurrentLimit(75);//100
    
    public BottomRollers(boolean isEnabled) {
        super(constants, new FlywheelIOTalonFX(isEnabled, constants, leftMotor));
    }
}

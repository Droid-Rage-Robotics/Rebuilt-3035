package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.DroidRageConstants;
import frc.utility.devices.motor.MotorConstants;
import frc.utility.devices.motor.MotorConstants.Direction;
import frc.utility.io.FlywheelIOTalonFX;
import frc.utility.template.FlywheelTemplate;
import frc.utility.template.Constants.FlywheelConstants;

public class TopRoller extends FlywheelTemplate {
    private static final FlywheelConstants constants = new FlywheelConstants()
        .withGearRatio(1.0)
        .withPID(0.013449, 0, 0)
        .withFeedforward(0.2067, 0.11912, 0.0032711)
        .withVelocityLimits(RotationsPerSecond.of(-150), RotationsPerSecond.of(-150))
        .withName("Top Roller")
        .withMainNum(0);
    
    private static final MotorConstants rightMotor = new MotorConstants()
        .withDeviceId(17)
        .withCANBus(DroidRageConstants.driveCanBus)
        .withDirection(Direction.Reversed)
        .withIdleMode(NeutralModeValue.Coast)
        .withSupplyCurrentLimit(80)
        .withStatorCurrentLimit(75); 

    public TopRoller(boolean isEnabled) {
        super(constants, new FlywheelIOTalonFX(isEnabled, constants, rightMotor));
    }
}

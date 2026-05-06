package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.DroidRageConstants;
import frc.robot.subsystems.intake.Intake.IntakeValue;
import frc.utility.devices.motor.MotorConstants;
import frc.utility.io.FlywheelIOTalonFX;
import frc.utility.template.Constants.FlywheelConstants;
import frc.utility.template.FlywheelTemplate;

public class IntakeWheel extends FlywheelTemplate {
    private static final FlywheelConstants constants = new FlywheelConstants()
        .withPID(0.015741, 0, 0)
        .withFeedforward(0.50353, 0.12171, 0.0036507)
        .withVelocityLimits(RotationsPerSecond.of(-100), RotationsPerSecond.of(100))
        .withName("Intake")
        .withMainNum(0);

    private static final MotorConstants motorConstants = new MotorConstants() 
        .withDeviceId(15)
        .withCANBus(DroidRageConstants.driveCanBus)
        .withDirection(InvertedValue.CounterClockwise_Positive)
        .withIdleMode(NeutralModeValue.Brake)
        .withConversionFactor(1)
        .withMotorType(MotorType.KrakenX60)
        .withStatorCurrentLimit(65)//50
        .withSupplyCurrentLimit(60);//50

    public IntakeWheel(boolean isEnabled) {
        super(constants, new FlywheelIOTalonFX(isEnabled, constants, motorConstants));
    }

    public Command setTargetVelocityCommand(IntakeValue.WheelVelocity target) {
        return super.setTargetVelocityCommand(target.getVelocity());
    }
}

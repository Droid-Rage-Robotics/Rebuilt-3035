package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.DroidRageConstants;
import frc.utility.motor.MotorConstants;
import frc.utility.motor.MotorConstants.Direction;
import frc.utility.template.FlywheelTemplate;
import frc.utility.template.SubsystemConstants;
import frc.utility.template.SubsystemConstants.EncoderType;
import lombok.Getter;

public class Kicker extends FlywheelTemplate {
    public enum KickerValue {
        INTAKE(55),
        OUTTAKE(-55),
        STOP(0),
        HOLD(0);

        @Getter private final AngularVelocity kickerValue;

        private KickerValue(double kickerValue) {
            this.kickerValue = RotationsPerSecond.of(kickerValue);
        }
    } 

    private static final SubsystemConstants constants = new SubsystemConstants()
        .withConversionFactor(1)
        .withEncoderType(EncoderType.INTEGRATED)
        .withMinVelocity(RotationsPerSecond.of(-60))
        .withMaxVelocity(RotationsPerSecond.of(60))
        .withName("Kicker")
        .withOffset(0)
        .withMainNum(0);
    
    private static final MotorConstants motor = new MotorConstants() 
        .withDeviceId(18)
        .withCANBus(DroidRageConstants.rioCanBus)
        .withDirection(Direction.Forward)
        .withIdleMode(NeutralModeValue.Brake)
        .withConversionFactor( 1)
        .withSupplyCurrentLimit(50)
        .withStatorCurrentLimit(50);

    public Kicker(boolean isEnabled) {
        super(isEnabled,
            new PIDController(0.11878, 0, 0), 
            new SimpleMotorFeedforward(0.043254, 0.1229, 0.02026), 
            constants, 
            motor);
    }
}

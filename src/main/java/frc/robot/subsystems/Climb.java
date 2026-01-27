package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Distance;
import frc.robot.DroidRageConstants;
import frc.utility.motor.MotorConstants.Direction;
import frc.utility.motor.MotorConstants;
import frc.utility.template.ElevatorTemplate;
import frc.utility.template.SubsystemConstants;
import frc.utility.template.SubsystemConstants.EncoderType;
import lombok.Getter;

public class Climb extends ElevatorTemplate {
    public enum ClimbValue {
        START(Inches.of(0)),
        CLIMB(Inches.of(0));

        @Getter private final Distance height;

        private ClimbValue(Distance height) {
            this.height = height;
        }

    }

    private static final SubsystemConstants constants = new SubsystemConstants()
        .withConversionFactor(1)
        .withEncoderType(EncoderType.INTEGRATED)
        .withLowerLimit(0)
        .withUpperLimit(0)
        .withName("Turret")
        .withOffset(0)
        .withMainNum(0);
    
    private static final MotorConstants motorConstants = new MotorConstants() 
        .withCANBus(DroidRageConstants.rioCanBus)
        .withDirection(Direction.Forward)
        .withIdleMode(NeutralModeValue.Brake)
        .withConversionFactor( 1)
        .withSupplyCurrentLimit(70)
        .withStatorCurrentLimit(70);

    public Climb(boolean isEnabled) {
        super(isEnabled, 
            new ProfiledPIDController(0, 0, 0, 
            new TrapezoidProfile.Constraints(0, 0)), 
            new ElevatorFeedforward(0, 0, 0), 
            constants, 
            null, 
            motorConstants);
    }
}

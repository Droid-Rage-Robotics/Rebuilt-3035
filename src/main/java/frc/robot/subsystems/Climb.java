package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
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

    } //0.375

    private static final SubsystemConstants constants = new SubsystemConstants()
        .withConversionFactor(1)
        .withEncoderType(EncoderType.INTEGRATED)
        .withLowerLimit(Units.inchesToMeters(0))
        .withUpperLimit(Units.inchesToMeters(10))
        .withName("Climb")
        .withOffset(0)
        .withMainNum(0);
    
    private static final MotorConstants motorConstants = new MotorConstants() 
        .withDeviceId(17)
        .withCANBus(DroidRageConstants.driveCanBus)
        .withDirection(Direction.Forward)
        .withIdleMode(NeutralModeValue.Brake)
        .withConversionFactor((1.0/48.0) * 0.375)
        .withSupplyCurrentLimit(70)
        .withStatorCurrentLimit(70);

    public Climb(boolean isEnabled) {
        super(isEnabled, 
            new ProfiledPIDController(0.5, 0, 0, 
            new TrapezoidProfile.Constraints(0.5, 0.5)), 
            new ElevatorFeedforward(0, 0, 0), 
            constants, 
            null, 
            motorConstants);
    }
}

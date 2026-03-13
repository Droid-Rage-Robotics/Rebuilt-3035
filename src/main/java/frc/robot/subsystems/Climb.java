package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import frc.robot.DroidRageConstants;
import frc.utility.devices.motor.MotorConstants;
import frc.utility.devices.motor.MotorConstants.Direction;
import frc.utility.template.ElevatorTemplate;
import frc.utility.template.SubsystemConstants;
import frc.utility.template.SubsystemConstants.EncoderType;
import lombok.Getter;
import lombok.Setter;

public class Climb extends ElevatorTemplate {
    public enum ClimbValue {
        START(0),
        CLIMB(6);//5.48

        @Getter private final Distance height;

        private ClimbValue(double height) {
            this.height = Inches.of(height);
        }

    } //0.375

    private static final SubsystemConstants constants = new SubsystemConstants()
        .withEncoderType(EncoderType.INTEGRATED)
        .withMinDistance(Inches.of(-7))
        .withMaxDistance(Inches.of(8.5))//7
        .withName("Climb")
        .withConversionFactor(Units.inchesToMeters(0.375)/48.0)
        .withOffset(0)
        .withMainNum(0);
    
    private static final MotorConstants motorConstants = new MotorConstants() 
        .withDeviceId(17)
        .withCANBus(DroidRageConstants.driveCanBus)
        .withDirection(Direction.Reversed)
        .withIdleMode(NeutralModeValue.Brake)
        .withSupplyCurrentLimit(70) //Reefscape 120
        .withStatorCurrentLimit(70); //Reefscape 120

    @Setter public boolean isResetting = false;

    public Climb(boolean isEnabled) {
        super(isEnabled,
            new ProfiledPIDController(400, 0, 0, 
            new TrapezoidProfile.Constraints(40, 25)), 
            new ElevatorFeedforward(1, 0, 5), 
            constants, 
            null, 
            motorConstants);
    }

    @Override
    public void periodic() {
        if(isResetting) {
            return;
        } else {
            super.periodic();
        }
    }

    
}

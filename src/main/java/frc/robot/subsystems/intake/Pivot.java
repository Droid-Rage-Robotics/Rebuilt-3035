package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.DroidRageConstants;
import frc.utility.motor.MotorConstants;
import frc.utility.motor.MotorConstants.Direction;
import frc.utility.template.ArmTemplate;
import frc.utility.template.SubsystemConstants;
import frc.utility.template.SubsystemConstants.EncoderType;

public class Pivot extends ArmTemplate {
    private static final SubsystemConstants constants = new SubsystemConstants()
        .withConversionFactor(1.0/54.0)
        .withEncoderType(EncoderType.INTEGRATED)
        .withMinAngle(Degrees.zero())
        .withMaxAngle(Degrees.of(130))
        .withName("Pivot")
        .withOffset(0)
        .withMainNum(0);

    private static final MotorConstants motorConstants = new MotorConstants()
        .withDeviceId(16)
        .withCANBus(DroidRageConstants.driveCanBus)
        .withConversionFactor(1)
        .withDirection(Direction.Forward)
        .withIdleMode(NeutralModeValue.Coast)
        .withStatorCurrentLimit(70)
        .withSupplyCurrentLimit(70);


    public Pivot(boolean isEnabled) {
        super(isEnabled, 
            new ProfiledPIDController(5, 0, 0,
            new TrapezoidProfile.Constraints(2, 4)), 
            new ArmFeedforward(0.74109, 0.27134, 3.1928, 0.21903), 
            constants, 
            null, 
            motorConstants);
    }

    @Override
    public void periodic() {
        super.periodic();

        if (getCurrentAngle().getDegrees()<0) {
            resetEncoder();
        }
    }
}
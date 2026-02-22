package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.DroidRageConstants;
import frc.utility.motor.MotorConstants;
import frc.utility.motor.MotorConstants.Direction;
import frc.utility.template.FlywheelTemplate;
import frc.utility.template.SubsystemConstants;
import frc.utility.template.SubsystemConstants.EncoderType;
import lombok.Getter;

public class Indexer extends FlywheelTemplate{
    public enum IndexerValue {
        INTAKE(50),
        OUTTAKE(-25),
        STOP(0),
        HOLD(0);

        @Getter private final double indexerValue;

        private IndexerValue(double indexerValue) {
            this.indexerValue = indexerValue;
        }
    }
    
    private static final SubsystemConstants constants = new SubsystemConstants()
        .withConversionFactor(1.0/3.0)
        .withEncoderType(EncoderType.INTEGRATED)
        .withLowerLimit(-50)
        .withUpperLimit(50)
        .withName("Indexer")
        .withOffset(0)
        .withMainNum(0);
    
    private static final MotorConstants motor = new MotorConstants() 
        .withDeviceId(14)
        .withCANBus(DroidRageConstants.driveCanBus)
        .withDirection(Direction.Forward)
        .withIdleMode(NeutralModeValue.Coast)
        .withConversionFactor( 1);
        // .withSupplyCurrentLimit(70)
        // .withStatorCurrentLimit(70);

    public Indexer(boolean isEnabled) {
        super(isEnabled,
            new PIDController(0.032889, 0, 0), 
            new SimpleMotorFeedforward(0.34224, 0.37116, 0.0095347), 
            constants, 
            motor);
    }

    // @Override
    // public void periodic() {
    //     super.periodic();

    //     if (getCurrent().in(Amps) >= 100) {
    //         setTargetVelocity(-getTargetVelocity());
    //     }
    // }
}

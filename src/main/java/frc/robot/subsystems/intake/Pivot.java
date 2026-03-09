package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.DroidRageConstants;
import frc.robot.subsystems.intake.Intake.IntakeValue;
import frc.utility.devices.motor.MotorConstants;
import frc.utility.devices.motor.MotorConstants.Direction;
import frc.utility.template.ArmTemplate;
import frc.utility.template.SubsystemConstants;
import frc.utility.template.SubsystemConstants.EncoderType;

public class Pivot extends ArmTemplate {
    private static double startingPosDegree = 35;
    private static final SubsystemConstants constants = new SubsystemConstants()
        .withConversionFactor(1.0/54.0)
        .withPID(25, 0, 0.35)
        .withFeedforward(0.74109, 0.27134, 3.3, 0.23)
        .withMaxVelocity(RotationsPerSecond.of(10))
        .withMaxAcceleration(RotationsPerSecondPerSecond.of(15))
        .withEncoderType(EncoderType.INTEGRATED)
        .withMinAngle(Degrees.of(35))
        .withMaxAngle(Degrees.of(160))
        .withName("Pivot")
        .withOffset(Units.degreesToRotations(startingPosDegree))//Rotation
        .withMainNum(0);

    private static final MotorConstants motorConstants = new MotorConstants()
        .withDeviceId(16)
        .withCANBus(DroidRageConstants.driveCanBus)
        .withConversionFactor(1)
        .withDirection(Direction.Forward)
        .withIdleMode(NeutralModeValue.Coast);
        // .withStatorCurrentLimit(20) //Reefscape 50
        // .withSupplyCurrentLimit(15); //Reefscape None


    public Pivot(boolean isEnabled) {
        super(isEnabled, constants, null, motorConstants);
        
        setTargetPositionDegrees(startingPosDegree);
    }

    public Command setTargetPositionCommand(IntakeValue.PivotAngle goalAngle) {
        return super.setTargetPositionCommand(goalAngle.getAngle());
    }

    @Override
    public void periodic() {
        super.periodic();

        //TODO:TEst
        if (getCurrentAngle().getDegrees()<35) {
            resetEncoderCommand(0);
        }

    }

    // public void isPushed(){  //Might need to put a timeout period for this AND might need a switch to turn it off when mechanism is off
    //     boolean isPushed = pivot.getSetpointError()> 20 && pivot.getMotor().getVoltage()>5;
    //     if (isPushed){
    //         setPositionCommand(IntakeValue.HOLD);
    //     }
    // }
}
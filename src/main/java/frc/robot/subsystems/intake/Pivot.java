package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.DroidRageConstants;
import frc.robot.subsystems.intake.Intake.IntakeValue;
import frc.utility.devices.motor.MotorConstants;
import frc.utility.devices.motor.MotorConstants.Direction;
import frc.utility.template.ArmTemplate;
import frc.utility.template.SubsystemConstants;
import frc.utility.template.SubsystemConstants.EncoderType;

public class Pivot extends ArmTemplate {
    private static final double startingPosDegree = 35;
    private static final SubsystemConstants constants = new SubsystemConstants()
        .withPID(25, 0, 0.35)
        .withFeedforward(0.74109, 0.27134, 3.3, 0.23)
        .withMaxVelocity(RotationsPerSecond.of(10))
        .withMaxAcceleration(RotationsPerSecondPerSecond.of(15))
        .withGearRatio(54.0)
        .withEncoderType(EncoderType.INTEGRATED)
        .withMinAngle(Degrees.of(35))
        .withMaxAngle(Degrees.of(168))
        .withName("Pivot")
        // .withOffset(Units.degreesToRotations(startingPosDegree))//Rotation
        .withResetAngle(Degrees.of(startingPosDegree))
        .withMainNum(0);

    private static final MotorConstants motorConstants = new MotorConstants()
        .withDeviceId(16)
        .withCANBus(DroidRageConstants.driveCanBus)
        .withConversionFactor(1)
        .withDirection(Direction.Forward)
        .withIdleMode(NeutralModeValue.Brake)
        .withStatorCurrentLimit(50) //Reefscape 50
        .withSupplyCurrentLimit(40); //Reefscape None
        //TODO: Uncomment and check the command to turn on and off current limit


    public Pivot(boolean isEnabled) {
        super(isEnabled, constants, null, motorConstants);

            setGoalAngle(Degrees.of(startingPosDegree));

        // controller.setTolerance(Units.degreesToRadians(1), 1);
    }

    @Override
    public void elasticInit() {
        SmartDashboard.putData(constants.name + "/Reset Encoder", resetEncoderCommand(Units.degreesToRotations(startingPosDegree)));
    }

    public Command setTargetPositionCommand(IntakeValue.PivotAngle goalAngle) {
        return super.setTargetPositionCommand(goalAngle.getAngle());
    }

    @Override
    public void periodic() {
        super.periodic();

       // System.out.println("Pivot Angle: " + getCurrentAngle().getDegrees());
        //TODO:TEst
        // if (getCurrentAngle().in(Degrees)<35) {
        //     resetEncoderCommand(35);
        // }

    }

    // public void isPushed(){  //Might need to put a timeout period for this AND might need a switch to turn it off when mechanism is off
    //     boolean isPushed = pivot.getSetpointError()> 20 && pivot.getMotor().getVoltage()>5;
    //     if (isPushed){
    //         setPositionCommand(IntakeValue.HOLD);
    //     }
    // }

    public void turnCurrentLimitOff(){ 
        getMotor().turnCurrentLimitOff();
    }
    public void turnCurrentLimitOn(){ 
        getMotor().turnCurrentLimitOn();

    }
}
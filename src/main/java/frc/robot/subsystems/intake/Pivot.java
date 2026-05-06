package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.DroidRageConstants;
import frc.robot.subsystems.intake.Intake.IntakeValue;
import frc.robot.subsystems.intake.Intake.IntakeValue.PivotAngle;
import frc.utility.devices.motor.MotorConstants;
import frc.utility.io.ArmIOTalonFX;
import frc.utility.template.ArmTemplate;
import frc.utility.template.Constants.ArmConstants;
import frc.utility.template.Constants.EncoderType;

public class Pivot extends ArmTemplate {
    private static final Angle startingPos = Degrees.of(35);

    private static final ArmConstants constants = new ArmConstants()
        .withPID(25, 0, 0.35)
        .withFeedforward(0.74109, 0.27134, 3.3, 0.25)//kA:0.23 kV:3.3
        .withMotionMagic(RotationsPerSecond.of(10), RotationsPerSecondPerSecond.of(15), 0)
        .withGearRatio(54.0)
        .withEncoderType(EncoderType.INTEGRATED)
        .withLimits(startingPos, Degrees.of(168))
        .withName("Pivot")
        .withResetAngle(startingPos);

    private static final MotorConstants motorConstants = new MotorConstants()
        .withDeviceId(16)
        .withCANBus(DroidRageConstants.driveCanBus)
        .withConversionFactor(1)
        .withDirection(InvertedValue.Clockwise_Positive)
        .withIdleMode(NeutralModeValue.Brake)
        .withStatorCurrentLimit(45) //Reefscape 50
        .withSupplyCurrentLimit(35); //Reefscape None
        //TODO: Uncomment and check the command to turn on and off current limit

    public final Trigger highCurrent = new Trigger(() -> (getInputs().statorCurrent.in(Amps) > 15));

    public Pivot(boolean isEnabled, CommandXboxController driver) {
        super(isEnabled, constants, new ArmIOTalonFX(isEnabled, constants, null, motorConstants));

        setGoalAngle(startingPos);

        highCurrent.and(driver.rightTrigger()).whileTrue(
            new SequentialCommandGroup(
                new WaitCommand(1.5),
                new InstantCommand(() -> System.out.println("INTAKE STALL!")),
                new InstantCommand(() -> System.out.println(getGoalAngle().in(Degrees))),

                resetEncoderCommand(PivotAngle.DOWN.getAngle())  
            )
        );
        
    }

    public Command setTargetPositionCommand(IntakeValue.PivotAngle goalAngle) {
        return super.setTargetPositionCommand(goalAngle.getAngle());
    }

    @Override
    public void periodic() {
        super.periodic();

        // if (isStalled()) {
        //     resetEncoder(getGoalAngle().in(Rotations));
        // }

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

    // public void turnCurrentLimitOff(){ 
    //     getMotor().turnCurrentLimitOff();
    // }
    // public void turnCurrentLimitOn(){ 
    //     getMotor().turnCurrentLimitOn();

    // }
}
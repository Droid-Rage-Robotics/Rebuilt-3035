package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import lombok.Getter;

public class Intake implements Sendable {
    public class IntakeValue {
        public enum PivotAngle {
            DOWN(164),//166
            AUTO_DOWN(155),
            HALF_THREE(DOWN.getAngle().in(Degrees)-5),
            HALF_TWO(DOWN.getAngle().in(Degrees)-10),
            HALF_ONE(DOWN.getAngle().in(Degrees)-15),

            UP(40),
            HALF(105),
            ;

            @Getter private final Angle angle;

            private PivotAngle(double angle) {
                this.angle = Degrees.of(angle);
            }
        }

        public enum WheelVelocity {
            INTAKE(-120), //-90
            OUTTAKE(90),
            STOP(0)
            ;

            @Getter private final AngularVelocity velocity;

            private WheelVelocity(double velocity) {
                this.velocity = RotationsPerSecond.of(velocity);
            }
        }
    }

    @Getter private final Pivot pivot;
    @Getter private final IntakeWheel intakeWheel;

    private IntakeValue intakeValue;
    // public final BooleanSupplier atGoal;

    public Intake(Pivot pivot, IntakeWheel intakeWheel){
        this.pivot = pivot;
        this.intakeWheel = intakeWheel;
        // this.atGoal = pivot::atGoal;

        // CommandScheduler.getInstance().schedule(setPositionCommand(IntakeValue.STOP));
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("IntakeValue", this::getIntakeValue, null);
    }
    
    public String getIntakeValue() {
        return intakeValue.toString();
    }

    // public Command setTargetVelocityWaitCommand(WheelVelocity target) {
    //     return new SequentialCommandGroup(
    //         new WaitCommand(0.25),   
    //         intakeWheel.setTargetVelocityCommand(target)
    //     );
    // }
    
    // public Command setPositionCommand(IntakeValue targetPos) {
    //     intakeValue = targetPos;
    //     return new SequentialCommandGroup (
    //         pivot.setTargetPositionCommand(targetPos.getPivotAngle()),
    //         intakeWheel.setTargetVelocityCommand(targetPos.intakeSpeed)
    //     );
    // }

    public boolean isIntakeValue(IntakeValue value){
        return intakeValue == value;
    }

    // public void isPushed(){  //Might need to put a timeout period for this AND might need a switch to turn it off when mechanism is off
    //     boolean isPushed = pivot.getSetpointError()> 20 && pivot.getMotor().getVoltage()>5;
    //     if (isPushed){
    //         setPositionCommand(IntakeValue.HOLD);
    //     }
    // }
}

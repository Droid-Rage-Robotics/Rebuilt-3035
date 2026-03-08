package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import lombok.Getter;

public class Intake implements Sendable {
    public class IntakeValue {
        public enum PivotAngle {
            DOWN(170),
            UP(40),
            ;

            @Getter private final Rotation2d angle;

            private PivotAngle(double angle) {
                this.angle = Rotation2d.fromDegrees(angle);
            }
        }

        public enum WheelVelocity {
            INTAKE(-100),
            OUTTAKE(100),
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

    public Intake(Pivot pivot, IntakeWheel intakeWheel){
        this.pivot = pivot;
        this.intakeWheel = intakeWheel;
        // CommandScheduler.getInstance().schedule(setPositionCommand(IntakeValue.STOP));
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("IntakeValue", this::getIntakeValue, null);
    }
    
    public String getIntakeValue() {
        return intakeValue.toString();
    }
    
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

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import lombok.Getter;

public class Intake {
    public enum IntakeValue{
        STOP(0,0),

        INTAKE(0,0),
        OUTTAKE(0,0),

        HOLD(0,0)

        ;

        /*
        @Getter is an annotation from the lombok plugin.
        It creates a method to return stuff without creating our own getters.
        */ 
        @Getter private final double pivotAngle;
        @Getter private final double intakeSpeed;


        private IntakeValue(double pivotAngle, double intakeSpeed){
            this.pivotAngle = pivotAngle;
            this.intakeSpeed = intakeSpeed;
        }
        
        private IntakeValue(IntakeValue value) {
            this.pivotAngle = value.pivotAngle;
            this.intakeSpeed = value.intakeSpeed;
        }
    }

    @Getter private final Pivot pivot;
    @Getter private final IntakeWheel intakeWheel;

    private IntakeValue position;

    public Intake(Pivot pivot, IntakeWheel intakeWheel){
        this.pivot = pivot;
        this.intakeWheel = intakeWheel;
        pivot.setTargetPositionDegrees(IntakeValue.STOP.pivotAngle);
        intakeWheel.setTargetVelocity(IntakeValue.STOP.intakeSpeed);
        position = IntakeValue.STOP;
        SmartDashboard.putString("Carriage/Position", "INTAKE_HPS");
    }
    
    public IntakeValue getPosition() {
        return position;
    }
    
    public Command setPositionCommand(IntakeValue targetPos) {
        return Commands.sequence(    
            switch (targetPos) {      
                default -> 
                    new SequentialCommandGroup(
                        pivot.setTargetPositionCommand(targetPos.getPivotAngle()),
                        intakeWheel.setTargetVelocityCommand(targetPos.intakeSpeed)
                    );
            },
            
            new InstantCommand(() -> position=targetPos),
            new InstantCommand(()-> SmartDashboard.putString("Intake/Position", targetPos.name()))
        );
    }
    /** Not something to Command anything other than to make the writers reflect the position */
     public Command setPosition(IntakeValue targetPos) {
         return new SequentialCommandGroup(
            new InstantCommand(()-> position = targetPos)
        );
    }

    public boolean isIntakeValue(IntakeValue value){
        return position == value;
    }

    public void isPushed(){  //Might need to put a timeout period for this AND might need a switch to turn it off when mechanism is off
        boolean isPushed = pivot.getSetpointError()> 20 && pivot.getMotor().getVoltage()>5;
        if (isPushed){
            setPositionCommand(IntakeValue.HOLD);
        }
    }
}

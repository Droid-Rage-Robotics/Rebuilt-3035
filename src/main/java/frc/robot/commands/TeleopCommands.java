package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorValue;
import frc.robot.subsystems.carriage.Carriage;
import frc.robot.subsystems.carriage.Carriage.CarriageIntakeValue;
import frc.robot.subsystems.carriage.Carriage.CarriageValue;

public class TeleopCommands{
    public Command teleopOuttakeCommand(Carriage carriage){
        return SuppliedCommand.create(() -> Commands.sequence(
                switch (carriage.getPosition()) {
                    case L1:
                        yield carriage.setIntakeCommand(CarriageIntakeValue.OUTTAKE_L1);
                    case BARGE:
                        yield carriage.setIntakeCommand(CarriageIntakeValue.SHOOT);
                    case PROCESSOR:
                        yield carriage.setIntakeCommand(CarriageIntakeValue.OUTTAKE_PROCESSOR);
                    default:
                        yield carriage.setIntakeCommand(CarriageIntakeValue.OUTTAKE);
                }));
        // return new SequentialCommandGroup(
        //     new ConditionalCommand(
        //         carriage.setIntakeCommand(CarriageIntakeValue.OUTTAKE_L1),
        //             new ConditionalCommand(
        //                     carriage.setIntakeCommand(CarriageIntakeValue.SHOOT),
        //                     carriage.setIntakeCommand(CarriageIntakeValue.OUTTAKE),
        //                     () -> carriage.isCarriageValue(CarriageValue.BARGE)),
        //         () -> carriage.isCarriageValue(CarriageValue.L1))
        // );
    }

    public Command teleopHoldCommand(Carriage carriage) {
        return SuppliedCommand.create(() -> Commands.sequence(
            switch(carriage.getPosition()){
                case ALGAE_HIGH, ALGAE_LOW, BARGE, BARGE_HOLD, INTAKE_GROUND, PROCESSOR:
                    yield carriage.setIntakeCommand(CarriageIntakeValue.HOLD_ALGAE);
                    // yield carriage.setIntakeCommand(CarriageIntakeValue.HOLD_CORAL);
                    
                case INTAKE_HPS, INTAKE_HPS_BLOCK, L1, L2, L3,L4:
                    yield carriage.setIntakeCommand(CarriageIntakeValue.HOLD_CORAL);

                default:
                    yield carriage.setIntakeCommand(CarriageIntakeValue.STOP);
            }
        ));
    }
    
    public SequentialCommandGroup runIntakeFor(Carriage carriage, CarriageIntakeValue value, double waitSec) {
        return new SequentialCommandGroup(
            carriage.setIntakeCommand(value),
            new WaitCommand(waitSec),
            carriage.setIntakeCommand(CarriageIntakeValue.STOP)
        );
    }

    public SequentialCommandGroup barge(Elevator elevator, Carriage carriage){
        return new SequentialCommandGroup(
            // carriage.getPivot().setTargetPositionCommand(CarriageValue.BARGE_HOLD.getPivotAngle()+5),
            // new WaitCommand(.2),
            // carriage.setPositionCommand(CarriageValue.BARGE_HOLD),
            // new WaitUntilCommand(()->
            //     Math.abs(carriage.getArm().getTargetPosition()-carriage.getArm().getEncoderPosition())<3),
            // elevator.setTargetPositionCommand(ElevatorValue.BARGE),
            // new WaitUntilCommand(()-> elevator.getPosition()>=(47 * Elevator.Constants.MOTOR_ROT_2_METER)),
            // // new WaitCommand(1),

            carriage.getArm().setTargetPositionCommand(CarriageValue.L4.getArmAngle()),
            new ParallelCommandGroup(
                elevator.setTargetPositionCommand(ElevatorValue.BARGE),
                carriage.getPivot().setTargetPositionCommand(CarriageValue.L4.getPivotAngle())
            ),
            new WaitUntilCommand(()-> elevator.getPosition()>=(47 * Elevator.Constants.MOTOR_ROT_2_METER)),
            carriage.setPositionCommand(CarriageValue.BARGE),
            carriage.setPosition(CarriageValue.BARGE),
            new InstantCommand(()-> SmartDashboard.putString("Carriage/Position", "BARGE"))
        );
    }
    

    public SequentialCommandGroup goL4(Elevator elevator, Carriage carriage){
        return new SequentialCommandGroup(
            carriage.getArm().setTargetPositionCommand(CarriageValue.L4.getArmAngle()),
            // new WaitUntilCommand(()->carriage.getArm().atSetpoint()).withTimeout(.25),
            new ParallelCommandGroup(
                elevator.setTargetPositionCommand(ElevatorValue.L4),
                carriage.getPivot().setTargetPositionCommand(CarriageValue.L4.getPivotAngle())
            ),
            new InstantCommand(()-> SmartDashboard.putString("Carriage/Position", "L4"))
        );
    }
    public SequentialCommandGroup autoGoL4(Elevator elevator, Carriage carriage){
        return new SequentialCommandGroup(
            carriage.getArm().setTargetPositionCommand(CarriageValue.L4.getArmAngle()),
            new WaitUntilCommand(()->carriage.getArm().atSetpoint()),//.withTimeout(.25),
            new ParallelCommandGroup(
                elevator.setTargetPositionCommand(ElevatorValue.L4),
                carriage.getPivot().setTargetPositionCommand(CarriageValue.L4.getPivotAngle())
            ),
            // new WaitUntilCommand(()->elevator.getEncoderPosition()>ElevatorValue.L4.getHeight()-1.5)
            carriage.setPosition(CarriageValue.L4),
            new InstantCommand(()-> SmartDashboard.putString("Carriage/Position", "L4"))
        );
    }


    public Command resetHP(Elevator elevator, Carriage carriage, CarriageValue value){
        return SuppliedCommand.create(() -> Commands.sequence(
            switch(carriage.getPosition()){
                    case BARGE, BARGE_HOLD, L4:
                        yield new SequentialCommandGroup(
                                carriage.setPositionCommand(CarriageValue.L4),
                                new WaitCommand(1),
                                elevator.setTargetPositionCommand(ElevatorValue.GROUND),
                                new WaitCommand(0.2),

                                new WaitUntilCommand(() -> elevator.getPosition() < elevator.resetPos),
                                carriage.getArm().setTargetPositionCommand(value.getArmAngle()),
                                carriage.getPivot().setTargetPositionCommand(value.getPivotAngle())
                        );
                    case INTAKE_GROUND:
                        yield new SequentialCommandGroup(
                                elevator.setTargetPositionCommand(ElevatorValue.INTAKE_HPS),
                                new WaitUntilCommand(() -> elevator.getPosition() <= elevator.resetPos),
                                // carriage.setPositionCommand(CarriageValue.L1),
                                // carriage.setPositionCommand(value)
                                carriage.getArm().setTargetPositionCommand(value.getArmAngle()),
                                carriage.getPivot().setTargetPositionCommand(value.getPivotAngle())
                        );
                    default:
                        yield new SequentialCommandGroup(
                                elevator.setTargetPositionCommand(ElevatorValue.INTAKE_HPS),
                                new WaitUntilCommand(() -> elevator.getPosition() <= elevator.resetPos),
                                new ParallelCommandGroup(
                                    carriage.getPivot().setTargetPositionCommand(value.getPivotAngle()),
                                    carriage.getArm().setTargetPositionCommand(value.getArmAngle())
                                )

                );
            },
            carriage.setPosition(value),
            new InstantCommand(()-> SmartDashboard.putString("Carriage/Position", "INTAKE_HPS"))

        ));
    }

    public TeleopCommands() {}
}

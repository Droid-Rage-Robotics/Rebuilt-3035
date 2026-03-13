package frc.robot.commands.manual;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meter;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.DroidRageConstants;
import frc.robot.subsystems.Climb;

public class ManualClimb extends Command {
    private final Climb climb;
    private final Supplier<Double> climbMove;
    
    public ManualClimb(Climb climb, Supplier<Double> elevatorMove) {
        this.climb = climb;
        this.climbMove = elevatorMove;
        
        addRequirements(climb);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double move = climbMove.get();
        move = DroidRageConstants.squareInput(move);
        move = DroidRageConstants.applyDeadBand(move);
        // if(move<0){
            // return;
        // }
        climb.setTargetPosition(Inches.of(climb.getGoalPosition().in(Inches) + move * 0.06));
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
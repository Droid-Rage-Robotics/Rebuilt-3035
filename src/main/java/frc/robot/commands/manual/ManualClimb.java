package frc.robot.commands.manual;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.DroidRageConstants;
import frc.robot.subsystems.Climb;

public class ManualClimb extends Command {
    private final Climb climb;
    private final Supplier<Double> elevatorMove;
    
    public ManualClimb(Climb climb, Supplier<Double> elevatorMove) {
        this.climb = climb;
        this.elevatorMove = elevatorMove;
        
        addRequirements(climb);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double move = -elevatorMove.get();
        move = DroidRageConstants.squareInput(move);
        move = DroidRageConstants.applyDeadBand(move);
        if(move<0){
            return;
        }
        climb.setTargetPosition(Math.toDegrees(climb.getTargetPosition()) + move * 0.45);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}

package frc.robot.commands.manual;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.DroidRageConstants;
import frc.robot.subsystems.carriage.Pivot;

public class ManualPivot extends Command {
    private final Pivot pivot;
    private final Supplier<Double> pivotMove;

    public ManualPivot(Pivot pivot, Supplier<Double> pivotMove) {
        this.pivot = pivot;
        this.pivotMove = pivotMove;
        
        addRequirements(pivot);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double move = -pivotMove.get();
        move = DroidRageConstants.squareInput(move);
        move = DroidRageConstants.applyDeadBand(move);
        pivot.setTargetPosition(pivot.getTargetPosition() + move * 0.05);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
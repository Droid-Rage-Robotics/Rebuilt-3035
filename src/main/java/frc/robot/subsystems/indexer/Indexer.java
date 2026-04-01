package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import lombok.Getter;

public class Indexer {
    public enum IndexerValue {
        INTAKE(50,30),
        OUTTAKE(-50,-30),
        STOP(0,0),
        HOLD(0,0);

        @Getter private final AngularVelocity top;
        @Getter private final AngularVelocity bottom;


        private IndexerValue(double top, double bottom) {
            this.bottom = RotationsPerSecond.of(bottom);
            this.top = RotationsPerSecond.of(top);

        }
    }
    
    @Getter private final BottomRollers bottomRollers;
    @Getter private final TopRoller topRoller;

    public Indexer(BottomRollers bottomRollers, TopRoller topRoller) {
        this.bottomRollers=bottomRollers;
        this.topRoller=topRoller;
    }

    public boolean isIndexerOn(){
        return !(bottomRollers.getTargetVelocity() == IndexerValue.STOP.bottom);
    }

    public Command setTargetVelocityCommand(IndexerValue target) {
        return new ParallelCommandGroup(
            bottomRollers.setTargetVelocityCommand(target.bottom),
            topRoller.setTargetVelocityCommand(target.top)
        );
    }

    public void setTargetVelocity(IndexerValue target) {
        bottomRollers.setTargetVelocity(target.bottom);
        topRoller.setTargetVelocity(target.top);
    }
}

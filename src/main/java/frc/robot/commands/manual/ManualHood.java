package frc.robot.commands.manual;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.DroidRageConstants;
import frc.robot.subsystems.shooter.Shooter;

public class ManualHood extends Command{
    private Shooter shooter;
    private CommandXboxController operator;

    public ManualHood(Shooter shooter, CommandXboxController operator){
        this.shooter = shooter;
        this.operator = operator;

        addRequirements(shooter.getHood());
    }

    @Override
    public void execute(){
        if(!DroidRageConstants.isWithinDeadzone(operator.getRightY())){
            shooter.getHood().setGoalAngle(
                Degrees.of(shooter.getHood().getGoalAngle().in(Degrees)+operator.getRightY()*-0.4)
            );
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}

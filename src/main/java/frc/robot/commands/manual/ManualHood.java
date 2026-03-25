package frc.robot.commands.manual;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.DroidRageConstants;
import frc.robot.DroidRageConstants.ControllerUtils;
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
        // if(operator.y().getAsBoolean()){
            if(!DroidRageConstants.isWithinDeadzone(operator.getRightY())){
                shooter.getHood().setGoalAngle(
                    Rotation2d.fromDegrees(shooter.getHood().getGoalAngle().getDegrees()+operator.getRightY()*-0.7)
                );
            }
        // }
        

        // if(operator.povUp().getAsBoolean()){
        //     shooter.getHood().setGoalAngle(
        //         Rotation2d.fromDegrees(shooter.getHood().getGoalAngle().getDegrees()+0.4));
        // }
        // if(operator.povDown().getAsBoolean()){
        //     shooter.getHood().setGoalAngle(
        //         Rotation2d.fromDegrees(shooter.getHood().getGoalAngle().getDegrees()-0.4));
        // }

    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}

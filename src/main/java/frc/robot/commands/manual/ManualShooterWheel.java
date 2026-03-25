package frc.robot.commands.manual;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.DroidRageConstants;
import frc.robot.DroidRageConstants.ControllerUtils;
import frc.robot.subsystems.shooter.Shooter;

public class ManualShooterWheel extends Command{
    private Shooter shooter;
    private CommandXboxController operator;

    public ManualShooterWheel(Shooter shooter, CommandXboxController operator){
        this.shooter = shooter;
        this.operator = operator;

        addRequirements(shooter.getShooterWheel());
    }

    @Override
    public void execute(){
        // if(operator.x().getAsBoolean()){
            if(!DroidRageConstants.isWithinDeadzone(operator.getLeftY())){
                shooter.getShooterWheel().setTargetVelocity(
                    RotationsPerSecond.of(shooter.getShooterWheel().getTargetVelocity().magnitude()+
                        (-.6*operator.getLeftY())));
        }
        // }
        
        
        // if(operator.povRight().getAsBoolean()){
        //     shooter.getShooterWheel().setTargetVelocity(
        //         RotationsPerSecond.of(shooter.getShooterWheel().getTargetVelocity().magnitude()+.5));
        // }
        // if(operator.povLeft().getAsBoolean()){
        //     shooter.getShooterWheel().setTargetVelocity(
        //         RotationsPerSecond.of(shooter.getShooterWheel().getTargetVelocity().magnitude()-.5));
        // }

    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}

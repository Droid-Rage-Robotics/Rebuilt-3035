package frc.robot.commands.manual;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.DroidRageConstants;
import frc.robot.DroidRageConstants.ControllerUtils;
import frc.robot.subsystems.shooter.Shooter;

public class ManualTurret extends Command{
    private Shooter shooter;
    private CommandXboxController operator;

    public ManualTurret(Shooter shooter, CommandXboxController operator){
        this.shooter = shooter;
        this.operator = operator;

        addRequirements(shooter.getTurret());
    }

    @Override
    public void execute(){
        if(DroidRageConstants.isWithinDeadzone(operator.getRightX())
            &&DroidRageConstants.isWithinDeadzone(operator.getRightY())){

        // if(ControllerUtils.getRightStickRotation2d(operator) != new Rotation2d(0)){
            // shooter.getTurret().setTargetPositionDegrees(ControllerUtils.getRightStickDeg(operator)); //ToDo: Test
            shooter.getTurret().setGoalAngle(ControllerUtils.getRightStickRotation2d(operator)); //ToDo: Test
            // System.out.println(ControllerUtils.getRightStickDeg(operator));
            System.out.println(ControllerUtils.getRightStickRotation2d(operator));


        } else {
            shooter.getTurret().setGoalAngle(shooter.getTurret().getGoalAngle()); //ToDo: Test
        }

    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}

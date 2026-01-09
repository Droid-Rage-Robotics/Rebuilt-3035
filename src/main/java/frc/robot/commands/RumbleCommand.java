package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;;

public class RumbleCommand extends Command {
	public enum RumbleStates {
		INTAKE,
		ELEMENT_IN,
		NOTHING,
		END_GAME,
		ELEVATOR,
		ALIGN    
	}
		
	private final CommandXboxController driver;   

	public RumbleCommand(CommandXboxController driver) {
		this.driver = driver;
	}
    
	@Override
	public void execute() {
		driver.setRumble(RumbleType.kLeftRumble, 1.0);
		driver.setRumble(RumbleType.kRightRumble, 1.0);
	}
	
	@Override
	public void end(boolean interrupted) {
		driver.setRumble(RumbleType.kLeftRumble, 0);
		driver.setRumble(RumbleType.kRightRumble, 0);
	}
}
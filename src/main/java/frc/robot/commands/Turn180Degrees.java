package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.SwerveDrive;

public class Turn180Degrees extends Command {
    private final SwerveDrive drive;
    private final PIDController controller;
    private double targetAngle;
    private CommandXboxController driver;

    public Turn180Degrees(SwerveDrive drive, CommandXboxController driver) {
        this.drive = drive;
        this.controller = new PIDController(0.03, 0, 0);  // Tune these PID constants as needed
        // controller.setTolerance(1);
        this.driver = driver;
        addRequirements(drive);

    }

    @Override
    public void initialize() {
        
    }
    @Override
    public void execute() {
        if (driver.x().getAsBoolean()) {

            // Get the current angle of the robot and set the target to 180 degrees away
            targetAngle = drive.getHeading() + 180;
        }
        
        // Reset PID controller with the current gyro angle
        controller.setSetpoint(targetAngle);
        // Use the PID controller to get the turn speed
        double turnSpeed = controller.calculate(drive.getHeading());
        // Apply turn speed to the drivetrain
        drive.drive(0,0, turnSpeed);  // Assuming you only want to turn, so forward speed is 0
    }

    @Override
    public boolean isFinished() {
        // Check if the robot is within tolerance of the target angle
        return controller.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the drivetrain when the command finishes
        drive.stop();
    }
}

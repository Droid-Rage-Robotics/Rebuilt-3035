package frc.robot.commands.manual;

import static edu.wpi.first.units.Units.*;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.DroidRageConstants;
import frc.robot.subsystems.drive.SwerveConfig;
import frc.robot.subsystems.drive.SwerveConfig.DriveOptions;
import frc.robot.subsystems.drive.SwerveConfig.Speed;
import frc.robot.subsystems.drive.maple.Drive;

public class SwerveDriveTeleop extends Command {
    private final Drive drive;
    
    private final Supplier<Double> x, y, turn;
    private volatile double xSpeed, ySpeed, turnSpeed;

    private final BooleanSupplier noInput;
    // private static final PIDController antiTipY = 
    //     new PIDController(0.006, 0, 0.0005);
    // private static final PIDController antiTipX = 
    //     new PIDController(0.006, 0, 0.0005);

    public SwerveDriveTeleop(Drive drive, CommandXboxController driver) {
        this.drive = drive;
        this.x = driver::getLeftX;
        this.y = driver::getLeftY;
        this.turn = driver::getRightX;
        // antiTipX.setTolerance(2);
        // antiTipY.setTolerance(2);

        noInput = () ->
            xSpeed == 0.0 &&
            ySpeed == 0.0 &&
            turnSpeed == 0.0;

        driver.rightBumper().onTrue(drive.setSpeed(Speed.SLOW))
            .onFalse(drive.setSpeed(Speed.NORMAL));
        driver.leftBumper().onTrue(drive.setSpeed(Speed.SUPER_SLOW))
            .onFalse(drive.setSpeed(Speed.NORMAL));

        driver.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

        driver.b().onTrue(new InstantCommand(drive::seedFieldCentric));

        addRequirements(drive);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        xSpeed = -y.get(); //Forward
        ySpeed = -x.get(); //Strafe
        turnSpeed = -turn.get(); //Turn

        // Square inputs
        if (DriveOptions.IS_SQUARED_INPUTS.get()) {
            xSpeed = DroidRageConstants.squareInput(xSpeed);
            ySpeed = DroidRageConstants.squareInput(ySpeed);
            turnSpeed = DroidRageConstants.squareInput(turnSpeed);
        }

        // Apply deadzone
        if (Math.abs(xSpeed) < DroidRageConstants.Gamepad.DRIVER_STICK_DEADZONE) xSpeed = 0;
        if (Math.abs(ySpeed) < DroidRageConstants.Gamepad.DRIVER_STICK_DEADZONE) ySpeed = 0;
        if (Math.abs(turnSpeed) < DroidRageConstants.Gamepad.DRIVER_STICK_DEADZONE) turnSpeed = 0;

        // if(Math.abs(xSpeed) < DroidRageConstants.Gamepad.DRIVER_STICK_DEADZONE &&
        //     Math.abs(ySpeed) < DroidRageConstants.Gamepad.DRIVER_STICK_DEADZONE &&
        //     Math.abs(turnSpeed) < DroidRageConstants.Gamepad.DRIVER_STICK_DEADZONE){
        //     drive.setControl(brakeRequest);
        // }

        // Smooth driving and apply speed
        xSpeed = 
            (xSpeed *
            SwerveConfig.ATTAINABLE_MAX_SPEED.in(MetersPerSecond)) * 
            drive.getTranslationalSpeed();
        ySpeed = 
            (ySpeed *
            SwerveConfig.ATTAINABLE_MAX_SPEED.in(MetersPerSecond)) *
            drive.getTranslationalSpeed();
        turnSpeed = 
            turnSpeed *
            SwerveConfig.ATTAINABLE_MAX_SPEED_ANG.in(RadiansPerSecond) * 
            drive.getAngularSpeed();

        ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, turnSpeed);

        if (noInput.getAsBoolean()) {
            drive.stopDriveMotors();
            return;
        }

        drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, drive.getRotation()));
        
        // if (DriveOptions.IS_FIELD_ORIENTED.get()) {
        //     drive.setControl(
        //         fieldCentricRequest
        //             .withVelocityX(xSpeed)
        //             .withVelocityY(ySpeed)
        //             .withRotationalRate(turnSpeed)
        //     );
        // } else {
        //     drive.setControl(
        //         robotCentricRequest
        //             .withVelocityX(xSpeed)
        //             .withVelocityY(ySpeed)
        //             .withRotationalRate(turnSpeed)
        //     );
        // }
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
        // drive.setControl(brakeRequest);
        // drive.stopWithX();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
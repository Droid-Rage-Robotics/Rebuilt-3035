package frc.robot.commands.manual;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.DroidRageConstants;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.drive.SwerveConfig;
import frc.robot.subsystems.drive.SwerveConfig.Speed;
import frc.robot.subsystems.drive.DriveConstants.DriveOptions;

public class SwerveDriveTeleop extends Command {
    private final SwerveDrive drive;

    private final SwerveRequest.FieldCentric fieldCentricRequest = new SwerveRequest.FieldCentric();
    private final SwerveRequest.RobotCentric robotCentricRequest = new SwerveRequest.RobotCentric();
    private final SwerveRequest.SwerveDriveBrake brakeRequest = new SwerveRequest.SwerveDriveBrake();
    
    private final Supplier<Double> x, y, turn;
    private volatile double xSpeed, ySpeed, turnSpeed;
    private static final PIDController antiTipY = 
        new PIDController(0.006, 0, 0.0005);
    private static final PIDController antiTipX = 
        new PIDController(0.006, 0, 0.0005);

    public SwerveDriveTeleop(SwerveDrive drive, CommandXboxController driver) {
        this.drive = drive;
        this.x = driver::getLeftX;
        this.y = driver::getLeftY;
        this.turn = driver::getRightX;
        antiTipX.setTolerance(2);
        antiTipY.setTolerance(2);

        driver.rightBumper().whileTrue(drive.setSpeed(Speed.SLOW))
            .whileFalse(drive.setSpeed(Speed.NORMAL));
        
        // driver.rightBumper().whileTrue(drive.setSpeed(Speed.SUPER_SLOW))
            // .whileFalse(drive.setSpeed(Speed.SLOW));

        driver.b().onTrue(drive.runOnce(drive::seedFieldCentric));

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

        // // Apply Anti-Tip
        // double xTilt = drive.getRoll(); //Is this Roll or pitch
        // double yTilt = drive.getPitch();// Is this Roll or pitch

        // if(drive.getTippingState()==TippingState.ANTI_TIP) {//Need to take into account on the direction of the tip
        //     if (Math.abs(xTilt) > 10)
        //         xSpeed = -antiTipX.calculate(xTilt, 0);
        //     if (Math.abs(yTilt) >10)
        //         ySpeed = -antiTipY.calculate(yTilt, 0);
        // }

        // Apply deadzone
        if (Math.abs(xSpeed) < DroidRageConstants.Gamepad.DRIVER_STICK_DEADZONE) xSpeed = 0;
        if (Math.abs(ySpeed) < DroidRageConstants.Gamepad.DRIVER_STICK_DEADZONE) ySpeed = 0;
        if (Math.abs(turnSpeed) < DroidRageConstants.Gamepad.DRIVER_STICK_DEADZONE) turnSpeed = 0;

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

        if (DriveOptions.IS_FIELD_ORIENTED.get()) {
            drive.setControl(
                fieldCentricRequest
                    .withVelocityX(xSpeed)
                    .withVelocityY(ySpeed)
                    .withRotationalRate(turnSpeed)
            );
        } else {
            drive.setControl(
                robotCentricRequest
                    .withVelocityX(xSpeed)
                    .withVelocityY(ySpeed)
                    .withRotationalRate(turnSpeed)
            );
        }
    }

    @Override
    public void end(boolean interrupted) {
        drive.setControl(brakeRequest);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

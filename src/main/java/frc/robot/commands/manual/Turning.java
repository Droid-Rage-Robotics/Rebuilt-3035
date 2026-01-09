package frc.robot.commands.manual;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.DroidRageConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorValue;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.drive.SwerveDrive.TippingState;
import frc.robot.subsystems.drive.SwerveDriveConstants;
import frc.robot.subsystems.drive.SwerveDriveConstants.DriveOptions;
import frc.robot.subsystems.drive.SwerveDriveConstants.Speed;
import frc.utility.ControllerUtils;
import frc.robot.subsystems.drive.SwerveModule;

public class Turning extends Command {
    private final SwerveDrive drive;
    private final Elevator elevator;
    private final CommandXboxController driver;
    private final Supplier<Double> x, y;
    private volatile double xSpeed, ySpeed, turnSpeed;
    private double rightStickDeg;
    private Rotation2d heading;
    private static final PIDController antiTipY = new PIDController(0.006, 0, 0.0005);
    private static final PIDController antiTipX = new PIDController(0.006, 0, 0.0005);
    private static final PIDController turnController = new PIDController(0.025, 0, 0.00005);

    private final double TURN_CONTROLLER_DEADZONE = 0.05;

    // private SlewRateLimiter xLimiter = new
    // SlewRateLimiter(SwerveDriveConstants.SwerveDriveConfig.MAX_ACCELERATION_UNITS_PER_SECOND.getValue());
    // private SlewRateLimiter yLimiter = new
    // SlewRateLimiter(SwerveDriveConstants.SwerveDriveConfig.MAX_ACCELERATION_UNITS_PER_SECOND.getValue());

    public Turning(SwerveDrive drive, CommandXboxController driver, Elevator elevator) {
        this.drive = drive;
        this.elevator=elevator;
        this.driver=driver;
        this.x = driver::getLeftX;
        this.y = driver::getLeftY;
        antiTipX.setTolerance(2);
        antiTipY.setTolerance(2);

        driver.rightBumper().whileTrue(drive.setSpeed(Speed.SUPER_SLOW))
                .whileFalse(drive.setSpeed(Speed.SLOW));
        // driver.rightBumper().whileTrue(drive.setSpeed(Speed.SLOW))
        // .whileFalse(drive.setSpeed(Speed.NORMAL));

        driver.b().onTrue(new SequentialCommandGroup(
            drive.setYawCommand(0),
            new InstantCommand(()->rightStickDeg=0)
        ));

        if (elevator.getPosition() >= ElevatorValue.L3.getHeight()) {
            drive.setSpeed(Speed.SLOW);
        }

        SmartDashboard.putData("Drive/Turn Goal", turnGoal);

        turnController.enableContinuousInput(0, 360);

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        rightStickDeg=drive.getHeadingCW();
    }

    @Override
    public void execute() {
        xSpeed = -y.get(); // Forward
        ySpeed = -x.get(); // Strafe

        // Square inputs
        if (DriveOptions.IS_SQUARED_INPUTS.get()) {
            xSpeed = DroidRageConstants.squareInput(xSpeed);
            ySpeed = DroidRageConstants.squareInput(ySpeed);
            // turnSpeed = DroidRageConstants.squareInput(turnSpeed);
        }

        if (!(Math.abs(driver.getRightX())<TURN_CONTROLLER_DEADZONE)||!(Math.abs(driver.getRightY())<TURN_CONTROLLER_DEADZONE)) {
            rightStickDeg = ControllerUtils.getRightStickDeg(driver);
            
        }

        turnSpeed = turnController.calculate(drive.getHeading(), -rightStickDeg); // needs to be negative because calculations are CCW+

        // Apply Field Oriented 
        if (DriveOptions.IS_FIELD_ORIENTED.get()) {
            double modifiedXSpeed = xSpeed;
            double modifiedYSpeed = ySpeed;

            heading = drive.getRotation2d();

            modifiedXSpeed = xSpeed * heading.getCos() + ySpeed * heading.getSin();
            modifiedYSpeed = -xSpeed * heading.getSin() + ySpeed * heading.getCos();

            xSpeed = modifiedXSpeed;
            ySpeed = modifiedYSpeed;
        }

        // Apply Anti-Tip
        double xTilt = drive.getRoll(); // Is this Roll or pitch
        double yTilt = drive.getPitch();// Is this Roll or pitch

        if (drive.getTippingState() == TippingState.ANTI_TIP) {// Need to take into account on the direction of the tip
            if (Math.abs(xTilt) > 10)
                xSpeed = -antiTipX.calculate(xTilt, 0);
            if (Math.abs(yTilt) > 10)
                ySpeed = -antiTipY.calculate(yTilt, 0);
        }

        // Apply deadzone
        if (Math.abs(xSpeed) < DroidRageConstants.Gamepad.DRIVER_STICK_DEADZONE)
            xSpeed = 0;
        if (Math.abs(ySpeed) < DroidRageConstants.Gamepad.DRIVER_STICK_DEADZONE)
            ySpeed = 0;
        // if (Math.abs(turnSpeed) < DroidRageConstants.Gamepad.DRIVER_STICK_DEADZONE)
        //     turnSpeed = 0;

        double translationalSpeed;

        if (elevator.getPosition() >= (0.15 * Elevator.Constants.MAX_HEIGHT)) {
            translationalSpeed = 1.0 - (elevator.getPosition() / Elevator.Constants.MAX_HEIGHT) * 0.95;

            translationalSpeed = MathUtil.clamp(translationalSpeed, 0.01, drive.getTranslationalSpeed());
        }
        else {
            translationalSpeed = drive.getTranslationalSpeed();
        }

        // Smooth driving and apply speed
        xSpeed = 
            (xSpeed *
            SwerveModule.Constants.PHYSICAL_MAX_SPEED_METERS_PER_SECOND) * 
            translationalSpeed;
        ySpeed = 
            (ySpeed *
            SwerveModule.Constants.PHYSICAL_MAX_SPEED_METERS_PER_SECOND) *
            translationalSpeed;
        turnSpeed = 
            turnSpeed *
            SwerveDriveConstants.SwerveDriveConfig.PHYSICAL_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND.getValue() * 
            drive.getAngularSpeed();

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turnSpeed);

        SwerveModuleState[] states = SwerveDrive.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        drive.setModuleStates(states);
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    public final Sendable turnGoal = new Sendable() {
        @Override
        public void initSendable(SendableBuilder builder) {
            builder.setSmartDashboardType("Gyro");
            builder.addDoubleProperty("Value", () -> rightStickDeg, null); // doesn't need negative because we observe CW+
        }
    };
}

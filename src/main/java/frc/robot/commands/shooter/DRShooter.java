package frc.robot.commands.shooter;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.DroidRageConstants;
import frc.robot.DroidRageConstants.FieldConstants;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.shooter.HubShooterMath;
import frc.robot.subsystems.shooter.Shooter;

public class DRShooter extends Command{

    private final Shooter shooter;    
    private final Translation3d hubPose;
    private final SwerveDrive drive;
    // private final Supplier<SwerveDriveState> robot;

     private static final InterpolatingDoubleTreeMap hoodMap =
        new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap flywheelSpeedMap =
        new InterpolatingDoubleTreeMap();
    {
        flywheelSpeedMap.put(1.1, 1.1);
        flywheelSpeedMap.put(1.1, 1.1);
        flywheelSpeedMap.put(1.1, 1.1);
        flywheelSpeedMap.put(1.1, 1.1);
        flywheelSpeedMap.put(1.1, 1.1);

        // flywheelSpeedMap.put(0.96, 150.0);
        // flywheelSpeedMap.put(1.16, 155.0);
        // flywheelSpeedMap.put(1.58, 160.0);
        // flywheelSpeedMap.put(2.07, 165.0);
        // flywheelSpeedMap.put(2.37, 170.0);
        // flywheelSpeedMap.put(2.47, 170.0);
        // flywheelSpeedMap.put(2.70, 170.0);
        // flywheelSpeedMap.put(2.94, 175.0);
        // flywheelSpeedMap.put(3.48, 175.0);
        // flywheelSpeedMap.put(3.92, 180.0);
        // flywheelSpeedMap.put(4.35, 185.0);
        // flywheelSpeedMap.put(4.84, 190.0);

        hoodMap.put(1., 1.0);
        hoodMap.put(1., 1.0);
        hoodMap.put(1., 1.0);
        hoodMap.put(1., 1.0);




    }

    public DRShooter(SwerveDrive drive, Shooter shooter) {
        this.shooter = shooter;
        // this.robot = drive::getState;
        this.drive = drive;

        this.hubPose = DroidRageConstants.alliance == Alliance.Red 
            ? FieldConstants.HUB_RED 
            : FieldConstants.HUB_BLUE;


        addRequirements(
            shooter.getHood(),
            shooter.getShooterWheel(),
            shooter.getTurret());
    }

    @Override
    public void initialize(){

    }

    @Override 
    public void execute(){
        double distanceRobotToHub = 
        shooter.getTurret().setGoalAngle(HubShooterMath.calculateTurretAngle(drive.getState().Pose, hubPose));
        shooter.getHood().setGoalAngle(hoodMap.get(null));
    }
    
}

package frc.robot.commands.autos;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;



import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.drive.SwerveConfig;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Vision;

public final class Autos {
    public static Command rightNeutralOutpost(SwerveDrive drive, Intake intake, Indexer indexer, Kicker kicker, Shooter shooter, Vision vision) {
        return new SequentialCommandGroup(
            PathPlannerFollow.create(drive, "rightNeutralOutpost")
            // PathPlannerFollow.create(drive, "test")
                .setMaxVelocity(8)
                .setAcceleration(8)
                .build()
        );
    }
    public static Command rightNeutralOutpostSh(SwerveDrive drive, Intake intake, Indexer indexer, Kicker kicker, Shooter shooter, Vision vision) {
        return new SequentialCommandGroup(
            PathPlannerFollow.create(drive, "rightNeutralOutpostSh")
            // PathPlannerFollow.create(drive, "test")
                .setMaxVelocity(8)
                .setAcceleration(8)
                .build()
        );
    }

    public static Command centerHubDepot(SwerveDrive drive, Intake intake, Indexer indexer, Kicker kicker, Shooter shooter, Vision vision) {
        return new SequentialCommandGroup(
            PathPlannerFollow.create(drive, "centerHubDepot")
                .setMaxVelocity(8)
                .setAcceleration(8)
                .build()
        );
    }

    

    // public static Command rightNeutralOutpostDouble(SwerveDrive drive, Intake intake, Indexer indexer, Kicker kicker, Shooter shooter, Vision vision) {
    //     return new SequentialCommandGroup(
    //         PathPlannerFollow.create(drive, "rightNeutralOutpost")
    //             .setMaxVelocity(8)
    //             .setAcceleration(8)
    //             .build(),
    //         // shooter.setShooterTargetCommand(ShooterValue.AUTO_SHOOT_TRENCH_RIGHT),
    //         PathPlannerFollow.create(drive, "rightNeutralOutpost2")
    //             .setMaxVelocity(8)
    //             .setAcceleration(8)
    //             .build()
    //     );
    // }

    public static Command leftNeutralDepot(SwerveDrive drive, Intake intake, Indexer indexer, Kicker kicker, Shooter shooter, Vision vision) {
        return new SequentialCommandGroup(
            PathPlannerFollow.create(drive, "leftNeutralDepot")
                .setMaxVelocity(8)
                .setAcceleration(8)
                .build()
        );
    }
    
    public static Command leftNeutralDepotSh(SwerveDrive drive, Intake intake, Indexer indexer, Kicker kicker, Shooter shooter, Vision vision) {
        return new SequentialCommandGroup(
            PathPlannerFollow.create(drive, "leftNeutralDepotSh")
                .setMaxVelocity(8)
                .setAcceleration(8)
                .build()
        );
    }

    // public static Command leftNeutralDepotDouble(SwerveDrive drive, Intake intake, Indexer indexer, Kicker kicker, Shooter shooter, Vision vision) {
    //     return new SequentialCommandGroup(
    //         PathPlannerFollow.create(drive, "leftNeutralDepot")
    //             .setMaxVelocity(8)
    //             .setAcceleration(8)
    //             .build(),
    //         PathPlannerFollow.create(drive, "leftNeutralDepot2")
    //             .setMaxVelocity(8)
    //             .setAcceleration(8)
    //             .build()
    //     );
    // }

    public static Command testPathFindingtoPath(){
        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        // Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
        //     PathPlannerPath.fromPathFile("ForwardTest"),
        //     SwerveConfig.pathConstraint);
        // return pathfindingCommand;
        return new InstantCommand();
    }
    public static Command testPathFinding(SwerveDrive drive){
        return new SequentialCommandGroup(
            PathPlannerFollow.create(drive, "ForwardTest")
                .setMaxVelocity(1)
                .setAcceleration(1)
                .build(),
            AutoBuilder.pathfindToPose(
                new Pose2d(Meters.of(3), Meters.of(1), new Rotation2d(Degrees.of(90))), 
                SwerveConfig.pathConstraint),
            AutoBuilder.pathfindToPose(
                new Pose2d(Meters.of(1), Meters.of(1.5), new Rotation2d(Degrees.of(180))), 
                SwerveConfig.pathConstraint)
        );

        // return new SequentialCommandGroup(
        //     PathPlannerFollow.create(drive, "ForwardTest")
        //         .setMaxVelocity(1)
        //         .setAcceleration(1)
        //         .build(),
        //     AutoBuilder.pathfindToPose(
        //         new Pose2d(Meters.of(13), Meters.of(7), new Rotation2d(Degrees.of(270))), 
        //         SwerveConfig.pathConstraint)
        // );
        
    }
}
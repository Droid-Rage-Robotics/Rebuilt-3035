package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.shooter.Kicker;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Vision;

public final class Autos {
   
    public static Command crossNeutralDepot(SwerveDrive drive, Shooter shooter, Indexer indexer, Kicker kicker, Vision vision) {//Top Red/Bottom Blue
        return new SequentialCommandGroup(
            PathPlannerFollow.create(drive, "crossNeutralDepot")
                .setMaxVelocity(6)
                .setAcceleration(6)
                .build()
        );
    }
    public static Command crossNeuralOutpost(SwerveDrive drive, Shooter shooter, Indexer indexer, Kicker kicker, Vision vision) {//Top Red/Bottom Blue
        return new SequentialCommandGroup(
            PathPlannerFollow.create(drive, "crossNeutralOutpost")
                .setMaxVelocity(6)
                .setAcceleration(6)
                .build()
        );
    }
    public static Command middle(SwerveDrive drive, Shooter shooter, Indexer indexer, Kicker kicker, Vision vision) {//Top Red/Bottom Blue
        return new SequentialCommandGroup(
            PathPlannerFollow.create(drive, "middle")
                .setMaxVelocity(6)
                .setAcceleration(6)
                .build()
        );
    }
    
    public static Command neutralDepot(SwerveDrive drive, Shooter shooter, Indexer indexer, Kicker kicker, Vision vision) {//Top Red/Bottom Blue
        return new SequentialCommandGroup(
            PathPlannerFollow.create(drive, "neutralDepot")
                .setMaxVelocity(6)
                .setAcceleration(6)
                .build()
        );
    }
    public static Command neutralOutpost(SwerveDrive drive, Shooter shooter, Indexer indexer, Kicker kicker, Vision vision) {//Top Red/Bottom Blue
        return new SequentialCommandGroup(
            // new WaitCommand(4),
            PathPlannerFollow.create(drive, "neutralOutpost")
                .setMaxVelocity(6)
                .setAcceleration(6)
                .build()
        );
    }
    public static Command autoName(SwerveDrive drive, Shooter shooter, Indexer indexer, Kicker kicker, Vision vision) {//Top Red/Bottom Blue
        return new SequentialCommandGroup(
            PathPlannerFollow.create(drive, "auto name")
                .setMaxVelocity(6)
                .setAcceleration(6)
                .build()
           
        );
    }
    // public static Command autoName(SwerveDrive drive, Shooter shooter, Indexer indexer, Kicker kicker, Vision vision) {//Top Red/Bottom Blue
    //     return new SequentialCommandGroup(
    //         PathPlannerFollow.create(drive, "auto name")
    //             .setMaxVelocity(6)
    //             .setAcceleration(6)
    //             .build()
    //     );
    // }
    
    // public static Command autoName(SwerveDrive drive, Shooter shooter, Indexer indexer, Kicker kicker,  Vision vision) {//Top Red/Bottom Blue
    //     return new SequentialCommandGroup(
    //         PathPlannerFollow.create(drive, "auto name")
    //             .setMaxVelocity(6)
    //             .setAcceleration(6)
    //             .build()
    //     );
    // }
    // public static Command autoName(SwerveDrive drive, Shooter shooter, Indexer indexer, Kicker kicker, Vision vision) {//Top Red/Bottom Blue
    //     return new SequentialCommandGroup(
    //         PathPlannerFollow.create(drive, "auto name")
    //             .setMaxVelocity(6)
    //             .setAcceleration(6)
    //             .build()
    //     );
    // }
    // public static Command autoName(SwerveDrive drive, Shooter shooter, Indexer indexer, Kicker kicker, Vision vision) {//Top Red/Bottom Blue
    //     return new SequentialCommandGroup(
    //         PathPlannerFollow.create(drive, "auto name")
    //             .setMaxVelocity(6)
    //             .setAcceleration(6)
    //             .build()
    //     );
    // }
   
    // public static Command autoName(SwerveDrive drive, Shooter shooter, Indexer indexer, Kicker kicker, Vision vision) {//Top Red/Bottom Blue
    //     return new SequentialCommandGroup(
    //         PathPlannerFollow.create(drive, "auto name")
    //             .setMaxVelocity(6)
    //             .setAcceleration(6)
    //             .build()
    //     );
    // }
}
package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.drive.SwerveDrive;
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
    public static Command NeutralToTrenchDepot(SwerveDrive drive, Shooter shooter, Indexer indexer, Kicker kicker, Vision vision) {//Top Red/Bottom Blue
        return new SequentialCommandGroup(
            PathPlannerFollow.create(drive, "NeutralToTrenchDepot")
                .setMaxVelocity(6)
                .setAcceleration(6)
                .build()
        );
    }
    public static Command TrenchToNeutralDepot(SwerveDrive drive, Shooter shooter, Indexer indexer, Kicker kicker, Vision vision) {//Top Red/Bottom Blue
        return new SequentialCommandGroup(
            PathPlannerFollow.create(drive, "TrenchToNeutralDepot")
                .setMaxVelocity(6)
                .setAcceleration(6)
                .build()
        );
    }
    
    public static Command TrenchToTowerDepot(SwerveDrive drive, Shooter shooter, Indexer indexer, Kicker kicker, Vision vision) {//Top Red/Bottom Blue
        return new SequentialCommandGroup(
            PathPlannerFollow.create(drive, "TrenchToTowerDepot")
                .setMaxVelocity(6)
                .setAcceleration(6)
                .build()
        );
    }
    public static Command DepotToTower(SwerveDrive drive, Shooter shooter, Indexer indexer, Kicker kicker, Vision vision) {//Top Red/Bottom Blue
        return new SequentialCommandGroup(
            // new WaitCommand(4),
            PathPlannerFollow.create(drive, "DepotToTower")
                .setMaxVelocity(6)
                .setAcceleration(6)
                .build()
        );
    }
    public static Command HPSToDepot(SwerveDrive drive, Shooter shooter, Indexer indexer, Kicker kicker, Vision vision) {//Top Red/Bottom Blue
        return new SequentialCommandGroup(
            PathPlannerFollow.create(drive, "HPSToDepot")
                .setMaxVelocity(6)
                .setAcceleration(6)
                .build()
           
        );
    }
    public static Command hubToHPS(SwerveDrive drive, Shooter shooter, Indexer indexer, Kicker kicker, Vision vision) {//Top Red/Bottom Blue
        return new SequentialCommandGroup(
            PathPlannerFollow.create(drive, "hubToHPS")
                .setMaxVelocity(6)
                .setAcceleration(6)
                .build()
        );
    }
    
    public static Command crossNeutralOutpost(SwerveDrive drive, Shooter shooter, Indexer indexer, Kicker kicker,  Vision vision) {//Top Red/Bottom Blue
        return new SequentialCommandGroup(
            PathPlannerFollow.create(drive, "crossNeutralOutpost")
                .setMaxVelocity(6)
                .setAcceleration(6)
                .build()
        );
    }
    public static Command NeutralToTrenchOutpost(SwerveDrive drive, Shooter shooter, Indexer indexer, Kicker kicker, Vision vision) {//Top Red/Bottom Blue
        return new SequentialCommandGroup(
            PathPlannerFollow.create(drive, "NeutralToTrenchOutpost")
                .setMaxVelocity(6)
                .setAcceleration(6)
                .build()
        );
    }
    public static Command TrenchToNeutralOutpost(SwerveDrive drive, Shooter shooter, Indexer indexer, Kicker kicker, Vision vision) {//Top Red/Bottom Blue
        return new SequentialCommandGroup(
            PathPlannerFollow.create(drive, "TrenchToNeutralOutpost")
                .setMaxVelocity(6)
                .setAcceleration(6)
                .build()
        );
    }
   
    public static Command TrenchToTowerOutpost(SwerveDrive drive, Shooter shooter, Indexer indexer, Kicker kicker, Vision vision) {//Top Red/Bottom Blue
        return new SequentialCommandGroup(
            PathPlannerFollow.create(drive, "TrenchToTowerOutpost")
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
}
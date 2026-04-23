package frc.robot.commands.autos;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.commands.PathPlannerAuto;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.DroidRageConstants;
import frc.robot.commands.shooter.AutoDRShooter;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Kicker.KickerValue;
import frc.robot.subsystems.drive.SwerveConfig;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.Indexer.IndexerValue;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeValue;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterValue;
import frc.robot.subsystems.vision.Vision;
import frc.utility.DRAreaManager;
import frc.utility.DRAreaManager.Zone;

public final class Autos {
    // public static Command rightNeutralOutpost(SwerveDrive drive, Intake intake, Indexer indexer, Kicker kicker, Shooter shooter, Vision vision) {
    //     return new SequentialCommandGroup(
    //         PathPlannerFollow.create(drive, "rightNeutralOutpost")
    //         // PathPlannerFollow.create(drive, "test")
    //             .setMaxVelocity(8)
    //             .setAcceleration(8)
    //             .build()
    //     );
    // }
    // public static Command rightNeutralOutpostSh(SwerveDrive drive, Intake intake, Indexer indexer, Kicker kicker, Shooter shooter, Vision vision) {
    //     return new SequentialCommandGroup(
    //         PathPlannerFollow.create(drive, "rightNeutralOutpostSh")
    //         // PathPlannerFollow.create(drive, "test")
    //             .setMaxVelocity(8)
    //             .setAcceleration(8)
    //             .build()
    //     );
    // }

    // public static Command centerHubDepot(SwerveDrive drive, Intake intake, Indexer indexer, Kicker kicker, Shooter shooter, Vision vision) {
    //     return new SequentialCommandGroup(
    //         PathPlannerFollow.create(drive, "centerHubDepot")
    //             .setMaxVelocity(8)
    //             .setAcceleration(8)
    //             .build()
    //     );
    // }

    // public static Command leftNeutralDepot(SwerveDrive drive, Intake intake, Indexer indexer, Kicker kicker, Shooter shooter, Vision vision) {
    //     return new SequentialCommandGroup(
    //         PathPlannerFollow.create(drive, "leftNeutralDepot")
    //             .setMaxVelocity(8)
    //             .setAcceleration(8)
    //             .build()
    //     );
    // }
    
    public static Command leftNeutralDepotSh(SwerveDrive drive, Intake intake, Indexer indexer, Kicker kicker, Shooter shooter, Vision vision) {
        return new SequentialCommandGroup(
            PathPlannerFollow.create(drive, "leftNeutralDepotSh")
                .setMaxVelocity(8)
                .setAcceleration(8).build()
        );
    }

            // new ConditionalCommand(
            //     new AutoDRShooter(drive, shooter)
            //         .repeatedly()
            //         .alongWith(AutoCommands.autoIndexerWiggleIntake(intake), AutoCommands.index(indexer, kicker)
            //     ),
            //     new SequentialCommandGroup(
            //         PathPlannerPathFindingFollow.create(drive, AutoChooser.getStartingPoseFromPath("NeutralToTrenchRight"))
            //             .build(),
            //         PathPlannerPathFollow.create(drive, "NeutralToTrenchRight")
            //             .withMaxVelocity(13)
            //             .withMaxAcceleration(13)
            //             .build(),
            //         new AutoDRShooter(drive, shooter)
            //             .repeatedly()
            //             .alongWith(AutoCommands.autoIndexerWiggleIntake(intake), AutoCommands.index(indexer, kicker))
            //     ),
            //     ()->DRAreaManager.getCurrentZone()==Zone.ALLIANCE_ZONE
            // )


    public static Command newRightOutpost(String depth, SwerveDrive drive, Intake intake, Indexer indexer, Kicker kicker, Shooter shooter, Vision vision) {
        return new SequentialCommandGroup (
            new ParallelCommandGroup (
                PathPlannerPathFollow.create(drive, "TrenchToNeutralRight" + depth, true)
                    .withMaxVelocity(11)
                    .withMaxAcceleration(11)
                    .build(),

                new SequentialCommandGroup(
                    AutoCommands.startPivotCommand(intake),
                    new WaitCommand(0.25),
                    intake.getIntakeWheel().setTargetVelocityCommand(IntakeValue.WheelVelocity.INTAKE),
                    new WaitCommand(.5),
                    shooter.setTurretCommand(Shooter.ShooterValue.AUTO_SHOOT_TRENCH_RIGHT_FAR_ONE)
                )
            ),

            PathPlannerPathFollow.create(drive, "NeutralToTrenchRight" + depth)
                .withMaxVelocity(13)
                .withMaxAcceleration(13)
                .build(),
            new SequentialCommandGroup(
                shooter.setShooterTargetCommand(Shooter.ShooterValue.AUTO_SHOOT_TRENCH_RIGHT_FAR_ONE),
                new WaitCommand(.5),
                new ParallelCommandGroup(
                // AutoCommands.autoIndexerWiggleIntake(intake), 
                AutoCommands.index(indexer, kicker),
                new WaitCommand(8)
                )
            ),
            AutoCommands.resetBot(shooter, indexer, kicker, intake),

            new ParallelCommandGroup (
                PathPlannerPathFollow.create(drive, "TrenchToNeutralRightSec")
                    .withMaxVelocity(13)
                    .withMaxAcceleration(13)
                    .build(),

                new SequentialCommandGroup(
                    AutoCommands.startPivotCommand(intake),
                    new WaitCommand(0.25),
                    intake.getIntakeWheel().setTargetVelocityCommand(IntakeValue.WheelVelocity.INTAKE),
                    new WaitCommand(.5),
                    shooter.setTurretCommand(Shooter.ShooterValue.AUTO_SHOOT_TRENCH_RIGHT_FAR_TWO)
                )
            ),

            PathPlannerPathFollow.create(drive, "NeutralToTrenchRightSec")
                .withMaxVelocity(13)
                .withMaxAcceleration(13)
                .build(),

            new SequentialCommandGroup(
                shooter.setShooterTargetCommand(Shooter.ShooterValue.AUTO_SHOOT_TRENCH_RIGHT_FAR_TWO),
                new WaitCommand(.5),
                new ParallelCommandGroup(
                // AutoCommands.autoIndexerWiggleIntake(intake), 
                AutoCommands.index(indexer, kicker)
                )
            )
        );
    }

    public static Command newLeftDepot(String depth, SwerveDrive drive, Intake intake, Indexer indexer, Kicker kicker, Shooter shooter, Vision vision) {
        return new SequentialCommandGroup (
            new ParallelCommandGroup (
                PathPlannerPathFollow.create(drive, "TrenchToNeutralLeft" + depth, true)
                    .withMaxVelocity(11)
                    .withMaxAcceleration(11)
                    .build(),

                new SequentialCommandGroup(
                    AutoCommands.startPivotCommand(intake),
                    new WaitCommand(0.25),
                    intake.getIntakeWheel().setTargetVelocityCommand(IntakeValue.WheelVelocity.INTAKE),
                    new WaitCommand(.5),
                    shooter.setTurretCommand(Shooter.ShooterValue.AUTO_SHOOT_TRENCH_LEFT_FAR_ONE)
                )
            ),

            PathPlannerPathFollow.create(drive, "NeutralToTrenchLeft" + depth)
                .withMaxVelocity(13)
                .withMaxAcceleration(13)
                .build(),
            new SequentialCommandGroup(
                shooter.setShooterTargetCommand(Shooter.ShooterValue.AUTO_SHOOT_TRENCH_LEFT_FAR_ONE),
                new WaitCommand(.5),
                new ParallelCommandGroup(
                // AutoCommands.autoIndexerWiggleIntake(intake), 
                AutoCommands.index(indexer, kicker),
                new WaitCommand(8)
                )
            ),
            AutoCommands.resetBot(shooter, indexer, kicker, intake),

            new ParallelCommandGroup (
                PathPlannerPathFollow.create(drive, "TrenchToNeutralLeftSec")
                    .withMaxVelocity(13)
                    .withMaxAcceleration(13)
                    .build(),

                new SequentialCommandGroup(
                    AutoCommands.startPivotCommand(intake),
                    new WaitCommand(0.25),
                    intake.getIntakeWheel().setTargetVelocityCommand(IntakeValue.WheelVelocity.INTAKE),
                    new WaitCommand(.5),
                    shooter.setTurretCommand(Shooter.ShooterValue.AUTO_SHOOT_TRENCH_LEFT_FAR_TWO)
                )
            ),

            PathPlannerPathFollow.create(drive, "NeutralToTrenchLeftSec")
                .withMaxVelocity(13)
                .withMaxAcceleration(13)
                .build(),

            new ParallelCommandGroup(
                shooter.setShooterTargetCommand(Shooter.ShooterValue.AUTO_SHOOT_TRENCH_LEFT_FAR_TWO),
                new WaitCommand(.5),
                new ParallelCommandGroup(
                // AutoCommands.autoIndexerWiggleIntake(intake), 
                AutoCommands.index(indexer, kicker)
                )
            )
        );
    }

    public static Command rightNeutralSwoop(SwerveDrive drive, Intake intake, Indexer indexer, Kicker kicker, Shooter shooter, Vision vision) {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                PathPlannerPathFollow.create(drive, "TrenchToNeutralSwoopRight", true)
                    .build(),

                new SequentialCommandGroup(
                    AutoCommands.startPivotCommand(intake),
                    new WaitCommand(0.25),
                    intake.getIntakeWheel().setTargetVelocityCommand(IntakeValue.WheelVelocity.INTAKE)
                )
            ),

            new ParallelCommandGroup(
                PathPlannerPathFollow.create(drive, "NeutralSwoopToTrenchRight")
                    .build(),

                intake.getIntakeWheel().setTargetVelocityCommand(IntakeValue.WheelVelocity.STOP)
            ),
            
            new AutoDRShooter(drive, shooter)
                .repeatedly()
                .alongWith(AutoCommands.index(indexer, kicker))
            // new ParallelCommandGroup(
            //     shooter.setShooterTargetCommand(ShooterValue.AUTO_SHOOT_TRENCH_RIGHT_FAR),
            //     AutoCommands.autoIndexerWiggleIntake(intake),
            //     AutoCommands.index(indexer, kicker)
            // )
        );
    }

    public static Command leftNeutralSwoop(SwerveDrive drive, Intake intake, Indexer indexer, Kicker kicker, Shooter shooter, Vision vision) {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                PathPlannerPathFollow.create(drive, "TrenchToNeutralSwoopLeft", true)
                    .build(),

                new SequentialCommandGroup(
                    AutoCommands.startPivotCommand(intake),
                    new WaitCommand(0.25),
                    intake.getIntakeWheel().setTargetVelocityCommand(IntakeValue.WheelVelocity.INTAKE)
                )
            ),

            new ParallelCommandGroup(
                PathPlannerPathFollow.create(drive, "NeutralSwoopToTrenchLeft")
                    .build(),

                intake.getIntakeWheel().setTargetVelocityCommand(IntakeValue.WheelVelocity.STOP)
            ),

            new AutoDRShooter(drive, shooter)
                .repeatedly()
                .alongWith(AutoCommands.index(indexer, kicker))
            // new ParallelCommandGroup(
            //     shooter.setShooterTargetCommand(ShooterValue.AUTO_SHOOT_TRENCH_LEFT_FAR),
            //     AutoCommands.autoIndexerWiggleIntake(intake),
            //     AutoCommands.index(indexer, kicker)
            // )
        );
    }

    public static Command centerDepot(SwerveDrive drive, Intake intake, Indexer indexer, Kicker kicker, Shooter shooter, Vision vision) {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                PathPlannerPathFollow.create(drive, "HubToDepot", true)
                    .build(),
                new SequentialCommandGroup(
                    // AutoCommands.startPivotCommand(intake),
                    intake.getPivot().setTargetPositionCommand(IntakeValue.PivotAngle.AUTO_DOWN),
                    new WaitCommand(0.25),
                    intake.getIntakeWheel().setTargetVelocityCommand(IntakeValue.WheelVelocity.INTAKE)
                )
            ),
            new WaitCommand(.5),
            intake.getPivot().setTargetPositionCommand(IntakeValue.PivotAngle.DOWN),
            new WaitCommand(.5),

            new ParallelCommandGroup(
                PathPlannerPathFollow.create(drive, "HubToDepot2")
                    .build(),
                new SequentialCommandGroup(
                    new WaitCommand(1.5),
                    shooter.setShooterTargetCommand(Shooter.ShooterValue.AUTO_DEPOT),
                    new WaitCommand(.4),
                    AutoCommands.index(indexer, kicker),
                    new WaitCommand(.5),
                    new ParallelCommandGroup(
                        AutoCommands.autoWiggleIntake(intake)
                    )
                )
            ),
            new WaitCommand(8), //Remove if wiggling indexer
            new ParallelCommandGroup(
                shooter.setShooterTargetCommand(Shooter.ShooterValue.HOLD),
                PathPlannerPathFollow.create(drive, "HubToDepot3")
                    .build()
                
            )
        );
    }

    public static Command bumpTest(int num, SwerveDrive drive, Intake intake, Indexer indexer, Kicker kicker, Shooter shooter, Vision vision) {
        return new SequentialCommandGroup(
            intake.getPivot().setTargetPositionCommand(IntakeValue.PivotAngle.DOWN),
            PathPlannerPathFollow.create(drive, "RightBump" + num, true)
                .build()
        );
    }

    public static Command centerDepotStaright(SwerveDrive drive, Intake intake, Indexer indexer, Kicker kicker, Shooter shooter, Vision vision) {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                PathPlannerPathFollow.create(drive, "center", true)
                    .build()
            ),
            new WaitCommand(1),
            AutoCommands.startPivotCommand(intake),
            // intake.getPivot().setTargetPositionCommand(IntakeValue.PivotAngle.HALF_TWO),

            new SequentialCommandGroup(
                shooter.setShooterTargetCommand(Shooter.ShooterValue.SHORT),
                new WaitCommand(2),
                new ParallelCommandGroup(
                AutoCommands.autoWiggleIntake(intake), 
                AutoCommands.index(indexer, kicker)
                )
            )
        );
    }

    public static Command testPath(SwerveDrive drive, Intake intake, Indexer indexer, Kicker kicker, Shooter shooter, Vision vision) {
        // return new SequentialCommandGroup(
        //     new ParallelCommandGroup (
        //         PathPlannerPathFollow.create(drive, "TrenchToNeutralRightS", true)
        //             .withMaxVelocity(11)
        //             .withMaxAcceleration(11)
        //             .build(),
        //         new AutoDRShooter(drive, shooter)
        //     )
        // );
        return new SequentialCommandGroup (
            new ParallelCommandGroup (
                PathPlannerPathFollow.create(drive, "TrenchToNeutralRightS", true)
                    .withMaxVelocity(6)
                    .withMaxAcceleration(6)
                    .build(),
            ),
            PathPlannerPathFollow.create(drive, "NeutralToTrenchRightS" + depth)
                .withMaxVelocity(6)
                .withMaxAcceleration()
                .build(),
            new AutoDRShooter(drive, shooter),
            new WaitCommand(19),
            AutoCommands.resetBot(shooter, indexer, kicker, intake),
        );
    }

    public static Command newRightOutpost(String depth, SwerveDrive drive, Intake intake, Indexer indexer, Kicker kicker, Shooter shooter, Vision vision) {
        
    }
}
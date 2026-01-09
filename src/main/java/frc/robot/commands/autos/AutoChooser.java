package frc.robot.commands.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.DroidRageConstants;
import frc.robot.commands.TeleopCommands;
import frc.robot.commands.drive.AutoAlign;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.carriage.Carriage;
import frc.robot.subsystems.carriage.Carriage.CarriageIntakeValue;
import frc.robot.subsystems.carriage.Carriage.CarriageValue;
import frc.robot.subsystems.drive.SwerveDriveConstants;
import frc.robot.subsystems.vision.Vision;
import frc.utility.DashboardUtils;
import frc.utility.DashboardUtils.Dashboard;

public class AutoChooser implements Dashboard {
    public static final SendableChooser<Command> autoChooser = new SendableChooser<Command>();

    public AutoChooser(SwerveDrive drive, Elevator elevator, Carriage carriage, Vision vision){
        // NamedCommands.registerCommand("resetPose",
        //     new ResetPoseVision(drive, vision) 
        // );
        NamedCommands.registerCommand("outtakeProcessor",
            new TeleopCommands().runIntakeFor(carriage, CarriageIntakeValue.OUTTAKE_PROCESSOR, .066)
            );
        NamedCommands.registerCommand("shoot",
            new TeleopCommands().runIntakeFor(carriage, CarriageIntakeValue.SHOOT, .066)
        );
        NamedCommands.registerCommand("placeProcessor",
            new ParallelCommandGroup(
                elevator.setTargetPositionCommand(Elevator.ElevatorValue.GROUND),
                carriage.setPositionCommand(CarriageValue.PROCESSOR),
                carriage.setIntakeCommand(CarriageIntakeValue.OUTTAKE)
            )
        );
        NamedCommands.registerCommand("pickLAlgae",
            new AutoCommands().autoAlgaePickUp(elevator, carriage, 
                Elevator.ElevatorValue.ALGAE_LOW, CarriageValue.ALGAE_LOW)
        );

        NamedCommands.registerCommand("pickHAlgae",
            new AutoCommands().autoAlgaePickUp(elevator, carriage, 
                Elevator.ElevatorValue.ALGAE_HIGH, CarriageValue.ALGAE_HIGH)
        );
        NamedCommands.registerCommand("right",
            new InstantCommand(()-> DroidRageConstants.setAlignment(DroidRageConstants.Alignment.RIGHT))
        );
        NamedCommands.registerCommand("left",
            new InstantCommand(()-> DroidRageConstants.setAlignment(DroidRageConstants.Alignment.LEFT))
        );
        NamedCommands.registerCommand("middle",
            new InstantCommand(() -> DroidRageConstants.setAlignment(DroidRageConstants.Alignment.MIDDLE))
        );

        NamedCommands.registerCommand("placeL4",
            // new TeleopCommands().goL4(elevator, carriage)
            new TeleopCommands().autoGoL4(elevator, carriage)
        );
        NamedCommands.registerCommand("outtake", 
            new SequentialCommandGroup(
                new TeleopCommands().runIntakeFor(carriage, CarriageIntakeValue.OUTTAKE, .07)//.066
            )
        );
        NamedCommands.registerCommand("outtakeA", 
            new SequentialCommandGroup(
                new TeleopCommands().runIntakeFor(carriage, CarriageIntakeValue.OUTTAKE_PROCESSOR, .9)//.066
            )
        );
        NamedCommands.registerCommand("finalOuttake",
                new SequentialCommandGroup(
                    carriage.setIntakeCommand(CarriageIntakeValue.OUTTAKE)
                ));
        NamedCommands.registerCommand("align", 
            new SequentialCommandGroup(
                new AutoAlign(drive, vision).withTimeout(1.7)//1
                //1.5 for MIddleAutos
                //2.8 for left AUtos
                //Middle for playoffs
            )
        );
        NamedCommands.registerCommand("align1",
            new SequentialCommandGroup(
                new AutoAlign(drive, vision).withTimeout(1.)
        ));
         NamedCommands.registerCommand("align1.5",
            new SequentialCommandGroup(
                new AutoAlign(drive, vision).withTimeout(1.5)
        ));
        NamedCommands.registerCommand("align2",
            new SequentialCommandGroup(
                new AutoAlign(drive, vision).withTimeout(2)
        ));
        NamedCommands.registerCommand("align3",
            new SequentialCommandGroup(
                new AutoAlign(drive, vision).withTimeout(3)
        ));
        NamedCommands.registerCommand("setLP", 
        // new InstantCommand()
            new InstantCommand(()->vision.setUpLeftVision())
        );
        NamedCommands.registerCommand("setLFP", 
        // new InstantCommand()
            new InstantCommand(()->vision.setUpLeftFrontVision())
        );
        NamedCommands.registerCommand("setRP", 
        // new InstantCommand()
            new InstantCommand(()->vision.setUpLeftVision())
        );
        NamedCommands.registerCommand("setRFP", 
        // new InstantCommand()
            new InstantCommand(()->vision.setUpLeftFrontVision())
        );
        NamedCommands.registerCommand("revert", 
        // new InstantCommand()

            new InstantCommand(()->vision.setUpVision())
        );
        // NamedCommands.registerCommand("placeL3",
        //     new SequentialCommandGroup(
        //         carriage.setPositionCommand(CarriageValue.L3)
        //     )
        // );
        // NamedCommands.registerCommand("placeL2",
        //     new SequentialCommandGroup(
        //         carriage.setPositionCommand(CarriageValue.L2)
        //     )
        // );
        // NamedCommands.registerCommand("placeL1",
        //     new SequentialCommandGroup(
        //         carriage.setPositionCommand(CarriageValue.L1)
        //     )
        // );
        NamedCommands.registerCommand("resetCarriage",
            new SequentialCommandGroup(
                new TeleopCommands().resetHP(elevator, carriage, CarriageValue.INTAKE_HPS),
                new WaitCommand(.5),
                carriage.setIntakeCommand(CarriageIntakeValue.INTAKE)
            )
        );
        NamedCommands.registerCommand("resetBarge",
            new SequentialCommandGroup(
                new TeleopCommands().resetHP(elevator, carriage, CarriageValue.INTAKE_HPS),
                // new TeleopCommands().resetCarriageFromBarge(elevator, carriage),
                carriage.setIntakeCommand(CarriageIntakeValue.STOP)
            )
        );
        NamedCommands.registerCommand("placeBarge",
            new SequentialCommandGroup(
                new TeleopCommands().barge(elevator, carriage)
                // new WaitCommand(.2),//1.9
                // carriage.setIntakeCommand(CarriageIntakeValue.SHOOT)
            )
        );
        NamedCommands.registerCommand("intake",
            new SequentialCommandGroup(
                // carriage.setIntakeCommand(CarriageIntakeValue.INTAKE),
                new WaitCommand(.3)//1
            )   
        );
        NamedCommands.registerCommand("test",
            new SequentialCommandGroup(
                carriage.setIntakeCommand(CarriageIntakeValue.INTAKE)
                
            )   
        );
        NamedCommands.registerCommand("out",
            carriage.setPositionCommand(CarriageValue.INTAKE_HPS)
        );
        NamedCommands.registerCommand("holdAlgae",
            new SequentialCommandGroup(
                // new WaitCommand(2),
                carriage.setPositionCommand(CarriageValue.PROCESSOR),
                carriage.setIntakeCommand(CarriageIntakeValue.INTAKE),
                elevator.setTargetPositionCommand(Elevator.ElevatorValue.PROCESSOR)
            )
        );

        createAutoBuilder(drive);
        
        autoChooser.addOption("NothingAuto", new InstantCommand());
        // autoChooser.addOption("VisionTest", Autos.testVision(drive, vision));
        // autoChooser.addOption("testM", Autos.testM(drive,elevator, carriage, vision));
        addTuningAuto(drive);
        addAutos(drive, elevator, carriage, vision);
        // autoChooser = AutoBuilder.buildAutoChooser();
        carriage.setPositionCommand(CarriageValue.INTAKE_HPS);

        DashboardUtils.registerDashboard(this);
    }
    
    public  Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public static void addTuningAuto(SwerveDrive drive){
        autoChooser.addOption("BackTest", TuningAutos.backTest(drive));
        autoChooser.addOption("ForwardTest", TuningAutos.forwardTest(drive));
        autoChooser.addOption("TurnTest", TuningAutos.turnTest(drive));
        autoChooser.addOption("SplineTest", TuningAutos.splineTest(drive));
        autoChooser.addOption("StrafeRight", TuningAutos.strafeRight(drive));
        autoChooser.addOption("StrafeLeft", TuningAutos.strafeLeft(drive));
        autoChooser.addOption("LessForwardTest", TuningAutos.lessForwardTest(drive));
        // // autoChooser.addOption("ForwardAndBack", TuningAutos.forwardAndBackTest(drive));
    }

    public static void addAutos(SwerveDrive drive, Elevator elevator, Carriage carriage, Vision vision){
        autoChooser.addOption("middleProcessor", Autos.middleProcessor(drive, elevator, carriage, vision));
        autoChooser.setDefaultOption("middleBarge", Autos.middleBarge(drive, elevator, carriage, vision));
        // autoChooser.addOption("middleL2", Autos.middle(drive, elevator, carriage, vision, "L2"));
        // autoChooser.addOption("middleL3", Autos.middle(drive, elevator, carriage, vision, "L3"));
        // autoChooser.addOption("middleL4", Autos.middle(drive, elevator, carriage, vision, "L4"));
        // autoChooser.addOption("middleL4Algae", Autos.middleL4Algae(drive, elevator, carriage, vision));
        autoChooser.addOption("middleRight", Autos.middleRight(drive, elevator, carriage, vision));


        //middleL4Algae

        // autoChooser.addOption("left1+2", Autos.leftOnePlusTwo(drive, elevator, carriage, vision));
        autoChooser.addOption("left1+2Close", Autos.leftOnePlusTwoClose(drive, elevator, carriage, vision));

        // autoChooser.addOption("left1+1", Autos.leftOnePlusOne(drive, elevator, carriage, vision));
        // autoChooser.addOption("left1", Autos.leftOne(drive, elevator, carriage, vision));
        // autoChooser.addOption("left1+1Seperate", Autos.leftOnePlusOneSeperate(drive, elevator, carriage, vision));

        
        autoChooser.addOption("right1+2Close", Autos.rightOnePlusTwoClose(drive, elevator, carriage, vision));
        // autoChooser.addOption("right1+1", Autos.rightOnePlusOne(drive, elevator, carriage, vision));
        // autoChooser.addOption("right1", Autos.rightOne(drive, elevator, carriage, vision));

    }

    public static void createAutoBuilder(SwerveDrive drive){
        try {
            // RobotConfig config = new RobotConfig(Units.lbsToKilograms(120), 1, 
            //     null, 
            //     Units.inchesToMeters(29));
            RobotConfig config = RobotConfig.fromGUISettings();

            // Configure AutoBuilder
            AutoBuilder.configure(
                drive::getPose,
                drive::resetOdometry,
                drive::getSpeeds,
                drive::setFeedforwardModuleStates,
                new PPHolonomicDriveController(
                        new PIDConstants(SwerveDriveConstants.SwerveDriveConfig.TRANSLATIONAL_KP.getValue(), 
                    SwerveDriveConstants.SwerveDriveConfig.TRANSLATIONAL_KI.getValue(), 
                    SwerveDriveConstants.SwerveDriveConfig.TRANSLATIONAL_KD.getValue()),  // Translation PID constants
                new PIDConstants(SwerveDriveConstants.SwerveDriveConfig.THETA_KP.getValue(), 
                    SwerveDriveConstants.SwerveDriveConfig.THETA_KI.getValue(), 
                    SwerveDriveConstants.SwerveDriveConfig.THETA_KD.getValue())),  // Rotation PID constants
                config,
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                drive // Reference to this subsystem to set requirements
            );
        } catch (Exception e) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", e.getStackTrace());
        }
    }

    @Override
    public void elasticInit() {
        SmartDashboard.putData("Autos/AutoChooser", autoChooser);
    }

    @Override
    public void practiceWriters() {}

    @Override
    public void alerts() {}

    // public Command autoAlgaePickCommand(){
    //     return new SequentialCommandGroup(
    //         // algaeSubsystem.setIntakePositionCommand(IntakeValue.INTAKE),
    //         // algaeSubsystem.setPositionCommand(ArmValue.AUTO_GROUND),
    //         // new WaitUntilCommand(()-> algaeSubsystem.isAlgaeIn()).withTimeout(2),
    //         // new ParallelCommandGroup(
    //         //     algaeSubsystem.setIntakePositionCommand(IntakeValue.READY_SHOOT),
    //         //     algaeSubsystem.setPositionCommand(ArmValue.SHOOT)
    //         // )
    //     );
    // }
}

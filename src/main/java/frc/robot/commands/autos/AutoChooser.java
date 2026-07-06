package frc.robot.commands.autos;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Vision;
import frc.utility.TelemetryUtils;
import frc.utility.TelemetryUtils.Dashboard;

public class AutoChooser implements Dashboard {
    public static final SendableChooser<Command> autoChooser = new SendableChooser<Command>();

    public AutoChooser(Drive drive, Intake intake, Indexer indexer, Kicker kicker, Shooter shooter, Vision vision){
        
        // addTuningAutos(drive);
        // addAutos(drive, intake, indexer, kicker, shooter, vision);
        // adddChampsAutos(drive, intake, indexer, kicker, shooter, vision);
        // addTurretTesting(drive, shooter);

        TelemetryUtils.registerDashboard(this);
    }
    
    public  Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
    
    // public static void addTuningAutos(SwerveDrive drive) {
    //     autoChooser.addOption("BackTest", TuningAutos.backTest(drive));
    //     autoChooser.addOption("ForwardTest", TuningAutos.forwardTest(drive));
    //     autoChooser.addOption("TurnTest", TuningAutos.turnTest(drive));
    //     autoChooser.addOption("SplineTest", TuningAutos.splineTest(drive));
    //     autoChooser.addOption("StrafeRight", TuningAutos.strafeRight(drive));
    //     autoChooser.addOption("StrafeLeft", TuningAutos.strafeLeft(drive));
    //     autoChooser.addOption("LessForwardTest", TuningAutos.lessForwardTest(drive));
    // }

    // public static void addAutos(SwerveDrive drive, Intake intake, Indexer indexer, Kicker kicker, Shooter shooter, Vision vision) {
    //     // autoChooser.addOption("rightNeutralOutpost", Autos.rightNeutralOutpost(drive, intake, indexer, kicker, shooter, vision));
    //     // autoChooser.addOption("rightNeutralOutpostSh", Autos.rightNeutralOutpostSh(drive, intake, indexer, kicker, shooter, vision));
        
    //     // // autoChooser.setDefaultOption("test", Autos.rightNeutralOutpost(drive, intake, indexer, kicker, shooter, vision));
    //     // // autoChooser.addOption("rightNeutralOutpostDouble", Autos.rightNeutralOutpostDouble(drive, intake, indexer, kicker, shooter, vision));
    //     // autoChooser.addOption("leftNeutralDepot", Autos.leftNeutralDepot(drive, intake, indexer, kicker, shooter, vision));
    //     // autoChooser.addOption("leftNeutralDepotSh", Autos.leftNeutralDepotSh(drive, intake, indexer, kicker, shooter, vision));
    //     // // autoChooser.addOption("centerHubDepot", Autos.centerHubDepot(drive, intake, indexer, kicker, shooter, vision));
    //     // // autoChooser.addOption("leftNeutralDepotDouble", Autos.leftNeutralDepotDouble(drive, intake, indexer, kicker, shooter, vision));

    //     autoChooser.addOption("DeepRightOutpost", Autos.newRightOutpost("D",drive, intake, indexer, kicker, shooter, vision));
    //     autoChooser.addOption("MediumRightOutpost", Autos.newRightOutpost("M",drive, intake, indexer, kicker, shooter, vision));
    //     autoChooser.addOption("ShallowRightOutpost", Autos.newRightOutpost("S",drive, intake, indexer, kicker, shooter, vision));

    //     autoChooser.addOption("DeepLeftDepot", Autos.newLeftDepot("D",drive, intake, indexer, kicker, shooter, vision));
    //     autoChooser.addOption("MediumLeftDepot", Autos.newLeftDepot("M",drive, intake, indexer, kicker, shooter, vision));
    //     autoChooser.addOption("ShallowLeftDepot", Autos.newLeftDepot("S",drive, intake, indexer, kicker, shooter, vision));

    //     autoChooser.addOption("RightNeutralSwoop", Autos.rightNeutralSwoop(drive, intake, indexer, kicker, shooter, vision));
    //     autoChooser.addOption("LeftNeutralSwoop", Autos.leftNeutralSwoop(drive, intake, indexer, kicker, shooter, vision));
        
    //     autoChooser.addOption("CenterDepot", Autos.centerDepot(drive, intake, indexer, kicker, shooter, vision));
    //     autoChooser.addOption("CenterDepotStraigh", Autos.centerDepotStaright(drive, intake, indexer, kicker, shooter, vision));

    //     autoChooser.addOption("BumpTest0", Autos.bumpTest(0, drive, intake, indexer, kicker, shooter, vision));
    //     autoChooser.addOption("BumpTest1", Autos.bumpTest(1, drive, intake, indexer, kicker, shooter, vision));
    //     autoChooser.addOption("BumpTest2", Autos.bumpTest(2, drive, intake, indexer, kicker, shooter, vision));
    //     autoChooser.addOption("BumpTest3", Autos.bumpTest(3, drive, intake, indexer, kicker, shooter, vision));

    //     autoChooser.setDefaultOption("TestPath", Autos.testPath(drive, intake, indexer, kicker, shooter, vision));


    // }

    // public static void adddChampsAutos(Drive drive, Intake intake, Indexer indexer, Kicker kicker, Shooter shooter, Vision vision) {
    //     autoChooser.addOption("ChampsSideLeftS", ChampsAutos.sideDepot("S", false, drive, intake, indexer, kicker, shooter, vision));
    //     autoChooser.addOption("ChampsSideRightS", ChampsAutos.sideDepot("S", true, drive, intake, indexer, kicker, shooter, vision));
    //     autoChooser.addOption("ChampsSideLeftM", ChampsAutos.sideDepot("M", false, drive, intake, indexer, kicker, shooter, vision));
    //     autoChooser.addOption("ChampsSideRightM", ChampsAutos.sideDepot("M", true, drive, intake, indexer, kicker, shooter, vision));
    //     autoChooser.addOption("ChampsSideLeftD", ChampsAutos.sideDepot("D", false, drive, intake, indexer, kicker, shooter, vision));
    //     autoChooser.addOption("ChampsSideRightD", ChampsAutos.sideDepot("D", true, drive, intake, indexer, kicker, shooter, vision));

    //     // autoChooser.addOption("ChampsBumpLeft", ChampsAutos.bump(false, drive, intake, indexer, kicker, shooter, vision));
    //     // autoChooser.addOption("ChampsBumpRight", ChampsAutos.bump(true, drive, intake, indexer, kicker, shooter, vision));

    //     // autoChooser.addOption("ChampsSweepLeft", ChampsAutos.neutralSwoop(false, drive, intake, indexer, kicker, shooter, vision));
    //     // autoChooser.addOption("ChampsSweepRight", ChampsAutos.neutralSwoop(true, drive, intake, indexer, kicker, shooter, vision));

    //     autoChooser.addOption("ChampsLeftDouble", ChampsAutos.doubleSide(false, drive, intake, indexer, kicker, shooter, vision));
    //     autoChooser.addOption("ChampsRightDouble", ChampsAutos.doubleSide(true, drive, intake, indexer, kicker, shooter, vision));

    //     autoChooser.addOption("ChampsHubToDepot", ChampsAutos.centerDepot(drive, intake, indexer, kicker, shooter, vision));

    //     autoChooser.addOption("ChampsLeftBumpSwoopS", ChampsAutos.bumpSwoop("S", false, drive, intake, indexer, kicker, shooter, vision));
    //     autoChooser.addOption("ChampsRightBumpSwoopS", ChampsAutos.bumpSwoop("S", true, drive, intake, indexer, kicker, shooter, vision));
        
    //     autoChooser.addOption("ChampsLeftBumpSwoopM", ChampsAutos.bumpSwoop("M", false, drive, intake, indexer, kicker, shooter, vision));
    //     autoChooser.addOption("ChampsRightBumpSwoopM", ChampsAutos.bumpSwoop("M", true, drive, intake, indexer, kicker, shooter, vision));


    // }
    
    @Override
    public void elasticInit() {
        SmartDashboard.putData("Autos/AutoChooser", autoChooser);
    }

    @Override
    public void practiceWriters() {}

    @Override
    public void alerts() {}
}
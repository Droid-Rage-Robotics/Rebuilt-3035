// package frc.robot.commands.drive;

// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
// import frc.robot.DroidRageConstants;
// import frc.robot.subsystems.drive.SwerveDrive;
// import frc.robot.subsystems.vision.Vision;

// public class SpecificAutoAlign extends Command {
//     private SwerveDrive drive;
//     private Vision vision;
//     private int look;
//     // private Timer timer= new Timer();

//     // TELEOP
//     public SpecificAutoAlign(SwerveDrive drive, Vision vision, int look, int lookTwo) { // Pairs Since both blue/red autos
//         this.drive = drive;
//         this.vision = vision;
//         this.look = look;
//         addRequirements(vision);
//     }

//     @Override
//     public void execute() {
//         // timer.reset();
//         switch (DroidRageConstants.alignmentMode) {
//             case LEFT:
//                 if (vision.getID(DroidRageConstants.leftLimelight) != look) {
//                     // while(!vision.gettV(DroidRageConstants.leftLimelight)){
//                     // drive.drive(0, 0, -0.5);
//                     new InstantCommand(() -> drive.drive(0, 0, -0.2)).withTimeout(10)
//                             .onlyWhile(() -> (vision.getID(DroidRageConstants.leftLimelight) == look));
//                     // new
//                     // WaitUntilCommand(()->vision.getID(DroidRageConstants.leftLimelight)==look).withTimeout(1);
//                     // if (vision.getID(DroidRageConstants.leftLimelight) != look) {
//                     // return;
//                     // }
//                 }
//                 break;
//             case RIGHT:
//                 if (vision.getID(DroidRageConstants.rightLimelight) != look) {
//                     // while(!vision.gettV(DroidRageConstants.rightLimelight)){
//                     // drive.drive(0, 0, 0.5);
//                     new InstantCommand(() -> drive.drive(0, 0, 0.2)).withTimeout(2);

//                 }
//                 break;
//         }
//         // timer.restart();
//         // drive.drive(vision.rangeAuto(look), 0, vision.aimAuto(look) - 0.03);
//     }

//     @Override
//     public boolean isFinished() {
//         // return !driver.povUp().getAsBoolean();
//         return (vision.rotController.atSetpoint() && vision.xController.atSetpoint());// || timer.hasElapsed(5);
//     }

// }
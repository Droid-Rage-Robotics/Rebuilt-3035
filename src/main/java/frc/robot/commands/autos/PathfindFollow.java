// package frc.robot.commands.autos;

// import java.util.HashMap;

// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.path.PathConstraints;
// import com.pathplanner.lib.path.PathPlannerPath;

// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// public class PathfindFollow {
//     // private SwerveDrive drive;
//     private String pathName;
//     private boolean doesResetOdo, mirror;
//     private double maxVelocity = 3;
//     private double maxAcceleration = 3;
//     private HashMap<String, Command> eventMap = new HashMap<>();

//     private PathfindFollow(){

//     }

//     public static VelocityBuilder create(String pathName) {
//         PathfindFollow pf = new PathfindFollow();
//         pf.pathName = pathName;
//         return pf.new VelocityBuilder();
//     }
    
//     //FIX THIS
//     public class VelocityBuilder {
//         public AccelarationBuilder withVelocity(double velocity) {
//             PathfindFollow.this.maxVelocity = velocity;
//             return new AccelarationBuilder();
//         }
//     }
//     public class AccelarationBuilder {
//         public ResetOdoBuilder withAcceleration(double acceleration) {
//             PathfindFollow.this.maxAcceleration = acceleration;
//             return new ResetOdoBuilder();
//         }
//     }
//     public class ResetOdoBuilder {
//         public MirrorBuilder withResetOdo(boolean doesResetOdo) {
//             PathfindFollow.this.doesResetOdo = doesResetOdo;
//             return new MirrorBuilder();
//         }
//     }
//     public class MirrorBuilder {
//         public PathBuilder withMirror(boolean mirror) {
//             PathfindFollow.this.mirror = mirror;
//             return new PathBuilder();
//         }
//     }
    

//     public class PathBuilder {
//         public Command build(){
//             try {
//                 PathPlannerPath path;
//                 if(mirror){
//                     path = PathPlannerPath.fromPathFile(pathName).mirrorPath();
//                 }else{
//                     path = PathPlannerPath.fromPathFile(pathName);
//                 }

                

//                 if (doesResetOdo) {
//                     return new SequentialCommandGroup(
//                         AutoBuilder.resetOdom(path.getStartingHolonomicPose().get()),
//                         AutoBuilder.pathfindThenFollowPath(path, 
//                         new PathConstraints(
//                             maxVelocity, maxAcceleration, 
//                             Units.degreesToRadians(540), 
//                             Units.degreesToRadians(720)))
//                     );
//                 } else {
//                     return AutoBuilder.pathfindThenFollowPath(path, 
//                         new PathConstraints(
//                             maxVelocity, maxAcceleration, 
//                             Units.degreesToRadians(540), 
//                             Units.degreesToRadians(720)));
//                 }
//             } catch (Exception e) {
//                 DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
//                 return Commands.none();
//             }
//         }

//         public PathBuilder addMarker(String name, Command toRun) {
//             eventMap.put(name, toRun);
//             return this;
//         }
//         public PathBuilder addMarker(String name, ParallelCommandGroup toRun) {
//             eventMap.put(name, toRun);
//             return this;
//         }
//     }

        
// }
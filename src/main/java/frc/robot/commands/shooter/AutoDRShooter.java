// package frc.robot.commands.shooter;

// import static edu.wpi.first.units.Units.*;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.DroidRageConstants;
// import frc.robot.DroidRageConstants.FieldConstants;
// import frc.robot.subsystems.drive.SwerveDrive;
// import frc.robot.subsystems.shooter.Shooter;
// import frc.utility.DRAreaManager;

// public class AutoDRShooter extends Command {
//     private final Shooter shooter;
//     private final SwerveDrive drive;
    
//     private Translation2d hubPose;
//     private Translation2d goalPose, alliancePose;
//     private double distanceRobotToGoal;
//     private Pose2d drivePose = new Pose2d();
    
//     public AutoDRShooter(SwerveDrive drive, Shooter shooter) {
//         this.shooter = shooter;
//         this.drive = drive;

//         //LEAVE THIS - Do NOT Comment Out
//         if(DroidRageConstants.alliance == Alliance.Red){
//             this.hubPose = FieldConstants.HUB_RED;
//             this.alliancePose = FieldConstants.ALLIANCE_RED;
//         } else if (DroidRageConstants.alliance == Alliance.Blue){
//             this.hubPose = FieldConstants.HUB_BLUE;
//             this.alliancePose = FieldConstants.ALLIANCE_BLUE;
//         }
        
//         this.goalPose = this.hubPose;

//         addRequirements(
//             shooter.getHood(),
//             shooter.getShooterWheel(),
//             shooter.getTurret());
//     }

//     @Override
//     public void initialize(){
//         if(DroidRageConstants.alliance == Alliance.Red){
//             this.goalPose = FieldConstants.HUB_RED;
//             this.alliancePose = FieldConstants.ALLIANCE_RED;
//         } else if (DroidRageConstants.alliance == Alliance.Blue){
//             this.goalPose = FieldConstants.HUB_BLUE;
//             this.alliancePose = FieldConstants.ALLIANCE_BLUE;
//         }
//     }
    
//     @Override 
//     public void execute(){
//         // distanceRobotToGoal = Shooter.getDistanceToHub(drive.getState().Pose, goalPose);

//         // shooter.getTurret().setGoalAngle(Shooter.calculateAzimuthAngle(drive.getState().Pose, goalPose));
//         // shooter.getHood().setGoalAngle(Degrees.of(Shooter.hubHoodMap.get(distanceRobotToGoal)));
//         // shooter.getShooterWheel().setTargetVelocity(RotationsPerSecond.of(Shooter.hubWheelMap.get(distanceRobotToGoal)));
//         drivePose = drive.getState().Pose;
        
//         switch(DRAreaManager.getCurrentZone()){
//             case ALLIANCE_ZONE:
//                 this.goalPose = this.hubPose;
//                 break;
//             case BETWEEN:
//                 this.goalPose = this.hubPose;
//                 break;
//             case NEUTRAL, OPPOSITION:
//                 // this.goalPose = this.hubPose;
//                 // this.goalPose = new Translation2d(this.alliancePose.getX(), drivePose.getY());
//                 this.goalPose = new Translation2d(this.alliancePose.getX(), Shooter.getShuttleY(drivePose));
//                 break;
            
//         }

//         // Get the predicted robot pose based on current velocity to improve targeting while moving for 1 Second ahead
//         Pose2d lookAheadPose = Shooter.predictPosePos(
//             drivePose, 
//             goalPose,
//             drive.getCurrentRobotChassisSpeeds(),
//             Shooter.timeOfFlightMap.get(Shooter.getDistanceToHub(drivePose, goalPose)));
//         distanceRobotToGoal = Shooter.getDistanceToHub(lookAheadPose, goalPose);

//         //System.out.println(distanceRobotToGoal);

//         if (!DroidRageConstants.isShooterManual) {
//             switch(DRAreaManager.getCurrentZone()){
//                 case ALLIANCE_ZONE:
//                     shooter.getTurret().setGoalAngle(Shooter.calculateAzimuthAngle(drivePose, goalPose));
//                     shooter.getHood().setGoalAngle(Degrees.of(Shooter.hubHoodMap.get(distanceRobotToGoal)));
//                     shooter.getShooterWheel().setTargetVelocity(RotationsPerSecond.of(Shooter.hubWheelMap.get(distanceRobotToGoal)));
//                     break;
//                 case NEUTRAL:
//                     shooter.getTurret().setGoalAngle(Shooter.calculateAzimuthAngle(drivePose, goalPose));
//                     shooter.getHood().setGoalAngle(Degrees.of(Shooter.allianceHoodMap.get(distanceRobotToGoal)));
//                     shooter.getShooterWheel().setTargetVelocity(RotationsPerSecond.of(Shooter.allianceWheelMap.get(distanceRobotToGoal)));

//                     // shooter.getHood().setGoalAngle(Degrees.zero());
//                     // shooter.getShooterWheel().setTargetVelocity(Shooter.IDLE_VELOCITY);
//                     break;
//                 case OPPOSITION:
//                     shooter.getTurret().setGoalAngle(Shooter.calculateAzimuthAngle(drivePose, goalPose));
//                     shooter.getHood().setGoalAngle(Shooter.HOOD_SHUTTLE_ANGLE);
//                     shooter.getShooterWheel().setTargetVelocity(Shooter.SHUTTLE_VELOCITY);

//                     // shooter.getHood().setGoalAngle(Degrees.zero());
//                     // shooter.getShooterWheel().setTargetVelocity(Shooter.IDLE_VELOCITY);
//                     break;
//                 case BETWEEN:
//                     shooter.getTurret().setGoalAngle(Shooter.calculateAzimuthAngle(drivePose, goalPose));
//                     shooter.getHood().setGoalAngle(Degrees.zero());
//                     shooter.getShooterWheel().setTargetVelocity(RotationsPerSecond.of(Shooter.hubWheelMap.get(distanceRobotToGoal)));
//                     break;
//             }
//         }
//     }
// }

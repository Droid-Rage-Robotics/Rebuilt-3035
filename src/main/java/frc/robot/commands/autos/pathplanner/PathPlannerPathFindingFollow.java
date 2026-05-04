package frc.robot.commands.autos.pathplanner;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.autos.AutoPose;
import frc.robot.subsystems.drive.SwerveDrive;

public class PathPlannerPathFindingFollow {
    public static PathConstraints maxSpeedConstraint = new PathConstraints(
        3, //7.6
        3,//7.6
        Math.toRadians(360), //1506
        Math.toRadians(360));
    public static PathConstraints shootConstraint = new PathConstraints(
        3, //7.6
        3,//7.6
        Math.toRadians(360), //1506
        Math.toRadians(360));
    
    private final SwerveDrive drive;
    private final Pose2d bluePose;
    private PathConstraints pathConstraint = maxSpeedConstraint;

    //You might not actually need the DT
    private PathPlannerPathFindingFollow(SwerveDrive drive, Pose2d bluePose, PathConstraints pathConstraint) {
        this.drive = drive;
        this.bluePose = bluePose;
        this.pathConstraint = maxSpeedConstraint;
    }

    private PathPlannerPathFindingFollow(SwerveDrive drive, Pose2d bluePose) {
        this.drive = drive;
        this.bluePose = bluePose;
    }

    public static PathPlannerPathFindingFollow create(SwerveDrive drive, Pose2d bluePose) {
        return new PathPlannerPathFindingFollow(drive, bluePose);
    }
    public static PathPlannerPathFindingFollow create(SwerveDrive drive, AutoPose autoPose) {
        return new PathPlannerPathFindingFollow(drive, autoPose.bluePose);
    }

    public PathPlannerPathFindingFollow setConstraint(PathConstraints pathConstraint) {
        return new PathPlannerPathFindingFollow(drive, bluePose, pathConstraint);
    }

    public Command build(){
        return AutoBuilder.pathfindToPoseFlipped(bluePose, maxSpeedConstraint);
    }
}

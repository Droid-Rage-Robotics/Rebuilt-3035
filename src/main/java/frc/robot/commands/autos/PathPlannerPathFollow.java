package frc.robot.commands.autos;

import java.util.HashMap;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.drive.SwerveDrive;

public class PathPlannerPathFollow {
    private final SwerveDrive drive;
    private final String pathName;
    private final boolean doesResetOdo;
    private double maxVelocity = 0.3;
    private double acceleration = 0.5;
    private HashMap<String, Command> eventMap = new HashMap<>();

    /*  */
    private PathPlannerPathFollow(SwerveDrive drive, String pathName, double maxVelocity, double acceleration, HashMap<String, Command> eventMap, boolean doesResetOdo) {
        this.drive = drive;
        this.pathName = pathName;
        this.maxVelocity = maxVelocity;
        this.acceleration = acceleration;
        this.eventMap = eventMap;
        this.doesResetOdo=doesResetOdo;
    }

    private PathPlannerPathFollow(SwerveDrive drive, String pathName, boolean doesResetOdo) {
        this.drive = drive;
        this.pathName = pathName;
        this.doesResetOdo=doesResetOdo;
    }

    public static PathPlannerPathFollow create(SwerveDrive drive, String pathName) {
        return new PathPlannerPathFollow(drive, pathName, false);
    }

    public static PathPlannerPathFollow create(SwerveDrive drive, String pathName, boolean doesResetOdo) {
        return new PathPlannerPathFollow(drive, pathName, doesResetOdo);
        
    }

    public PathPlannerPathFollow withMaxVelocity(double maxVelocity) {
        return new PathPlannerPathFollow(drive, pathName, maxVelocity, acceleration, eventMap, doesResetOdo);
    }

    public PathPlannerPathFollow withMaxAcceleration(double acceleration) {
        return new PathPlannerPathFollow(drive, pathName, maxVelocity, acceleration, eventMap, doesResetOdo);
    }

    public PathPlannerPathFollow addMarker(String name, Command toRun) {
        eventMap.put(name, toRun);
        return new PathPlannerPathFollow(drive, pathName, maxVelocity, acceleration, eventMap, doesResetOdo);
    }
    public PathPlannerPathFollow addMarker(String name, ParallelCommandGroup toRun) {
        eventMap.put(name, toRun);
        return new PathPlannerPathFollow(drive, pathName, maxVelocity, acceleration, eventMap,doesResetOdo);
    }
    public Command build(){
        // PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
        // return AutoBuilder.followPath(path);

        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

            if (doesResetOdo) {
                AutoBuilder.resetOdom(path.getStartingHolonomicPose().get());
            }

            return AutoBuilder.followPath(path);
        } catch (Exception e) {
            DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
            return Commands.none();
        }
    }
}

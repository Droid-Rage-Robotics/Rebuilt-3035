package frc.robot.commands.autos;

import java.util.HashMap;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.SwerveDrive;

public class PathPlannerPathFollow {
    private final SwerveDrive drive;
    private final String pathName;
    private final boolean doesResetOdo, mirror;
    private double maxVelocity = 0.3;
    private double maxAcceleration = 0.5;
    private HashMap<String, Command> eventMap = new HashMap<>();

    /*  */
    private PathPlannerPathFollow(SwerveDrive drive, String pathName, HashMap<String, Command> eventMap, boolean doesResetOdo, boolean mirror) {
        this.drive = drive;
        this.pathName = pathName;
        this.eventMap = eventMap;
        this.doesResetOdo=doesResetOdo;
        this.mirror=mirror;
    }

    private PathPlannerPathFollow(SwerveDrive drive, String pathName, boolean doesResetOdo) {
        this.drive = drive;
        this.pathName = pathName;
        this.doesResetOdo=doesResetOdo;
        this.mirror=false;
    }

    public static PathPlannerPathFollow create(SwerveDrive drive, String pathName) {
        return new PathPlannerPathFollow(drive, pathName, false);
    }

    public static PathPlannerPathFollow create(SwerveDrive drive, String pathName, boolean doesResetOdo) {
        return new PathPlannerPathFollow(drive, pathName, doesResetOdo);   
    }

    public PathPlannerPathFollow withMaxVelocity(double maxVelocity) {
        this.maxVelocity=maxVelocity;
        return this;
    }

    public PathPlannerPathFollow withMaxAcceleration(double maxAcceleration) {
        this.maxAcceleration=maxAcceleration;
        return this;
    }
    // public PathPlannerPathFollow withMirror(boolean mirror) {
    //     this.mirror=mirror;
    //     return this;
    // }

    public PathPlannerPathFollow addMarker(String name, Command toRun) {
        eventMap.put(name, toRun);
        return new PathPlannerPathFollow(drive, pathName, eventMap, doesResetOdo, mirror);
    }
    public PathPlannerPathFollow addMarker(String name, ParallelCommandGroup toRun) {
        eventMap.put(name, toRun);
        return new PathPlannerPathFollow(drive, pathName, eventMap, doesResetOdo, mirror);
    }

    public Command build(){
        try {
            PathPlannerPath path;
            if(mirror){
                path = PathPlannerPath.fromPathFile(pathName).mirrorPath();
            }else{
                path = PathPlannerPath.fromPathFile(pathName);
            }

            if (doesResetOdo) {
                return new SequentialCommandGroup(
                    AutoBuilder.resetOdom(path.getStartingHolonomicPose().get()),
                    AutoBuilder.followPath(path)
                );
            } else {
                return AutoBuilder.followPath(path);
            }
        } catch (Exception e) {
            DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
            return Commands.none();
        }
    }
}

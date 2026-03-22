// package frc.robot;

// import org.littletonrobotics.junction.Logger;
// import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

// public class Telemetry {
//     /**
//      * Construct a telemetry object, with the specified max speed of the robot
//      * 
//      */
//     public Telemetry() {}

//     public void telemeterize(SwerveDriveState state) {
//         /* Log to AdvantageScope */ // TODO: check thread safety
//         Logger.recordOutput("DriveState/Pose", state.Pose);
//         Logger.recordOutput("DriveState/Speeds", state.Speeds);
//         Logger.recordOutput("DriveState/ModuleStates", state.ModuleStates);
//         Logger.recordOutput("DriveState/ModuleTargets", state.ModuleTargets);
//         Logger.recordOutput("DriveState/ModulePositions", state.ModulePositions);
//         Logger.recordOutput("DriveState/OdometryPeriod", state.OdometryPeriod);
//     }
// }

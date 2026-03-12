package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NTSendable;
import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.DroidRageConstants;
import frc.robot.subsystems.drive.SwerveConfig.Speed;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.LimelightHelpers;
import frc.robot.subsystems.vision.LimelightHelpers.PoseEstimate;
import lombok.Getter;

public class SwerveDrive extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements Subsystem, NTSendable {
    private static final double kSimLoopPeriod = 0.004; // 4 ms

    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    @Getter private final SwerveConfig config;

    private final boolean isEnabled;

    private volatile Speed speed = Speed.NORMAL;
    
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();
    private final SwerveRequest.ApplyRobotSpeeds AutoRequest = new SwerveRequest.ApplyRobotSpeeds();


    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            null
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            (log) -> {
                    log.motor("driveMotor_POD-0")    
                        .voltage(getModules()[0].getDriveMotor().getMotorVoltage().getValue())
                        .linearPosition(Meters.of(getModules()[0].getPosition(true).distanceMeters))
                        .linearVelocity(MetersPerSecond.of(getModules()[0].getCurrentState().speedMetersPerSecond));
                },
            this
        )
    );

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            null
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            (log) -> {
                    log.motor("steerMotor_POD-0")    
                        .voltage(getModules()[0].getSteerMotor().getMotorVoltage().getValue())
                        .angularPosition(getModules()[0].getEncoder().getPosition().getValue())
                        .angularVelocity(getModules()[0].getEncoder().getVelocity().getValue());
            },
            this
        )
    );

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI),
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            null
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
            },
            (log) -> {
                log.motor("chassis")
                    .voltage(Volts.of(m_rotationCharacterization.RotationalRate))
                    .angularPosition(getPigeon2().getYaw().getValue())
                    .angularVelocity(getPigeon2().getAngularVelocityZWorld().getValue());
            },
            this
        )
    );

    private final Field2d fieldPose = new Field2d();

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineSteer;

    public SwerveDrive(boolean isEnabled, SwerveConfig config) {
        super(
            TalonFX::new,
            TalonFX::new,
            CANcoder::new, 
            config.getDrivetrainConstants(), 
            config.getModules()
        );

        this.config = config;
        this.isEnabled=isEnabled;

        if (Utils.isSimulation()) {
            startSimThread();
        }

        SmartDashboard.putData("Drive/fieldPose", fieldPose);
        configurePathPlanner();
    }

    public void updateVisionOdometry() {
        PoseEstimate left = Vision.getLeftEstimate();
        PoseEstimate right = Vision.getRightEstimate();

        if (left != null && left.tagCount > 0) {
            double dist = Vision.closestTagDistance(left);
            double std = Vision.distanceToStdDev(dist);
            double stdTheta = Math.toRadians(Math.max(5, dist * 4));

            // if (Vision.isReasonable(getEstimatedPose(), left.pose)) {
                addVisionMeasurement(
                    left.pose,
                    left.timestampSeconds,
                    VecBuilder.fill(std, std, stdTheta)
                );
            // }
        }

        if (right != null && right.tagCount > 0) {
            double dist = Vision.closestTagDistance(right);
            double std = Vision.distanceToStdDev(dist);
            double stdTheta = Math.toRadians(Math.max(5, dist * 4));

            // if (Vision.isReasonable(getEstimatedPose(), right.pose)) {
                addVisionMeasurement(
                    right.pose,
                    right.timestampSeconds,
                    VecBuilder.fill(std, std, stdTheta)
                );
            // }  
        }
    }
    
    @Override
    public void initSendable(NTSendableBuilder builder) {
        
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> request) {
        return run(() -> this.setControl(request.get()));
    }

    @Override
    public void setControl(SwerveRequest request) {
        if (!isEnabled) {
            return;
        } else {
            super.setControl(request);
        }
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return getKinematics().toChassisSpeeds(getState().ModuleStates);
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    @Override
    public void periodic() {
        LimelightHelpers.SetRobotOrientation(DroidRageConstants.leftLL, 
            getState().RawHeading.getDegrees(), 0, 0, 0, 0, 0);

        LimelightHelpers.SetRobotOrientation(DroidRageConstants.rightLL, 
            getState().RawHeading.getDegrees(), 0, 0, 0, 0, 0);
        
        // updateVisionOdometry();

        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? config.getRedAlliancePerspectiveRotation()
                        : config.getBlueAlliancePerspectiveRotation()
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }
        
        Logger.recordOutput("DriveState/Pose", getState().Pose);
        Logger.recordOutput("DriveState/Speeds", getState().Speeds);
        Logger.recordOutput("DriveState/ModuleStates", getState().ModuleStates);
        Logger.recordOutput("DriveState/ModuleTargets", getState().ModuleTargets);
        Logger.recordOutput("DriveState/ModulePositions", getState().ModulePositions);
        Logger.recordOutput("DriveState/OdometryPeriod", getState().OdometryPeriod);

        fieldPose.setRobotPose(getState().Pose);
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    public Command setSpeed(Speed speed) {
        return new InstantCommand(() -> {
            this.speed = speed;
        });
    }

    public double getTranslationalSpeed() {
        return speed.getTranslationalSpeed();
    }

    public double getAngularSpeed() {
        return speed.getAngularSpeed();
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     * <p>
     * Note that the vision measurement standard deviations passed into this method
     * will continue to apply to future measurements until a subsequent call to
     * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement
     *     in the form [x, y, theta]ᵀ, with units in meters and radians.
     */
    @Override
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs
    ) {

        
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
    }

    /**
     * Return the pose at a given timestamp, if the buffer is not empty.
     *
     * @param timestampSeconds The timestamp of the pose in seconds.
     * @return The pose at the given timestamp (or Optional.empty() if the buffer is empty).
     */
    @Override
    public Optional<Pose2d> samplePoseAt(double timestampSeconds) {
        return super.samplePoseAt(Utils.fpgaToCurrentTime(timestampSeconds));
    }

    // --------------------------------------------------------------------------------
    // Path Planner
    // --------------------------------------------------------------------------------
    private void configurePathPlanner() {
        // Seed robot to mid field at start (Paths will change this starting position)
        // resetPose(
        //         new Pose2d(
        //                 Units.feetToMeters(10),
        //                 Units.feetToMeters(27.0 / 2.0),
        //                 config.getBlueAlliancePerspectiveRotation()));

        RobotConfig robotConfig = null; // Initialize with null in case of exception
        try {
            robotConfig =
                    RobotConfig.fromGUISettings(); // Takes config from Robot Config on Pathplanner
            // Settings
        } catch (Exception e) {
            e.printStackTrace(); // Fallback to a default configuration
        }

        AutoBuilder.configure(
                () -> this.getState().Pose, // Supplier of current robot pose
                this::resetPose, // Consumer for seeding pose against auto
                this::getCurrentRobotChassisSpeeds,
                speeds ->
                        this.setControl(
                                AutoRequest.withSpeeds(
                                        speeds)), // Consumer of ChassisSpeeds to drive the robot
                new PPHolonomicDriveController(
                        SwerveConfig.TRANSLATIONAL_PID, SwerveConfig.THETA_PID),
                robotConfig,
                () ->
                        DriverStation.getAlliance().orElse(Alliance.Blue)
                                == Alliance.Red, // Assume the path needs to be flipped for Red vs
                // Blue, this is normally
                // the case
                this); // Subsystem for requirements
    }

    public Translation2d getTranslation2d(){
        return fieldPose.getRobotPose().getTranslation();
    }
}

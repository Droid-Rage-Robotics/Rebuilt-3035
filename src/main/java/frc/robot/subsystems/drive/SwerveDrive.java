package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.DroidRageConstants;
import frc.robot.DroidRageConstants.FieldConstants;
import frc.robot.subsystems.drive.SwerveDriveConstants.Speed;
import frc.robot.subsystems.drive.SwerveDriveConstants.SwerveDriveConfig;
import frc.robot.subsystems.drive.SwerveModuleConstants.POD;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.LimelightHelpers.PoseEstimate;
import frc.utility.DashboardUtils;
import frc.utility.DashboardUtils.Dashboard;
import frc.utility.motor.MotorConstants.Direction;
import frc.utility.motor.TalonEx;
import lombok.Getter;
import lombok.Setter;

//Set Voltage instead of set Power
//Set them to 90 to 100%
public class SwerveDrive extends SubsystemBase implements Dashboard {
    public enum TippingState {
        NO_TIP_CORRECTION,
        ANTI_TIP,
        ;
    }
    // Translation2d(x,y) == Translation2d(front, left)
    //front +; back -
    //left
    public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(SwerveDriveConfig.WHEEL_BASE.getValue() / 2,
                    SwerveDriveConfig.TRACK_WIDTH.getValue() / 2), // Front Left ++
            new Translation2d(SwerveDriveConfig.WHEEL_BASE.getValue() / 2,
                    -SwerveDriveConfig.TRACK_WIDTH.getValue() / 2), // Front Right +-
            new Translation2d(-SwerveDriveConfig.WHEEL_BASE.getValue() / 2,
                    SwerveDriveConfig.TRACK_WIDTH.getValue() / 2), // Back Left -+
            new Translation2d(-SwerveDriveConfig.WHEEL_BASE.getValue() / 2,
                    -SwerveDriveConfig.TRACK_WIDTH.getValue() / 2) // Back Right --
    );

    private final SwerveModuleConstants frontRight = new SwerveModuleConstants()
        .withSubsystem(this)
        .withPodName(POD.FR)
        .withDriveMotorId(3)
        .withDriveMotorDirection(Direction.Forward)
        .withDriveMotorIsEnabled(false)
        .withTurnMotorId(1)
        .withTurnMotorDirection(Direction.Forward)
        .withTurnMotorIsEnabled(false)
        .withEncoderId(2)
        .withEncoderDirection(SensorDirectionValue.CounterClockwise_Positive)
        .withEncoderOffsetRad(SwerveDriveConfig.FRONT_RIGHT_ABSOLUTE_ENCODER_OFFSET_RADIANS.getValue());

    private final SwerveModuleConstants backRight = new SwerveModuleConstants()
        .withSubsystem(this)
        .withPodName(POD.BR)    
        .withDriveMotorId(6)
        .withDriveMotorDirection(Direction.Forward)
        .withDriveMotorIsEnabled(false)
        .withTurnMotorId(4)
        .withTurnMotorDirection(Direction.Forward)
        .withTurnMotorIsEnabled(false)
        .withEncoderId(5)
        .withEncoderDirection(SensorDirectionValue.CounterClockwise_Positive)
        .withEncoderOffsetRad(SwerveDriveConfig.BACK_RIGHT_ABSOLUTE_ENCODER_OFFSET_RADIANS.getValue());
    
    private final SwerveModuleConstants backLeft = new SwerveModuleConstants()
        .withSubsystem(this)
        .withPodName(POD.BL)
        .withDriveMotorId(9)
        .withDriveMotorDirection(Direction.Forward)
        .withDriveMotorIsEnabled(false)
        .withTurnMotorId(7)
        .withTurnMotorDirection(Direction.Forward)
        .withTurnMotorIsEnabled(false)
        .withEncoderId(8)
        .withEncoderDirection(SensorDirectionValue.CounterClockwise_Positive)
        .withEncoderOffsetRad(SwerveDriveConfig.BACK_LEFT_ABSOLUTE_ENCODER_OFFSET_RADIANS.getValue());
    
    private final SwerveModuleConstants frontLeft = new SwerveModuleConstants()
        .withSubsystem(this)
        .withPodName(POD.FL)
        .withDriveMotorId(12)
        .withDriveMotorDirection(Direction.Forward)
        .withDriveMotorIsEnabled(false)
        .withTurnMotorId(10)
        .withTurnMotorDirection(Direction.Forward)
        .withTurnMotorIsEnabled(false)
        .withEncoderId(11)
        .withEncoderDirection(SensorDirectionValue.CounterClockwise_Positive)
        .withEncoderOffsetRad(SwerveDriveConfig.FRONT_LEFT_ABSOLUTE_ENCODER_OFFSET_RADIANS.getValue());
    
    private final SwerveModuleConstants[] swerveModuleConstants = { frontLeft, frontRight, backLeft, backRight };
    
    @Getter private final SwerveModule[] swerveModules;
    
    private final Pigeon2 pigeon2 = new Pigeon2(13, DroidRageConstants.driveCanBus);

    private final SwerveDriveOdometry odometry;

    private final SwerveDriveOdometry3d odometry3d;

    private final SwerveDrivePoseEstimator poseEstimator;

    private final SwerveDrivePoseEstimator3d poseEstimator3d;

    private volatile Speed speed = Speed.NORMAL;
    @Getter @Setter private volatile TippingState tippingState = TippingState.NO_TIP_CORRECTION;

    private final Field2d field = new Field2d();
    private final Field2d visionField = new Field2d();

    private final boolean isEnabled;
    private final Vision vision;
    // SwerveDrivetrain

    public SwerveDrive(boolean isEnabled, Vision vision) {
        this.isEnabled = isEnabled;
        this.vision=vision;
        
        this.swerveModules = new SwerveModule[swerveModuleConstants.length];
        
        for (SwerveModuleConstants m_swerveModuleConstants: swerveModuleConstants) {
            m_swerveModuleConstants.driveMotorIsEnabled=isEnabled;
            m_swerveModuleConstants.turnMotorIsEnabled=isEnabled;
        }

        for (int i = 0; i < swerveModuleConstants.length; i++) {
            this.swerveModules[i] = SwerveModule.createWithConstants(swerveModuleConstants[i]);
        }
        
        for (SwerveModule swerveModule: swerveModules) {
            swerveModule.brakeMode();
            // swerveModule.coastMode();
            // swerveModule.brakeAndCoastMode();
        }

        // Pigeon Wires are facing the front of the robot
        pigeon2.getConfigurator().apply(new MountPoseConfigs());   
        for(int num = 0; num<4; num++){
            swerveModules[num].setDriveMotorIsEnabled(isEnabled);
            swerveModules[num].setTurnMotorIsEnabled(isEnabled);
        }   
        
        // NetworkTable table = NetworkTableInstance.getDefault().getTable("Drivetrain");
        // yawPublisher = table.getStructTopic("Yaw", Rotation2d.struct).publish();

        odometry = new SwerveDriveOdometry (
            DRIVE_KINEMATICS, 
            new Rotation2d(0), 
            getModulePositions()
        );

        odometry3d = new SwerveDriveOdometry3d(
            DRIVE_KINEMATICS, 
            new Rotation3d(), 
            getModulePositions(),
            new Pose3d()
        );

        poseEstimator = new SwerveDrivePoseEstimator(
            DRIVE_KINEMATICS, 
            getRotation2d(), 
            getModulePositions(), 
            new Pose2d()
        );

        poseEstimator3d = new SwerveDrivePoseEstimator3d(
            DRIVE_KINEMATICS, 
            getRotation3d(), 
            getModulePositions(), 
            new Pose3d()
        );
        
        
        DashboardUtils.registerDashboard(this);
    }

    // private final StructPublisher<Rotation2d> yawPublisher;

    
    
    @Override
    public void elasticInit() {
        SmartDashboard.putData("Drive/Swerve Drive", this);
        SmartDashboard.putData("Drive/Gyro", pigeon2);
        SmartDashboard.putData("Drive/Drive Pose", field);
        SmartDashboard.putData("Drive/Vision Pose", visionField);
        SmartDashboard.putBoolean("Drive/Info/isEnabled", isEnabled);
        SmartDashboard.putData("Drive/Data", data);

        


        
    }

    @Override
    public void practiceWriters() {
        // SmartDashboard.putData("Drive/Angles", frontLeft);
        // SmartDashboard.putData("Drive/Angles", frontRight);
        // SmartDashboard.putData("Drive/Angles", backLeft);
        // SmartDashboard.putData("Drive/Angles", backRight);
    }

    @Override
    public void alerts() {}

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveDrive");

        // Use POD enum for names
        POD[] pods = {POD.FL, POD.FR, POD.BL, POD.BR};
        
        for (int i = 0; i < swerveModules.length; i++) {
            final int index = i;
            String podName = pods[i].toString();
            
            builder.addDoubleProperty(podName + " Angle", 
                () -> swerveModules[index].getTurningPosition(), null);
            builder.addDoubleProperty(podName + " Velocity", 
                () -> swerveModules[index].getDriveVelocity(), null);
            builder.addDoubleProperty(podName + " Position", 
                () -> swerveModules[index].getDrivePos(), null);
        }

        builder.addDoubleProperty("Robot Angle", () -> getRotation2d().getRadians(), null);
    }

    private final Sendable data = new Sendable() {
        @Override
        public void initSendable(SendableBuilder builder) {
            builder.addStringProperty("TippingState", () -> tippingState.name(), null);
            builder.addStringProperty("Speed", () -> speed.name(), null);
        }
    };
     
    @Override
    public void periodic() {
        odometry.update(
            getRotation2d(),
            getModulePositions()
        );

        odometry3d.update(
            getRotation3d(), 
            getModulePositions()
        );

        field.setRobotPose(getPose());
        
        Logger.recordOutput("Drive/Pose3d", getPose3d());
        Logger.recordOutput("Drive/Pose", getPose());
        Logger.recordOutput("Drive/PoseEstimate3d", getEstimatedPose3d());
        Logger.recordOutput("Drive/PoseEstimate", getEstimatedPose());
        Logger.recordOutput("Drive/Rotation3d", getRotation3d());
        Logger.recordOutput("Drive/Rotation2d", getRotation2d());
        Logger.recordOutput("Drive/States", getModuleStates());
        Logger.recordOutput("Drive/Speeds", getChassisSpeeds());
        
        // updateVisionOdometry();

        // updateTelemetry();

        
    }

    // private void updateTelemetry() {
    //     for (SwerveModule module : swerveModules) {
    //         module.updateTelemetry();
    //     }

    //     yawPublisher.set(getRotation2d());
    // }

    @Override
    public void simulationPeriodic() {
        periodic();
    }

    public void updateVisionOdometry() {
        poseEstimator.update(getRotation2d(), getModulePositions());

        poseEstimator3d.update(getRotation3d(), getModulePositions());

        PoseEstimate estimate = vision.getOdoEstimate();

        if (estimate != null && estimate.tagCount > 0) {
            double dist = vision.closestTagDistance(estimate);
            double std = vision.distanceToStdDev(dist);
            double stdTheta = Math.toRadians(Math.max(5, dist * 4));

            if (vision.isReasonable(getEstimatedPose(), estimate.pose))
                poseEstimator.addVisionMeasurement(
                    estimate.pose,
                    estimate.timestampSeconds,
                    VecBuilder.fill(std, std, stdTheta)
                );

                poseEstimator3d.addVisionMeasurement(
                    new Pose3d(estimate.pose), 
                    estimate.timestampSeconds,
                    VecBuilder.fill(std, std, std, stdTheta));
        }

        visionField.setRobotPose(getEstimatedPose());
    }

    

    public double getTranslationalSpeed() {
        return speed.getTranslationalSpeed();
    }

    public double getAngularSpeed() {
        return speed.getAngularSpeed();
    }

    public double getForwardVelocity() {
        // Index 0 = FL, Index 1 = FR (based on your array order)
        double frontLeftVel = swerveModules[0].getDriveVelocity();
        double frontRightVel = swerveModules[1].getDriveVelocity();
        return (frontLeftVel + frontRightVel) / 2;
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(getRotation2d(), getModulePositions(), pose);
    }

    public void drive(double xSpeed, double ySpeed, double turnSpeed) {
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turnSpeed);
        drive(chassisSpeeds);
    }
    public void drive(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] states = SwerveDrive.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        setModuleStates(states);
    }

    public void setModuleStates(SwerveModuleState[] states) {
        if (!isEnabled) return;
        SwerveDriveKinematics.desaturateWheelSpeeds(
            states, 
            SwerveDriveConstants.SwerveDriveConfig.PHYSICAL_MAX_SPEED_METERS_PER_SECOND.getValue()
        );

        // swerveModules[1].setState(states[1]);
        for (int i = 0; i < 4; i++) {
            swerveModules[i].setState(states[i]);
        }
    }

    public void setFeedforwardModuleStates(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] states = SwerveDrive.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        setFeedforwardModuleStates(states);
    }
    
    public void setFeedforwardModuleStates(SwerveModuleState[] states) {
        if (!isEnabled) return;
        SwerveDriveKinematics.desaturateWheelSpeeds(
            states, 
            SwerveDriveConstants.SwerveDriveConfig.PHYSICAL_MAX_SPEED_METERS_PER_SECOND.getValue()
        );

        for (int i = 0; i < 4; i++) {
            swerveModules[i].setFeedforwardState(states[i]);
        }
    }

    public ChassisSpeeds getChassisSpeeds() {
        return DRIVE_KINEMATICS.toChassisSpeeds(getModuleStates());
    }

    public void stop() {
        for (SwerveModule swerveModule: swerveModules) {
            swerveModule.stop();
        }
    }

    public void setYaw(double degrees){
        pigeon2.setYaw(degrees, 5);
    }
    
    public TrapezoidProfile.Constraints getThetaConstraints() {
        return new TrapezoidProfile.Constraints(
            SwerveDriveConfig.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND.getValue(),
            SwerveDriveConfig.MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED.getValue());
    }

    // public void changeAllianceRotation(){//DO THIS AT THE END OF AUTOS ONLY
    //     //No WORK
    //     setYaw(getHeading() +90);
    //     switch (DriverStation.getAlliance().get()) {
    //         case Red:
    //             setYaw(getHeading() + 180);
    //             break;
    //         case Blue:
    //             setYaw(getHeading());
    //             break;
    //     }
    // } 

    public boolean isInNeutralZone() {
        boolean isInNeutralZone = (
            getPose().getX()>FieldConstants.NEUTRAL_ZONE_START.in(Units.Meters) && 
            getPose().getX()<FieldConstants.NEUTRAL_ZONE_END.in(Units.Meters)
        );

        return isInNeutralZone;
    }

    /* ---------------- SysId Routines ---------------- */
    
    public SysIdRoutine getDriveSysId() {
        return new SysIdRoutine(
            new SysIdRoutine.Config(
                null, // Use default ramp rate (1 V/s)
                Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
                null, // Use default timeout (10 s)
                (state) -> SignalLogger.writeString("sysid-test-state-SwerveDrive_drive", state.toString())
            ),
            new SysIdRoutine.Mechanism(voltage -> {
                // Apply voltage to all drive and turn motors
                for (SwerveModule module : swerveModules) {
                    module.getDriveMotor().setVoltage(voltage);
                    module.getTurnMotor().setVoltage(voltage);
                }
            }, null, this)
        );
    }

    public SysIdRoutine getTurnSysId() {
        return new SysIdRoutine(
            new SysIdRoutine.Config(
                null, // Use default ramp rate (1 V/s)
                Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
                null, // Use default timeout (10 s)
                (state) -> SignalLogger.writeString("sysid-test-state-SwerveDrive_turn", state.toString())
            ),
            new SysIdRoutine.Mechanism(voltage -> {
                for (SwerveModule module : swerveModules) {
                    module.getTurnMotor().setVoltage(voltage);
                }
            }, null, this)
        );
    }

    /* ---------------- Commands ---------------- */
    
    public Command setSpeed(Speed speed) {
        return runOnce(() -> {
            this.speed = speed;
        });
    }

    public Command resetEncoders() {
        return runOnce(() -> {
            for (SwerveModule swerveModule: swerveModules) {
                swerveModule.resetDriveEncoder();
            }
        });
    }

    public Command setYawCommand(double degrees) {
        return runOnce(
            () -> setYaw(degrees)
        );
    }

    public Command runStop() {
        return runOnce(this::stop);
    }

    public Command driveAutoReset(){
        return runOnce(()->setYawCommand(getRotation2d().rotateBy(Rotation2d.fromDegrees(0)).getDegrees()));
    }

    /* ---------------- Odometry ---------------- */
    
    public ChassisSpeeds getSpeeds() {//Is this Robot Relative
        return DRIVE_KINEMATICS.toChassisSpeeds(getModuleStates());
    }
    
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[swerveModules.length];
        for (int i = 0; i < swerveModules.length; i++) {
            states[i] = swerveModules[i].getState();
        }
        return states;
    }
    
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[swerveModules.length];
        for (int i = 0; i < swerveModules.length; i++) {
            positions[i] = swerveModules[i].getPosition();
        }
        return positions;
    }

    /**
     * Used to get the heading of the robot in degrees
     * with CLOCKWISE rotation being positive. This
     * value is wrapped to [0,360). ONLY USE THIS METHOD
     * IF YOU KNOW WHAT YOU ARE DOING!
     * 
     * @return the heading of the robot as a double
     */
    public double getHeadingCW() {
        double yaw = -pigeon2.getYaw().getValueAsDouble();

        // Normalize to [0, 360)
        yaw = ((yaw % 360) + 360) % 360;

        return yaw;
    }
    
    /**
     * Used to get the heading of the robot in degrees
     * with COUNTERCLOCKWISE rotation being positive. This
     * value is wrapped to [-180, 180].
     *
     * @return the heading of the robot as a double
     */
    public double getHeading() {
        return Math.IEEEremainder(pigeon2.getYaw().getValueAsDouble(), 360);
    }

    public double getPitch() {
        return Math.IEEEremainder(pigeon2.getPitch().getValueAsDouble(), 360);
    }

    public double getRoll() {
        return Math.IEEEremainder(pigeon2.getRoll().getValueAsDouble(), 360);
    }

    public double getRate() {
        return pigeon2.getAngularVelocityZWorld().getValueAsDouble();
    }

    /**
     * Used to get the heading of the robot in Rotation2d
     * with COUNTERCLOCKWISE rotation being positive. 
     *
     * @return the heading of the robot as a Rotation2d
     */
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(pigeon2.getYaw().getValueAsDouble());
    }

    public Rotation3d getRotation3d() {
        return new Rotation3d(pigeon2.getRoll().getValue(), pigeon2.getPitch().getValue(), pigeon2.getYaw().getValue());
    }
    
    public Pose3d getPose3d() {
        return odometry3d.getPoseMeters();
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public Pose3d getEstimatedPose3d() {
        return poseEstimator3d.getEstimatedPosition();
    }
}

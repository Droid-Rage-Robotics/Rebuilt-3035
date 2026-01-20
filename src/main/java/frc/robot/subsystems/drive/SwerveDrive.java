package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.DroidRageConstants;
import frc.robot.subsystems.drive.SwerveDriveConstants.Speed;
import frc.robot.subsystems.drive.SwerveDriveConstants.SwerveDriveConfig;
import frc.robot.subsystems.drive.SwerveModule.POD;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.LimelightHelpers.PoseEstimate;
import frc.utility.DashboardUtils;
import frc.utility.DashboardUtils.Dashboard;
import frc.utility.encoder.EncoderBase.EncoderDirection;
import frc.utility.motor.MotorBase.Direction;
import lombok.Getter;

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
    
    private final SwerveModule frontRight = SwerveModule.create()
        .withSubsystem(this, POD.FR)
        .withDriveMotor(3,Direction.Forward, true)
        .withTurnMotor(1, Direction.Forward, true)
        .withEncoder(2, SwerveDriveConfig.FRONT_RIGHT_ABSOLUTE_ENCODER_OFFSET_RADIANS::getValue, 
        EncoderDirection.Forward);
        
    private final SwerveModule backRight = SwerveModule.create()
        .withSubsystem(this, POD.BR)
        .withDriveMotor(6, Direction.Forward, true)
        .withTurnMotor(4, Direction.Forward, true)
        .withEncoder(5, SwerveDriveConfig.BACK_RIGHT_ABSOLUTE_ENCODER_OFFSET_RADIANS::getValue,
        EncoderDirection.Forward);

    private final SwerveModule backLeft = SwerveModule.create()
        .withSubsystem(this, POD.BL)
        .withDriveMotor(9, Direction.Forward, true)
        .withTurnMotor(7, Direction.Forward, true)
        .withEncoder(8, SwerveDriveConfig.BACK_LEFT_ABSOLUTE_ENCODER_OFFSET_RADIANS::getValue, 
        EncoderDirection.Forward);
    
    private final SwerveModule frontLeft = SwerveModule.create()
        .withSubsystem(this, POD.FL)
        .withDriveMotor(12, Direction.Forward, true)
        .withTurnMotor(10, Direction.Forward, true)
        .withEncoder(11, SwerveDriveConfig.FRONT_LEFT_ABSOLUTE_ENCODER_OFFSET_RADIANS::getValue, 
        EncoderDirection.Forward);
    
    @Getter private final SwerveModule[] swerveModules = { frontLeft, frontRight, backLeft, backRight };
    
    private final Pigeon2 pigeon2 = new Pigeon2(13, DroidRageConstants.driveCanBus);

    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry (
        DRIVE_KINEMATICS, 
        new Rotation2d(0), 
        getModulePositions()
    );

    private final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
        DRIVE_KINEMATICS, 
        getRotation2d(), 
        getModulePositions(), 
        new Pose2d()
    );

    private volatile Speed speed = Speed.NORMAL;
    private volatile TippingState tippingState = TippingState.NO_TIP_CORRECTION;

    private final Field2d field = new Field2d();
    private final Field2d visionField = new Field2d();

    private final boolean isEnabled;
    private final Vision vision;

    public SwerveDrive(boolean isEnabled, Vision vision) {
        this.isEnabled = isEnabled;
        this.vision=vision;
        
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
        
        DashboardUtils.registerDashboard(this);
    }

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
        SmartDashboard.putData("Drive/Angles", frontLeft);
        SmartDashboard.putData("Drive/Angles", frontRight);
        SmartDashboard.putData("Drive/Angles", backLeft);
        SmartDashboard.putData("Drive/Angles", backRight);
    }

    @Override
    public void alerts() {}

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveDrive");

        builder.addDoubleProperty("Front Left Angle", () -> frontLeft.getTurningPosition(), null);
        builder.addDoubleProperty("Front Left Velocity", () -> frontLeft.getDriveVelocity(), null);

        builder.addDoubleProperty("Front Right Angle", () -> frontRight.getTurningPosition(), null);
        builder.addDoubleProperty("Front Right Velocity", () -> frontRight.getDriveVelocity(), null);

        builder.addDoubleProperty("Back Left Angle", () -> backLeft.getTurningPosition(), null);
        builder.addDoubleProperty("Back Left Velocity", () -> backLeft.getDriveVelocity(), null);

        builder.addDoubleProperty("Back Right Angle", () -> backRight.getTurningPosition(), null);
        builder.addDoubleProperty("Back Right Velocity", () -> backRight.getDriveVelocity(), null);

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

        field.setRobotPose(getPose());
        
        updateVisionOdometry();
    }

    @Override
    public void simulationPeriodic() {
        periodic();
    }

    public void updateVisionOdometry() {
        poseEstimator.update(getRotation2d(), getModulePositions());

        PoseEstimate left = vision.getLeftEstimate();
        PoseEstimate right = vision.getRightEstimate();

        if (left != null && left.tagCount > 0) {
            double dist = vision.closestTagDistance(left);
            double std = vision.distanceToStdDev(dist);
            double stdTheta = Math.toRadians(Math.max(5, dist * 4));

            if (vision.isReasonable(getEstimatedPose(), left.pose))
                poseEstimator.addVisionMeasurement(
                    left.pose,
                    left.timestampSeconds,
                    VecBuilder.fill(std, std, stdTheta)
                );
            }

        if (right != null && right.tagCount > 0) {
            double dist = vision.closestTagDistance(right);
            double std = vision.distanceToStdDev(dist);
            double stdTheta = Math.toRadians(Math.max(5, dist * 4));

            if (vision.isReasonable(getEstimatedPose(), right.pose)) {
                poseEstimator.addVisionMeasurement(
                    right.pose,
                    right.timestampSeconds,
                    VecBuilder.fill(std, std, stdTheta)
                );
            }  
        }

        visionField.setRobotPose(getEstimatedPose());
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        };
    }

    public TippingState getTippingState() {
        return tippingState;
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

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public double getTranslationalSpeed() {
        return speed.getTranslationalSpeed();
    }

    public double getAngularSpeed() {
        return speed.getAngularSpeed();
    }

    public double getForwardVelocity() {
        return (frontLeft.getDriveVelocity() + frontRight.getDriveVelocity()) / 2;
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

    public void stop() {
        for (SwerveModule swerveModule: swerveModules) {
            swerveModule.stop();
        }
    }

    public void setTippingState(TippingState tippingState) {
        this.tippingState = tippingState;
    }

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
    public void setYaw(double degrees){
        pigeon2.setYaw(degrees, 5);
    }

    public Command runStop() {
        return runOnce(this::stop);
    }

    public TrapezoidProfile.Constraints getThetaConstraints() {
        return new TrapezoidProfile.Constraints(
            SwerveDriveConfig.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND.getValue(),
            SwerveDriveConfig.MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED.getValue());
    }

    public Command driveAutoReset(){
        return runOnce(()->setYawCommand(getRotation2d().rotateBy(Rotation2d.fromDegrees(0)).getDegrees()));
    }

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
}

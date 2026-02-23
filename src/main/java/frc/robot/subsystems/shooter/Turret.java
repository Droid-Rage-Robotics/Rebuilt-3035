package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import frc.robot.DroidRageConstants;
import frc.utility.encoder.EncoderConstants;
import frc.utility.motor.MotorConstants;
import frc.utility.motor.MotorConstants.Direction;
import frc.utility.motor.TalonEx;
import frc.utility.template.SubsystemConstants;
import frc.utility.template.SubsystemConstants.EncoderType;
import frc.utility.template.TurretTemplate;

public class Turret extends TurretTemplate { 
    
    Mechanism2d mech = new Mechanism2d(10, 10);
    MechanismLigament2d ligma = new MechanismLigament2d(getName(), getPositionSetpoint(), getPositionError());

    private static Translation2d hubPos = new Translation2d(0, 0);//TODO: Changes based on the alliance
    private static final SubsystemConstants constants = new SubsystemConstants()
        .withConversionFactor(3.0/50.0)
        .withEncoderType(EncoderType.INTEGRATED)
        .withLowerLimit(-135) //Degree
        .withUpperLimit(135) //Degree
        .withName("Turret")
        .withOffset(0)
        .withMainNum(0);
    
    private static final MotorConstants motorConstants = new MotorConstants() 
        .withDeviceId(2)
        .withCANBus(DroidRageConstants.rioCanBus)
        .withDirection(Direction.Forward)
        .withIdleMode(NeutralModeValue.Brake)
        .withSupplyCurrentLimit(70)
        .withStatorCurrentLimit(70);
    
    // private static final EncoderConstants encoderConstants = new EncoderConstants()
    //     .withDeviceId(3)    
    //     .withCANBus(DroidRageConstants.rioCanBus)
    //     .withConversionFactor(1)
    //     .withDirection(SensorDirectionValue.Clockwise_Positive);
    
    public Turret(boolean isEnabled) {
        super(isEnabled, 
            new ProfiledPIDController(1.1597, 0, 0, 
            new TrapezoidProfile.Constraints(1, 1)), 
            new SimpleMotorFeedforward(0.11055, 1.6667, 0.15809), 
            constants, 
            null,            
            motorConstants);

        
    }

    private final NetworkTable driveTable = NetworkTableInstance.getDefault().getTable("Drivetrain");
    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("Turret");

    private final StructSubscriber<Pose3d> pose3dSub = driveTable.getStructTopic("Pose3d", Pose3d.struct).subscribe(new Pose3d());
    private final StructPublisher<Pose3d> pose3dPub  = table.getStructTopic("Pose3d", Pose3d.struct).publish();


    @Override
    public void periodic() {
        super.periodic();
        
        // var pose3d = pose3dSub.get().transformBy(HubShooterMath.ROBOT_TO_TURRET_TRANSFORM);

        // pose3d = pose3d.transformBy(new Transform3d(Translation3d.kZero, new Rotation3d(0, 0, getCurrentAngle().getRadians())));

        // pose3dPub.accept(pose3d);
    }

    

    // public double getTurretGoalAngle(SwerveDrive drive, Pose2d target) { //MIGHT BE WRONG; https://www.chiefdelphi.com/t/turret-tracking-hub-using-odometry/512844/5
    //     // Get robot pose with turret offset
    //     // Pose2d robotPose = drive.getEstimatedPose(); //drive.getPose();
    //     // Apply turret offset to robot pose
    //     Transform2d turretOffset = 
    //     new Transform2d(
    //         Units.inchesToMeters(1),
    //         Units.inchesToMeters(1),
    //         new Rotation2d());
    //     Pose2d turretPose = robotPose.plus(turretOffset);
        
    //     // Calculate vector to target
    //     double dY = target.getY() - turretPose.getY();
    //     double dX = target.getX() - turretPose.getX();
        
    //     // Calculate field-relative angle to target
    //     Rotation2d fieldRelativeAngle = Rotation2d.fromRadians(Math.atan2(dY, dX));
        
    //     // Convert to robot-relative angle
    //     Rotation2d robotRelativeAngle = fieldRelativeAngle.minus(robotPose.getRotation());

    //     // Update angle variables
    //     return robotRelativeAngle.getDegrees();
    //     // m_fieldRelativeAngle = fieldRelativeAngle.getDegrees();
    // }

    // public static double getTurretAngleToHub(
    //     SwerveDrive drive
    // ) {
    //     // Robot position
    //     Pose2d robotPose = drive.getEstimatedPose();
    //     double rx = robotPose.getX();
    //     double ry = robotPose.getY();
    //     double robotHeading = robotPose.getRotation().getRadians();

    //     // Vector from robot to hub
    //     double dx = hubPos.getX() - rx;
    //     double dy = hubPos.getY() - ry;

    //     // Field-relative angle to hub
    //     double angleToHub = Math.atan2(dy, dx);

    //     // Robot-relative turret angle
    //     double turretAngle = angleToHub - robotHeading; //WRONG

    //     // Normalize to [-pi, pi]
    //     return MathUtil.angleModulus(turretAngle);
    // }
}
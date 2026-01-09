package frc.robot.subsystems.carriage;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.DroidRageConstants.Control;
import frc.utility.encoder.AbsoluteDutyEncoderRIO;
import frc.utility.encoder.EncoderBase.EncoderDirection;
import frc.utility.motor.SparkMaxEx;
import frc.utility.motor.MotorBase.Direction;
import frc.utility.motor.MotorBase.ZeroPowerMode;
import frc.utility.template.ArmAbsoluteTemplate;

public class Pivot extends ArmAbsoluteTemplate {
    public static class Constants {
        public static final double MAX_POSITION = 246;//200
        public static final double MIN_POSITION = 100;//100
        public static final double OFFSET = Math.PI;
        // public static final double OFFSET = 0;
    }
    
    private static SparkMaxEx motor = SparkMaxEx.create(27)
        .withDirection(Direction.Forward)
        .withIdleMode(ZeroPowerMode.Brake)
        .withConversionFactor(1)
        .withSubsystem(null)
        .withIsEnabled(true)
        .withSupplyCurrentLimit(50);
    
    private static AbsoluteDutyEncoderRIO encoder = AbsoluteDutyEncoderRIO.create(2)
        .withDirection(EncoderDirection.Forward)
        .withZeroOffset(0.05470208887) // works i guess
        // OLD VALUES //0.9916014247900357 //0.8880354222008856
        .withRange(1);
        // .withSubsystemBase("pivot", Carriage.class.getSimpleName());

    public Pivot(boolean isEnabled) {
        super(
        new SparkMaxEx[]{motor}, 
        new PIDController(4.8, 0, 0), // 2.7 p
        new ArmFeedforward(0.04, 0.21, .25, 0.15),

        // new ArmFeedforward(0.079284, 0.12603, 372.93,
        //                 0.05276), 
        //OLD PROGEJCT - 0.079284, 0.12603, 2.3793, 0.05276
        new TrapezoidProfile.Constraints(0, 0),
        Constants.MAX_POSITION, Constants.MIN_POSITION, Constants.OFFSET, 
        Control.FEEDFORWARD, Carriage.class.getSimpleName(), "Pivot", 0, encoder, isEnabled);

        SmartDashboard.putData(encoder);
    }
    
    
}

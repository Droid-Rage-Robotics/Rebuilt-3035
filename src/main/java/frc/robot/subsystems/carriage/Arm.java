package frc.robot.subsystems.carriage;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.DroidRageConstants.Control;
import frc.utility.encoder.AbsoluteDutyEncoderRIO;
import frc.utility.encoder.EncoderBase.EncoderDirection;
import frc.utility.motor.MotorBase.Direction;
import frc.utility.motor.MotorBase.ZeroPowerMode;
import frc.utility.motor.SparkMaxEx;
import frc.utility.template.ArmAbsoluteTemplate;

public class Arm extends ArmAbsoluteTemplate {
    public static class Constants {
        public static final double MAX_POSITION = 215;
        public static final double MIN_POSITION = 45;
        public static final double OFFSET = Math.PI;
        // public static final double OFFSET = 0;
    }
    
    private static SparkMaxEx motor = SparkMaxEx.create(17)
        .withDirection(Direction.Reversed)
        .withIdleMode(ZeroPowerMode.Brake)
        .withConversionFactor(1)
        .withSubsystem(null)
        .withIsEnabled(true)
        .withSupplyCurrentLimit(50);
    
    // private static SparkAbsoluteEncoderEx encoder = SparkAbsoluteEncoderEx.create(motor)
    //     .withDirection(EncoderDirection.Forward)
    //     .withZeroOffset(0.0574237);
    //     // .withSubsystemBase("arm", Carriage.class.getSimpleName());

    private static final AbsoluteDutyEncoderRIO encoder = AbsoluteDutyEncoderRIO.create(0)
        .withDirection(EncoderDirection.Forward)
        .withZeroOffset(0.006806250170156254)
        .withRange(1);
        
    public Arm(boolean isEnabled) {
        super(
        new SparkMaxEx[]{motor}, 
        new PIDController(3, 0, 0), // p= 2.25
        new ArmFeedforward(0.1, 0.25, 0.1, 0.05),
        // new PIDController(2.9,0,0), //p= 2.25
        // new ArmFeedforward(0.14, 0.25, 0.12, 0.05), 
        new TrapezoidProfile.Constraints(.1, .1),
        Constants.MAX_POSITION, Constants.MIN_POSITION, 
                Constants.OFFSET, 
        Control.FEEDFORWARD, Carriage.class.getSimpleName(),"Arm", 0, encoder, isEnabled);

        SmartDashboard.putData(encoder);
    }
    
}

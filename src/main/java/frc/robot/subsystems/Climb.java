package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.DroidRageConstants.Control;
import frc.utility.motor.MotorBase;
import frc.utility.motor.TalonEx;
import frc.utility.motor.MotorBase.Direction;
import frc.utility.motor.MotorBase.ZeroPowerMode;
import frc.utility.template.ArmTemplate;

public class Climb extends ArmTemplate {
    public static class Constants {
        public static final double MAX_POSITION = 350;
        public static final double MIN_POSITION = 80;
        public static final double OFFSET = Math.PI/2;


    }
    
    public static double hold = 153;//180
    // public static double reset = 90;
    public static double climb = 215;//355
   //Champs CLimb Values:251(q3), 

    // public static boolean pidOn = true;
    // public static double climbMore = 355;
    // public static double climbMoreMOre = 370;

//3 4:1s+
//64
//12
    private static TalonEx motor = TalonEx.create(34)
        .withDirection(Direction.Forward)
        .withIdleMode(ZeroPowerMode.Brake)
        .withConversionFactor( .008)//(125/1)*(48/16); .02 //.008
        //.0166
        .withSubsystem(null)
        .withIsEnabled(true)
        .withSupplyCurrentLimit(120)
        .withStatorCurrentLimit(120);
        // .0605
    // 0.002666666

    // 0.00037333

    // private static SparkAbsoluteEncoderEx encoder = SparkAbsoluteEncoderEx.create(motor)
    //     .withDirection(EncoderDirection.Forward)
    //     .withOffset(0)
    //     .withSubsystemBase("Climb", Climb.class.getSimpleName());
        
    public Climb(boolean isEnabled) {
        super(
        new MotorBase[]{motor}, 
        new PIDController(150,0,0), //kp: 45
        //31
        new ArmFeedforward(0, 0.11, 0.3,0.15), //ks: 0.14 kv:0.1
        new TrapezoidProfile.Constraints(0, 0),
        Constants.MAX_POSITION, Constants.MIN_POSITION, Constants.OFFSET, 
        Control.FEEDFORWARD, "Climb", "Climb", 0, isEnabled);
        SmartDashboard.putData(this.getName() + "/Reset Encoder",runOnce(this::resetEncoder));
        // ComplexWidgetBuilder.create(DisabledCommand.create(runOnce(this::resetEncoder)), "Reset Encoder", this.getName());
        setTargetPosition(90);

    }

    // public Command increaseClimb(){
    //     return setTargetPositionCommand(Math.toDegrees(getTargetPosition())+5);
    // }

    // @Override
    // public void periodic(){
    //     if(pidOn){
    //         super.periodic();
    //     }
    // }
}

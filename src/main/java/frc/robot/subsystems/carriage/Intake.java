package frc.robot.subsystems.carriage;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.DroidRageConstants.Control;
import frc.utility.motor.MotorBase;
import frc.utility.motor.TalonEx;
import frc.utility.motor.MotorBase.Direction;
import frc.utility.motor.MotorBase.ZeroPowerMode;
import frc.utility.template.IntakeTemplate;

public class Intake extends IntakeTemplate {
    private static class Constants {
        public static final double MAX_SPEED = 800;
        public static final double MIN_SPEED = -800;
    }

    private static TalonEx motor = TalonEx.create(31)
        .withDirection(Direction.Reversed)
        .withIdleMode(ZeroPowerMode.Brake)
        .withConversionFactor(1)
        .withSubsystem(null)
        .withIsEnabled(true)
        .withSupplyCurrentLimit(80)//60,50
        .withStatorCurrentLimit(80);

    public Intake(boolean isEnabled) {
        super(
        new MotorBase[]{motor}, 
        // new PIDController(0.1,0,0), //.15
        new PIDController(0.0020637,0,0),
        // new SimpleMotorFeedforward(0.025, 0.01,0.01),
        // new SimpleMotorFeedforward(.0, .6, 0.3),  
        // new SimpleMotorFeedforward(3.98, 0, 0),0.125
        new SimpleMotorFeedforward(0.22808,0.13255,0.034457), // ka 0.034457
        
        new TrapezoidProfile.Constraints(0, 0),
        Constants.MAX_SPEED, Constants.MIN_SPEED, 1,
        Control.FEEDFORWARD, Carriage.class.getSimpleName(), "Intake", 0, isEnabled);
    }

    // public Command setPowerCommand(double power){
    //     return new InstantCommand(()->motor.setPower(power));
    // }
    // public boolean isElementIn() {
    //     // return coralLimitSwitch.get();
    //     isElementInWriter.set(getTargetPosition() - getEncoderPosition() > 40);
    //     return isElementInWriter.get();
    // }
}

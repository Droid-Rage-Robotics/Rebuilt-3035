package frc.utility.template;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.DroidRageConstants.Control;
import frc.utility.encoder.EncoderBase;
import frc.utility.motor.MotorBase;
import frc.utility.motor.SparkMaxEx;

public class ArmAbsoluteTemplate extends ArmTemplate {
    protected EncoderBase encoder;
    public ArmAbsoluteTemplate(
        SparkMaxEx[] motors,
        PIDController controller,
        ArmFeedforward feedforward,
        TrapezoidProfile.Constraints constraints,
        double maxPosition,
        double minPosition,
        double offset,
        Control control,
        String tabName,
        String subsystemName,
        int mainNum,
        EncoderBase encoder,
        boolean isEnabled
    ){
        super(motors, controller, feedforward, constraints,
        maxPosition, minPosition, offset, control, tabName,
        subsystemName, mainNum, isEnabled);
        this.encoder=encoder;

    }

    public ArmAbsoluteTemplate(
        SparkMaxEx[] motors,
        PIDController controller,
        ArmFeedforward feedforward,
        DigitalInput limitSwitch,
        TrapezoidProfile.Constraints constraints,
        double maxPosition,
        double minPosition,
        double offset,
        Control control,
        String tabName,
        String subsystemName,
        int mainNum,
        EncoderBase encoder,
        boolean isEnabled
    ){
        super(motors, controller, feedforward, limitSwitch, constraints,
        maxPosition, minPosition, offset, control, tabName,
        subsystemName, mainNum, isEnabled);
        this.encoder=encoder;

    }

    @Override
    public void periodic() {
        // encoder.periodic();
        switch(control){
            case PID:
                setVoltage(controller.calculate(getEncoderPosition(), controller.getSetpoint()));
                // setVoltage((controller.calculate(getEncoderPosition(), getTargetPosition())) + .37);
                //.37 is kG ^^
                break;
            case FEEDFORWARD:
                setVoltage(controller.calculate(getEncoderPosition(), controller.getSetpoint())
                +feedforward.calculate(getEncoderPosition(),.7)); 
                // + feedforward.calculate(getTargetPosition(), .5)); 
                //ks * Math.signum(velocity) + kg * Math.cos(pos) + kv * velocity + ka * acceleration; ^^
                break;
            case TRAPEZOID_PROFILE:
                current = profile.calculate(0.02, current, goal);

                setVoltage(controller.calculate(getEncoderPosition(), current.position)
                        + feedforward.calculate(current.position, current.velocity));
                break;
            case SYS_ID: break;
        };   
    }

    @Override
    protected void setVoltage(double voltage) {
        // if (!encoder.isConnectedWriter.get()) return;
        for (MotorBase motor: motors) {
            motor.setVoltage(voltage);
        }
    }
    
    @Override
    public double getEncoderPosition() {
        return (Units.rotationsToRadians(encoder.getAbsolutePosition()) + offset) % (Math.PI*2);
    }
}
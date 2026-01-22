package frc.utility.template;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.utility.DashboardUtils;
import frc.utility.DashboardUtils.Dashboard;
import frc.utility.encoder.CANcoderEx;
import frc.utility.encoder.EncoderConstants;
import frc.utility.motor.MotorBase;
import frc.utility.motor.MotorConstants;
import frc.utility.motor.TalonEx;
import frc.utility.template.SubsystemConstants.EncoderType;

public class ArmTemplate extends SubsystemBase implements Dashboard {
    protected final MotorBase[] motors;
    protected final PIDController controller;
    protected final ArmFeedforward feedforward;
    // protected final DigitalInput limitSwitch;
    protected final double maxPosition;
    protected final double minPosition;
    protected final double offset;
    protected final int mainNum;
    protected final TrapezoidProfile profile;
    protected TrapezoidProfile.State current = new TrapezoidProfile.State(0,0); //initial
    protected TrapezoidProfile.State goal = new TrapezoidProfile.State(0,0);
    protected final String name;
    private final CANcoderEx encoder;

    public ArmTemplate(
        PIDController controller,
        ArmFeedforward feedforward,
        TrapezoidProfile.Constraints constraints,
        double maxPosition,
        double minPosition,
        double offset,
        boolean isEnabled,
        SubsystemConstants constants,
        EncoderConstants encoderConstants,
        MotorConstants... motorConstants
    ){
        this.controller=controller;
        this.feedforward=feedforward;
        this.maxPosition=maxPosition;
        this.minPosition=minPosition;
        this.offset=offset;
        this.mainNum=constants.mainNum;
        this.name=constants.name;

        profile = new TrapezoidProfile(constraints);


        if (constants.encoderType==EncoderType.ABSOLUTE||encoderConstants==null) {
            throw new NullPointerException("Encoder constants are required when using an absolute encoder");
        } else if (constants.encoderType==EncoderType.ABSOLUTE) {
            encoder = CANcoderEx.createWithConstants(encoderConstants);
        } else {
            encoder = null;
        }

        // if (constants.hasLimitSwitch)

        this.motors = new MotorBase[motorConstants.length];
        
        for (MotorConstants m_motorConstants : motorConstants) {
            m_motorConstants.subsystem=this;
            m_motorConstants.isEnabled=isEnabled;
        }

        for (int i = 0; i < motorConstants.length; i++) {
            this.motors[i] = TalonEx.createWithConstants(motorConstants[i]);
        }

        DashboardUtils.registerDashboard(this);
    }

    @Override
    public void elasticInit() {
        SmartDashboard.putData(name, this);
    }

    @Override
    public void practiceWriters() {}

    @Override
    public void alerts() {}

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Current Position (Degrees)", () -> Math.toDegrees(getEncoderPosition()), null);
        builder.addDoubleProperty("Current Position (Radians)", this::getEncoderPosition, null);
        builder.addDoubleProperty("Target Position (Degrees)", () -> Math.toDegrees(controller.getSetpoint()), null);
        builder.addDoubleProperty("Target Position (Radians)", controller::getSetpoint, null);
        builder.addDoubleProperty("Applied Voltage", motors[mainNum]::getVoltage, null);
    }

    @Override
    public void periodic() {
        setVoltage(
            controller.calculate(getEncoderPosition(), controller.getSetpoint())
            +feedforward.calculate(1,1)
        ); 
        //ks * Math.signum(velocity) + kg + kv * velocity + ka * acceleration; ^^
    }

    @Override
    public void simulationPeriodic() {
        periodic();
    }
    
    public Command setTargetPositionCommand(double degree){
        return new InstantCommand(()->setTargetPosition(degree));
    }

    /*
     * Use this for initialization
     */
    public void setTargetPosition(double degree) {
        if(degree>maxPosition||degree<minPosition) {
            degree = Math.toDegrees(degree); //Pretty sure this needs to be like this
        };
        controller.setSetpoint(Math.toRadians(degree));
    }

    public double getTargetPosition(){
        return controller.getSetpoint();
    }
    
    /* ---------------- Motor Control ---------------- */
    
    protected void setVoltage(double voltage) {
        for (MotorBase motor: motors) {
            motor.setVoltage(voltage);
        }
    }
    
    public void resetEncoder() {
        for (MotorBase motor: motors) {
            // motor.getEncoder().setPosition(0);
            motor.resetEncoder(0);
        }
    }

    public double getEncoderPosition() {
        // double radian = motors[mainNum].getPosition()+offset;
        // + Constants.OFFSET) % Constants.RADIANS_PER_ROTATION
        // return motors[mainNum].getPosition()+offset;
        return 0;
    }

    public MotorBase getMotor(){
        return motors[mainNum];
    }
    
    public MotorBase[] getAllMotor() {
        return motors;
    }

    public boolean atSetpoint(){
        return controller.atSetpoint();
    }
}

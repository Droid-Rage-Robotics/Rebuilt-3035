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
import frc.robot.DroidRageConstants.Control;
import frc.utility.DashboardUtils;
import frc.utility.DashboardUtils.Dashboard;
import frc.utility.motor.MotorBase;

public class ArmTemplate extends SubsystemBase implements Dashboard {
    protected final MotorBase[] motors;
    protected final PIDController controller;
    protected final ArmFeedforward feedforward;
    protected DigitalInput limitSwitch;
    protected final Control control;
    protected final double maxPosition;
    protected final double minPosition;
    protected final double offset;
    protected final int mainNum;
    protected final TrapezoidProfile profile;
    protected TrapezoidProfile.State current = new TrapezoidProfile.State(0,0); //initial
    protected TrapezoidProfile.State goal = new TrapezoidProfile.State(0,0);
    protected final String name;

    public ArmTemplate(
        MotorBase[] motors,
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
        boolean isEnabled
    ){
        this.motors=motors;
        this.controller=controller;
        this.feedforward=feedforward;
        this.control=control;
        this.maxPosition=maxPosition;
        this.minPosition=minPosition;
        this.offset=offset;
        this.mainNum=mainNum;
        this.name=subsystemName;

        profile = new TrapezoidProfile(constraints);

        for (MotorBase motor: motors) {
            motor.withIsEnabled(isEnabled);
        }

        DashboardUtils.registerDashboard(this);
    }

    public ArmTemplate(
        MotorBase[] motors,
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
        boolean isEnabled
    ){
        this.motors=motors;
        this.controller=controller;
        this.feedforward=feedforward;
        this.limitSwitch=limitSwitch;
        this.control=control;
        this.maxPosition=maxPosition;
        this.minPosition=minPosition;
        this.offset=offset;
        this.mainNum=mainNum;
        this.name=subsystemName;

        for (MotorBase motor: motors) {
            motor.withIsEnabled(isEnabled);
        }

        profile = new TrapezoidProfile(constraints);

        DashboardUtils.registerDashboard(this);
    }

    @Override
    public void elasticInit() {
        SmartDashboard.putData(name, this);
    }

    @Override
    public void practiceWriters() {
        
    }

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
        switch(control){
            case PID:
                setVoltage(controller.calculate(getEncoderPosition(), controller.getSetpoint()));
                // setVoltage((controller.calculate(getEncoderPosition(), getTargetPosition())) + .37);
                //.37 is kG ^^
                break;
            case FEEDFORWARD:
                setVoltage(controller.calculate(getEncoderPosition(), controller.getSetpoint())
                +feedforward.calculate(1,1)); 
                //ks * Math.signum(velocity) + kg + kv * velocity + ka * acceleration; ^^
                break;
            case TRAPEZOID_PROFILE:
                goal = new TrapezoidProfile.State(controller.getSetpoint(),01);
                current = profile.calculate(0.02, current, goal);

                setVoltage(controller.calculate(getEncoderPosition(), current.position)
                        + feedforward.calculate(current.position, current.velocity));
                break;

            case SYS_ID: break;
        };        
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

    @Override
    public void alerts() {}
}

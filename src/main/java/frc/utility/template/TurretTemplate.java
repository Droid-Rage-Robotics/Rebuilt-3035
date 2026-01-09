package frc.utility.template;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DroidRageConstants.Control;
import frc.utility.DashboardUtils.Dashboard;
import frc.utility.motor.MotorBase;

public class TurretTemplate extends SubsystemBase implements Dashboard {
    protected final MotorBase[] motors;
    protected final PIDController controller;
    protected final SimpleMotorFeedforward feedforward;
    protected final Control control;
    protected final double maxPosition;
    protected final double minPosition;
    protected final double offset;
    protected final int mainNum;
    protected final String name;
    protected final TrapezoidProfile profile;
    protected TrapezoidProfile.State current = new TrapezoidProfile.State(0,0); //initial
    protected TrapezoidProfile.State goal = new TrapezoidProfile.State(0,0);

    public TurretTemplate(
        MotorBase[] motors,
        PIDController controller,
        SimpleMotorFeedforward feedforward,
        TrapezoidProfile.Constraints constraints,
        double maxPosition,
        double minPosition,
        double offset,
        Control control,
        String tabName,
        String subsystemName,
        int mainNum,
        String name
    ){
        this.motors=motors;
        this.controller=controller;
        this.feedforward=feedforward;
        this.control=control;
        this.maxPosition=maxPosition;
        this.minPosition=minPosition;
        this.offset=offset;
        this.mainNum=mainNum;
        this.name=name;

        profile = new TrapezoidProfile(constraints);
    }

    @Override
    public void elasticInit() {
        SmartDashboard.putData(name,this);
        SmartDashboard.putData(name + "/Reset Encoder", runOnce(this::resetEncoder));
    }

    @Override
    public void practiceWriters() {}

    @Override
    public void alerts() {}

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Current Position (Degrees)", () -> Math.toDegrees(getEncoderPosition()), null);
        builder.addDoubleProperty("Current Position (Radians)", this::getEncoderPosition, null);
        builder.addDoubleProperty("Target Position (Degrees)", () -> Math.toDegrees(getTargetPosition()), null);
        builder.addDoubleProperty("Target Position (Radians)", controller::getSetpoint, null);
        builder.addDoubleProperty("Applied Voltage", this::getVoltage, null);
    }

    @Override
    public void periodic() {
        // targetDegreeWriter.set(Math.toDegrees(controller.getSetpoint()));
        // targetRadianWriter.set(controller.getSetpoint());
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
        double radian = motors[mainNum].getPosition()+offset;
        // + Constants.OFFSET) % Constants.RADIANS_PER_ROTATION
        return radian;
    }

    public double getVoltage() {
        return motors[mainNum].getVoltage();
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

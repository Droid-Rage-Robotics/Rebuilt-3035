package frc.utility.template;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.utility.DashboardUtils.Dashboard;
import frc.utility.motor.MotorBase;

public class TurretTemplate extends SubsystemBase implements Dashboard {
    protected final MotorBase[] motors;
    protected final ProfiledPIDController controller;
    protected final double maxPosition;
    protected final double minPosition;
    protected final double offset;
    protected final double conversionFactor;
    protected final int mainNum;
    protected final boolean isEnabled;

    public TurretTemplate(
        MotorBase[] motors,
        ProfiledPIDController controller,
        double maxPosition,
        double minPosition,
        double conversionFactor,
        double offset,
        int mainNum,
        boolean isEnabled
    ){
        this.motors=motors;
        this.controller=controller;
        this.conversionFactor=conversionFactor;
        this.maxPosition=maxPosition;
        this.minPosition=minPosition;
        this.offset=offset;
        this.mainNum=mainNum;
        this.isEnabled=isEnabled;

        for (MotorBase motor: motors) {
            motor.withIsEnabled(isEnabled);
        }
    }

    @Override
    public void elasticInit() {
        SmartDashboard.putData(this.getName() ,this);
        SmartDashboard.putData(this.getName() + "/Reset Encoder", runOnce(this::resetEncoder));
    }

    @Override
    public void practiceWriters() {}

    @Override
    public void alerts() {}

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Goal Position", this::getGoalPosition, null);
        builder.addDoubleProperty("Current Position", this::getPosition, null);
        builder.addDoubleProperty("Position Setpoint", this::getPositionSetpoint, null);
        builder.addDoubleProperty("Velocity Setpoint", this::getVelocitySetpoint, null);
        builder.addDoubleProperty("Current Velocity", this::getVelocity, null);
        builder.addDoubleProperty("Applied Voltage", this::getVoltage, null);
        // builder.addDoubleProperty("Calculated Voltage", () -> calculatedVoltage, null);
        // builder.addDoubleProperty("Calculated FF", () -> calculatedFF, null);
        // builder.addDoubleProperty("Calculated PID", () -> calculatedPID, null);
        builder.addDoubleProperty("Position Error", controller::getPositionError, null);
    }

    @Override
    public void periodic() {

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
        controller.setGoal(Math.toRadians(degree));
    }

    public double getGoalPosition(){
        return controller.getGoal().position;
    }

    public double getVelocitySetpoint() {
        return controller.getSetpoint().velocity;
    }

    public double getPositionSetpoint() {
        return controller.getSetpoint().position;
    }

    public double getPosition() {
        return motors[mainNum].getPosition() * conversionFactor;
    }

    public double getVelocity() {
        return motors[mainNum].getVelocity() * conversionFactor;
    }

    protected void setVoltage(double voltage) {
        for (MotorBase motor: motors) {
            motor.setVoltage(voltage);
        }
    }
    
    public void resetEncoder() {
        for (MotorBase motor: motors) {
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

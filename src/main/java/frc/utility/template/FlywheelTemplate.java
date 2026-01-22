package frc.utility.template;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.utility.DashboardUtils;
import frc.utility.DashboardUtils.Dashboard;
import frc.utility.motor.MotorBase;
import frc.utility.motor.MotorConstants;
import frc.utility.motor.TalonEx;

public class FlywheelTemplate extends SubsystemBase implements Dashboard {
    private final MotorBase[] motors;
    private final PIDController controller;
    private final SimpleMotorFeedforward feedforward;
    private final double maxSpeed;
    private final double minSpeed;
    private final double conversionFactor;
    private final int mainNum;
    private final String name;

    public FlywheelTemplate(
        int mainNum,
        PIDController controller,
        SimpleMotorFeedforward feedforward,
        double maxSpeed,
        double minSpeed,
        double conversionFactor,
        String name,
        boolean isEnabled,
        MotorConstants...motorConstants
    ){
        this.controller=controller;
        this.feedforward=feedforward;
        this.maxSpeed=maxSpeed;
        this.minSpeed=minSpeed;
        this.conversionFactor=conversionFactor;
        this.mainNum=mainNum;
        this.name=name;

        this.motors = new MotorBase[motorConstants.length];
        
        for (MotorConstants constants : motorConstants) {
            constants.subsystem=this;
            constants.isEnabled=isEnabled;
        }

        for (int i = 0; i < motorConstants.length; i++) {
            this.motors[i] = TalonEx.createWithConstants(motorConstants[i]);
        }

        DashboardUtils.registerDashboard(this);
    }

    /* ---------------- Dashboard ---------------- */
    
    @Override
    public void elasticInit() {
        SmartDashboard.putData(name, this);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Target Speed", controller::getSetpoint, null);
        builder.addDoubleProperty("Current Speed", this::getVelocity, null);
        builder.addDoubleProperty("Applied Voltage", this::getVoltage, null);
    }

    @Override
    public void practiceWriters() {}

    @Override
    public void alerts() {}

    /* ---------------- Periodic Control Loop ---------------- */
    
    @Override
    public void periodic() {
        setVoltage(
            controller.calculate(getVelocity(), controller.getSetpoint())
            +feedforward.calculate(controller.getSetpoint()));
    }

    @Override
    public void simulationPeriodic() {
        periodic();
    }

    /* ---------------- Commands ---------------- */

    public Command setTargetVelocityCommand(double target){
        return Commands.sequence(
            new InstantCommand(()->setTargetVelocity(target))
        );
        // return new InstantCommand(()->setTargetPosition(target));
    }

    /* ---------------- Manual Goal Control ---------------- */

    public void setTargetVelocity(double target) {
        double clamped = MathUtil.clamp(
            target, 
            minSpeed, 
            maxSpeed);
        
        controller.setSetpoint(clamped);
    }

    public double getTargetVelocity(){
        return controller.getSetpoint();
    }

    /* ---------------- Sensor Access ---------------- */

    public double getVelocity() {
        return motors[mainNum].getVelocity() * conversionFactor;
    }
    
    public double getVoltage() {
        return motors[mainNum].getVoltage();
    }

    /* ---------------- Motor Control ---------------- */
    
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

    /* ---------------- Utility ---------------- */

    public MotorBase getMotor() {
        return motors[mainNum];
    }

    public MotorBase[] getAllMotor() {
        return motors;
    }

    public boolean atSetpoint() {
        return controller.atSetpoint();
    }

}

// isElementIn = this::(getTargetPosition() - getEncoderPosition() > 40);

    // private final Sendable isElementIn =  new Sendable() {
    //     @Override
    //     public void initSendable(SendableBuilder builder) {
    //         builder.setSmartDashboardType("Boolean Box");
    //         builder.addBooleanProperty("Is Element In", () -> (getTargetPosition() - getVelocity() > 40), null);
    //     }
    // };
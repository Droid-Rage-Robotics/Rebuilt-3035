package frc.utility.template;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DroidRageConstants.Control;
import frc.utility.DashboardUtils;
import frc.utility.DashboardUtils.Dashboard;
import frc.utility.motor.MotorBase;

public class IntakeTemplate extends SubsystemBase implements Dashboard {
    private final MotorBase[] motors;
    private final PIDController controller;
    private final SimpleMotorFeedforward feedforward;
    private final Control control;
    private final double maxSpeed;
    private final double minSpeed;
    private final double conversionFactor;
    private final int mainNum;
    private final String name;
    private final TrapezoidProfile profile;
    private TrapezoidProfile.State current = new TrapezoidProfile.State(0,0); //initial
    private final TrapezoidProfile.State goal = new TrapezoidProfile.State(0,0);


    public IntakeTemplate(
        MotorBase[] motors,
        PIDController controller,
        SimpleMotorFeedforward feedforward,
        TrapezoidProfile.Constraints constraints,
        double maxSpeed,
        double minSpeed,
        double conversionFactor,
        Control control,
        String tabName,
        String name,
        int mainNum,
        boolean isEnabled
    ){
        this.motors=motors;
        this.controller=controller;
        this.feedforward=feedforward;
        this.control=control;
        this.maxSpeed=maxSpeed;
        this.minSpeed=minSpeed;
        this.conversionFactor=conversionFactor;
        this.mainNum=mainNum;
        this.name=name;

        for (MotorBase motor: motors) {
            motor.withIsEnabled(isEnabled);
        }

        profile = new TrapezoidProfile(constraints);

        DashboardUtils.registerDashboard(this);
        
    }

    @Override
    public void elasticInit() {
        SmartDashboard.putData(name, this);
        SmartDashboard.putData("Is Element In", isElementIn); 
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Target Speed", controller::getSetpoint, null);
        builder.addDoubleProperty("Current Speed", this::getVelocity, null);
        builder.addDoubleProperty("Applied Voltage", motors[mainNum]::getVoltage, null);
    }

    @Override
    public void practiceWriters() {}
    // isElementIn = this::(getTargetPosition() - getEncoderPosition() > 40);

    private final Sendable isElementIn =  new Sendable() {
        @Override
        public void initSendable(SendableBuilder builder) {
            builder.setSmartDashboardType("Boolean Box");
            builder.addBooleanProperty("Is Element In", () -> (getTargetPosition() - getVelocity() > 40), null);
        }
    };

    @Override
    public void periodic() {
        switch(control){
            case PID:
                setVoltage(controller.calculate(getVelocity(), controller.getSetpoint()));
                // setVoltage((controller.calculate(getEncoderPosition(), getTargetPosition())) + .37);
                //.37 is kG ^^
                break;
            // case FEEDFORWARD:
            //     setVoltage(controller.calculate(getEncoderPosition(), controller.getSetpoint())
            //     +feedforward.calculate(1,1)); //To Change #
            //     // calculateWithVelocities
            //     //ks * Math.signum(velocity) + kg + kv * velocity + ka * acceleration; ^^
            //     break;
            case FEEDFORWARD:
                setVoltage(
                    controller.calculate(getVelocity(), controller.getSetpoint())
                    +feedforward.calculate(controller.getSetpoint())); //To Change #
                    // +feedforward.calculateWithVelocities(1,1)); //To Change #

                break;
            // case TRAPEZOID_PROFILE:
            //     current = profile.calculate(0.02, current, goal);

            //     setVoltage(controller.calculate(getEncoderPosition(), current.position)
            //             + feedforward.calculate(current.position, current.velocity));
            //     break;
            case TRAPEZOID_PROFILE:
                TrapezoidProfile.State next = profile.calculate(0.02, current, goal);

                double ff = feedforward.calculateWithVelocities(current.velocity, next.velocity);
                double pid = controller.calculate(getVelocity(), controller.getSetpoint());

                setVoltage(ff + pid);
                current = next;
                break;
            case SYS_ID: break;
        };        
    }

    @Override
    public void simulationPeriodic() {
        periodic();
    }

    public Command setTargetPositionCommand(double target){
        return Commands.sequence(
            new InstantCommand(()->setTargetPosition(target))
        );
        // return new InstantCommand(()->setTargetPosition(target));
    }

    /*
     * Use this for initialization
     */
    public void setTargetPosition(double target) {
        if(target>maxSpeed||target<minSpeed) return;
        controller.setSetpoint(target);
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
            motor.resetEncoder(0);
        }
    }

    public double getVelocity() {
        return motors[mainNum].getVelocity() * conversionFactor;
    }

    public MotorBase getMotor() {
        return motors[mainNum];
    }

    public MotorBase[] getAllMotor() {
        return motors;
    }

    @Override
    public void alerts() {}
}
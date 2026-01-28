package frc.utility.template;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
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
    private final TalonFXSimState[] motorSimStates;

    private final PIDController controller;
    private final SimpleMotorFeedforward feedforward;
    private final double maxSpeed;
    private final double minSpeed;
    private final double conversionFactor;
    private final int mainNum;
    private final String name;

    private final FlywheelSim simulation;

    private final LinearSystem<N1, N1, N1> flywheelSystem;

    private final DCMotor gearbox;

    public FlywheelTemplate(
        boolean isEnabled,
        PIDController controller,
        SimpleMotorFeedforward feedforward,
        SubsystemConstants constants,
        MotorConstants...motorConstants
    ){
        this.controller=controller;
        this.feedforward=feedforward;
        this.maxSpeed=constants.upperLimit;
        this.minSpeed=constants.lowerLimit;
        this.conversionFactor=constants.conversionFactor;
        this.mainNum=constants.mainNum;
        this.name=constants.name;

        this.motors = new MotorBase[motorConstants.length];
        
        for (MotorConstants m_motorConstants : motorConstants) {
            m_motorConstants.subsystem=this;
            m_motorConstants.isEnabled=isEnabled;
        }

        for (int i = 0; i < motorConstants.length; i++) {
            this.motors[i] = TalonEx.createWithConstants(motorConstants[i]);
        }

        this.motorSimStates = new TalonFXSimState[motors.length];

        for (int i = 0; i < motors.length; i++) {
            this.motorSimStates[i] = motors[i].getSimState();
        }

        for (TalonFXSimState simState : motorSimStates) {
            simState.setMotorType(motorConstants[mainNum].motorType);
        }

        gearbox = switch (motorConstants[mainNum].motorType) {
            case KrakenX44 -> DCMotor.getKrakenX44(motorConstants.length);
            case KrakenX60 -> DCMotor.getKrakenX60(motorConstants.length);
        };
        
        flywheelSystem =  LinearSystemId.createFlywheelSystem(
            gearbox, 0.001, constants.gearRatio
        );
        

        simulation = new FlywheelSim(flywheelSystem, gearbox);

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
        double loopTime = 0.020;

        for (TalonFXSimState simState : motorSimStates) {
            simState.setSupplyVoltage(RobotController.getBatteryVoltage());
            simState.setRotorVelocity(simulation.getAngularVelocity());
                
        }

        simulation.setInputVoltage(motorSimStates[mainNum].getMotorVoltage());
        // Next, we update it. The standard loop time is 20ms.
        simulation.update(loopTime);
        
        // SimBattery estimates loaded battery voltages
        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(simulation.getCurrentDrawAmps()));
  
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
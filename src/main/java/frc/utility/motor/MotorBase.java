package frc.utility.motor;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.utility.DashboardUtils.Dashboard;

public abstract class MotorBase implements Dashboard {
    /**
     * Determines the direction that the motor will
     * rotate when given a positive control request
     */
    public enum Direction {
        /**
         * Clockwise rotation is positive for the motor.
         */
        Forward,
        /**
         * Counterclockwise rotation is positive for the
         * motor.
         */
        Reversed,
    }

    /**
     * Determines motor behavior when not recieving
     * commands for voltage/power
     */
    public enum ZeroPowerMode {
        /**
         * Motor will resist movement when not recieving
         * commands for voltage/power.
         */
        Brake,

        /**
         * Motor will allow free movement when not recieving
         * commands for voltage/power.
         */
        Coast,
    }

    public abstract void setVoltage(double voltage);
    public abstract void setVoltage(Voltage voltage);
    public abstract void setPower(double value);
    public abstract double getVelocity();
    public abstract double getPosition();
    public abstract int getDeviceId();
    public abstract double getTemp();
    public abstract Subsystem getSubsystem();
    public abstract double getVoltage();
    public abstract void resetEncoder(double value);
    public abstract void stop();
    public abstract MotorBase withIsEnabled(boolean isEnabled);

    private final Alert tempAlert = new Alert("Temperature Warning: Motor " + getDeviceId() +" at " + getSubsystem(), AlertType.kWarning);

    @Override
    public void alerts() {
        if (getTemp() < 35) {
            tempAlert.set(true);
        } else {
            tempAlert.set(false);
        }
    }
    
    @Override
    public void elasticInit() {
        
    }

    @Override
    public void practiceWriters() {
        
    }
}

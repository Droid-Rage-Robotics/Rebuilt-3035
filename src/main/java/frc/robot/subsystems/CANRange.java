package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CANRange extends SubsystemBase {

    private CANrange range = new CANrange(0);
    // private CustomParamsConfigs name;
    private CANrangeConfiguration thing;
    
    public CANRange() {
        // Initialize CANRange
        range.getConfigurator().apply(thing);
    }

    public void periodic() {
        // Code to run every loop
    }

    public double getDistance() {
        // Get distance from sensor
        return range.getDistance().getValueAsDouble();
    }

    public boolean isConnected(){
        return range.isConnected();
    }
}

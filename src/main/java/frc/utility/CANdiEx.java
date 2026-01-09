package frc.utility;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.hardware.CANdi;

import frc.robot.DroidRageConstants;

public class CANdiEx {
    private final CANdi candi;
    private final CANdiConfiguration config;
    private final int deviceId;

    private CANdiEx(int deviceId, CANBus canBus) {
        this.deviceId=deviceId;
        this.candi = new CANdi(deviceId, canBus);
        this.config = new CANdiConfiguration();
    }

    public static CANdiEx create(int deviceId, CANBus canBus) {
        return new CANdiEx(deviceId, canBus);
    }

    public static CANdiEx create(int deviceId) {
        return new CANdiEx(deviceId, DroidRageConstants.rioCanBus);
    }
}
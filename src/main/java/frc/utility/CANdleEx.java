package frc.utility;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.hardware.CANdle;

import frc.robot.DroidRageConstants;

public class CANdleEx {
    private final CANdle candle;
    private final CANdleConfiguration config;
    private final int deviceId;

    private CANdleEx(int deviceId, CANBus canBus) {
        this.deviceId=deviceId;
        this.candle = new CANdle(deviceId, canBus);
        this.config = new CANdleConfiguration();
    }

    public static CANdleEx create(int deviceId, CANBus canBus) {
        return new CANdleEx(deviceId, canBus);
    }

    public static CANdleEx create(int deviceId) {
        return new CANdleEx(deviceId, DroidRageConstants.rioCanBus);
    }
}

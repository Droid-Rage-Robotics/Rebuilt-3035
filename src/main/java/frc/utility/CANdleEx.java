package frc.utility;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.Enable5VRailValue;
import com.ctre.phoenix6.signals.LossOfSignalBehaviorValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;

import edu.wpi.first.wpilibj.util.Color;

import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;

import frc.robot.DroidRageConstants;

public class CANdleEx {
    private final CANdle candle;
    private final CANdleConfiguration config;
    private final int deviceId;

    private CANdleEx(int deviceId, CANBus canBus) {
        this.deviceId = deviceId;
        this.candle = new CANdle(deviceId, canBus);
        this.config = new CANdleConfiguration();
        config.statusLedOffWhenActive = true;
        config.disableWhenLOS = false;
        config.stripType = LEDStripType.GRB;
        config.brightnessScalar = 0.1;
        config.vBatOutputMode = VBatOutputMode.Modulated;

        config.CANdleFeatures.Enable5VRail = Enable5VRailValue.Enabled;
        config.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Enabled;
        config.LED.LossOfSignalBehavior = LossOfSignalBehaviorValue.DisableLEDs;
        config.LED.StripType = StripTypeValue.RGB;
        config.LED.BrightnessScalar = 0.9;
        candle.getConfigurator().apply(config);

    }

    public static CANdleEx create(int deviceId, CANBus canBus) {
        return new CANdleEx(deviceId, canBus);
    }

    public static CANdleEx create(int deviceId) {
        return new CANdleEx(deviceId, DroidRageConstants.rioCanBus);
    }

    public void efsd() {
        candle.setControl(
                new RainbowAnimation(8, 10)
                        .withDirection(AnimationDirectionValue.Forward));
        new SingleFadeAnimation(8, 16)
                .withColor(new RGBWColor(Color.kAntiqueWhite));

    }
}

package frc.utility;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.RgbFadeAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.controls.TwinkleAnimation;
import com.ctre.phoenix6.controls.TwinkleOffAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.Enable5VRailValue;
import com.ctre.phoenix6.signals.LossOfSignalBehaviorValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.DroidRageConstants;

public class CANdleEx {
    private final CANdle candle;
    private final CANdleConfiguration config;
    private final int deviceId;
    private final int totalLength;

    public enum AnimationTypes {
        ColorFlow,
        Fire,
        Larson,
        Rainbow,
        RgbFade,
        SingleFade,
        Strobe,
        Twinkle,
        TwinkleOff,
        SetAll
    }
    private CANdleEx(int deviceId, int LEDLength, CANBus canBus) {
        this.deviceId = deviceId;
        this.candle = new CANdle(deviceId, canBus);
        this.config = new CANdleConfiguration();
        totalLength = LEDLength+7;

        config.CANdleFeatures.Enable5VRail = Enable5VRailValue.Enabled;
        config.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Enabled;
        config.LED.LossOfSignalBehavior = LossOfSignalBehaviorValue.DisableLEDs;
        config.LED.StripType = StripTypeValue.RGB;
        config.LED.BrightnessScalar = 0.9;
        candle.getConfigurator().apply(config);

    }

    public static CANdleEx create(int deviceId, int LEDLength, CANBus canBus) {
        return new CANdleEx(deviceId, LEDLength, canBus);
    }

    public static CANdleEx create(int deviceId, int LEDLength) {
        return new CANdleEx(deviceId, LEDLength, DroidRageConstants.rioCanBus);
    }

    /************* Functions for Color Set Up *************/

    // Animation that fades into and out of a specified color
    public void setSingleFadeColor(Color color) {
        candle.setControl(
            new SingleFadeAnimation(8, totalLength)
                .withColor(new RGBWColor(color)));
    }

    // Animation that gradually lights the entire LED strip one LED at a time
    public void setColorFlow(Color color, AnimationDirectionValue direction) {
        candle.setControl(
            new ColorFlowAnimation(8, totalLength)
                .withDirection(direction)
                .withColor(new RGBWColor(color)));
    }

    // Animation that looks similar to a flame flickering -Might REMOVE
    public void setFire(AnimationDirectionValue direction){
        candle.setControl(
            new FireAnimation(8, totalLength)
                .withDirection(direction));
    }

    // Animation that bounces a pocket of light across the LED strip -Might REMOVE
    public void setLarson(Color color){
        candle.setControl(
            new LarsonAnimation(8, totalLength)
                .withColor(new RGBWColor(color)));
    }

    // Animation that creates a rainbow throughout all the LEDs
    public void setRainbow(AnimationDirectionValue direction) {
        candle.setControl(
            new RainbowAnimation(8, totalLength)
                .withDirection(direction));
    }

    //Animation that fades all the LEDs of a strip simultaneously between Red, Green, and Blue -Might REMOVE
    public void setRGBFade(Color color) {
        candle.setControl(
            new RgbFadeAnimation(8, totalLength));
    }

    // Sets LEDs to a solid color
    public void setAll(Color color, int startIndex, int endIndex) {
        candle.setControl(
            new SolidColor(startIndex, endIndex)
                .withColor(new RGBWColor(color)));
    }

    public void setAlternating(Color colorOne, Color colorTwo) {
        for(int i = 8; i < totalLength; i++) {
            candle.setControl(
                new SolidColor(i, i+1)
                .withColor(new RGBWColor(colorOne)));
        }
    }

    // Animation that strobes the LEDs a specified color
    public void setStrobe(Color color) {
        candle.setControl(
            new StrobeAnimation(8, totalLength)
                .withColor(new RGBWColor(color)));
    }

    // Animation that randomly turns LEDs on and off to a certain color
    public void setTwinkle(Color color) {
        candle.setControl(
            new TwinkleAnimation(8, totalLength)
                .withColor(new RGBWColor(color)));
    }

    // Animation that randomly turns on LEDs until it reaches the maximum count, and then turns them all off
    public void setTwinkleOff(Color color) {
        candle.setControl(
            new TwinkleOffAnimation(8, totalLength)
                .withColor(new RGBWColor(color)));
    }
}

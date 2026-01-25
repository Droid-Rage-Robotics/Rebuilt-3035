package frc.robot.subsystems;


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
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.utility.CANdleEx;

public class Light extends SubsystemBase {
	// 0-7: On Board LEDs
	// 8-#: LED Strip
	public final Color red = Color.kRed,
		batteryBlue = Color.kMidnightBlue, //Battery Low
		orange = Color.kOrange, //Scoring Shift  //TODO:Fix Color
		green = Color.kGreen, //Ready to Shoot
		white = Color.kWhite, //Opponent Shift
		DRBlue = new Color(68,185,243),
		DROrange = new Color(255, 130, 0);
	public int LEDlength = 20,
		startShootNum = 0,
		endShootNum = LEDlength/2,
		startShiftNum = endShootNum+1,
		endShiftNum = LEDlength+8;
	private final CANdleEx candle;
	public Light(int deviceID) {
		candle = CANdleEx.create(deviceID, LEDlength);
	}

	@Override
	public void periodic() {
	}

	@Override
	public void simulationPeriodic() {
		periodic();
	}

	// Animation that fades into and out of a specified color
    public void setSingleFadeColor(Color color) {
        candle.setSingleFadeColor(color);
    }

    // Animation that gradually lights the entire LED strip one LED at a time
    public void setColorFlow(Color color, AnimationDirectionValue direction) {
        candle.setColorFlow(color, direction);
    }

    // Animation that looks similar to a flame flickering -Might REMOVE
    public void setFire(AnimationDirectionValue direction){
        candle.setFire(direction);
    }

    // Animation that bounces a pocket of light across the LED strip -Might REMOVE
    public void setLarson(Color color){
        candle.setLarson(color);
    }

    // Animation that creates a rainbow throughout all the LEDs
    public void setRainbow(AnimationDirectionValue direction) {
        candle.setRainbow(direction);
    }

    //Animation that fades all the LEDs of a strip simultaneously between Red, Green, and Blue -Might REMOVE
    public void setRGBFade(Color color) {
        candle.setRGBFade(color);
    }

    // Sets LEDs to a solid color
    public void setAll(Color color) {
        candle.setAll(color, 8, endShiftNum);
    }
	
	public void setShootHalf(Color color) {
		candle.setAll(color, startShootNum, endShootNum);
	}
	
	public void setShiftHalf(Color color) {
		candle.setAll(color, startShiftNum, endShiftNum);
	}

	public void setAlternating(Color colorOne, Color colorTwo) {	//TODO:Test
		candle.setAlternating(colorOne,colorTwo);
	}

    // Animation that strobes the LEDs a specified color
    public void setStrobe(Color color) {
        candle.setStrobe(color);
    }

    // Animation that randomly turns LEDs on and off to a certain color
    public void setTwinkle(Color color) {
        candle.setTwinkle(color);
    }

    // Animation that randomly turns on LEDs until it reaches the maximum count, and then turns them all off
    public void setTwinkleOff(Color color) {
        candle.setTwinkleOff(color);
    }
}

package frc.utility.encoder;

public abstract class EncoderBase {
    public enum EncoderRange {
        ZERO_TO_ONE, //Typically ALWAYS uses 0 to 1
        PLUS_MINUS_HALF
    }
    
    public enum EncoderDirection {
        Forward,
        Reversed,
    }

    public abstract double getAbsolutePosition();
    public abstract int getDeviceId();
    public abstract double getVelocity();
}

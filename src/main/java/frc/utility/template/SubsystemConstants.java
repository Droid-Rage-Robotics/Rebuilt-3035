package frc.utility.template;

public class SubsystemConstants {
    public enum EncoderType {
        INTEGRATED,
        ABSOLUTE
    }
    
    public String name;
    public EncoderType encoderType;
    public int mainNum;

    public boolean hasLimitSwitch;
}

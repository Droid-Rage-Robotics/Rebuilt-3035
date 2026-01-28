package frc.utility.template;

public final class SubsystemConstants {
    public enum EncoderType {
        INTEGRATED,
        ABSOLUTE
    }
    
    public String name;
    public EncoderType encoderType;
    public int mainNum;
    public double upperLimit;
    public double lowerLimit;
    public double conversionFactor;
    public double offset;

    public double gearRatio;

    public boolean hasLimitSwitch;

    public SubsystemConstants withEncoderType(EncoderType value) {
        this.encoderType=value;
        return this;
    }

    public SubsystemConstants withGearRatio(double value) {
        this.gearRatio=value;
        return this;
    }

    public SubsystemConstants withName(String name) {
        this.name=name;
        return this;
    }

    public SubsystemConstants withMainNum(int mainNum) {
        this.mainNum=mainNum;
        return this;
    }

    public SubsystemConstants withUpperLimit(double upperLimit) {
        this.upperLimit=upperLimit;
        return this;
    }

    public SubsystemConstants withLowerLimit(double lowerLimit) {
        this.lowerLimit=lowerLimit;
        return this;
    }

    public SubsystemConstants withConversionFactor(double conversionFactor) {
        this.conversionFactor=conversionFactor;
        return this;
    }

    public SubsystemConstants withOffset(double offset) {
        this.offset=offset;
        return this;
    }
}

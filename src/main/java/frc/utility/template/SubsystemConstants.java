package frc.utility.template;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

public final class SubsystemConstants {
    public enum EncoderType {
        INTEGRATED,
        ABSOLUTE,
        EXTERNAL
    }
    
    public String name;
    public EncoderType encoderType;
    public int mainNum;
    public Distance maxDistance;
    public Distance minDistance;
    public Angle maxAngle;
    public Angle minAngle;
    public AngularVelocity maxVelocity;
    public AngularVelocity minVelocity;
    public double conversionFactor;
    public double offset;

    public double gearRatio;

    public boolean hasLimitSwitch;
    
    public double length;
    public double width;

    public SubsystemConstants withEncoderType(EncoderType value) {
        this.encoderType=value;
        return this;
    }

    public SubsystemConstants withGearRatio(double value) {
        this.gearRatio=value;
        return this;
    }

    public SubsystemConstants withLength(double value) {
        this.length=value;
        return this;
    }
    public SubsystemConstants withWidth(double value) {
        this.width=value;
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

    public SubsystemConstants withMaxVelocity(AngularVelocity max) {
        this.maxVelocity=max;
        return this;
    }

    public SubsystemConstants withMinVelocity(AngularVelocity min) {
        this.minVelocity=min;
        return this;
    }

    public SubsystemConstants withMaxAngle(Angle max) {
        this.maxAngle=max;
        return this;
    }

    public SubsystemConstants withMinAngle(Angle min) {
        this.minAngle=min;
        return this;
    }

    public SubsystemConstants withMaxDistance(Distance max) {
        this.maxDistance=max;
        return this;
    }

    public SubsystemConstants withMinDistance(Distance min) {
        this.minDistance=min;
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

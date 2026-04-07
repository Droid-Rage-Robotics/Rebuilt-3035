package frc.utility.template;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
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
    public AngularAcceleration maxAcceleration;
    public double maxJerk;
    public AngularVelocity minVelocity;
    public double conversionFactor;
    public double offset;
    public Angle resetAngle;
    
    public double kP;
    public double kI;
    public double kD;
    public double kS;
    public double kG;
    public double kV;
    public double kA;

    public double gearRatio;

    public boolean hasLimitSwitch;
    
    public double length;
    public double width;

    public SubsystemConstants withEncoderType(EncoderType value) {
        this.encoderType=value;
        return this;
    }

    public SubsystemConstants withPID(double kP, double kI, double kD) {
        this.kP=kP;
        this.kI=kI;
        this.kD=kD;
        return this;
    }

    public SubsystemConstants withFeedforward(double kS, double kG, double kV, double kA) {
        this.kS=kS;
        this.kG=kG;
        this.kV=kV;
        this.kA=kA;
        return this;
    }
    
    public SubsystemConstants withFeedforward(double kS, double kV, double kA) {
        this.kS=kS;
        this.kV=kV;
        this.kA=kA;
        return this;
    }

    public SubsystemConstants withMaxAcceleration(AngularAcceleration max) {
        this.maxAcceleration=max;
        return this;
    }

    /**
     * This is the target jerk (acceleration derivative) Motion Magic®
     * based control modes are allowed to use.  Motion Magic® Expo control
     * modes do not use this config.  This allows Motion Magic® to
     * generate S-Curve profiles.
     * <p>
     * Jerk is optional; if this is set to zero, then Motion Magic® will
     * not apply a Jerk limit.
     * 
     * <p>
     * A good starting point is around 10x your acceleration, which means 
     * the acceleration would take 0.1 s to reach the max.
     * 
     * <ul>
     *   <li> <b>Minimum Value:</b> 0
     *   <li> <b>Maximum Value:</b> 9999
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> rot per sec³
     * </ul>
     */
    public SubsystemConstants withMaxJerk(double max) {
        this.maxJerk=max;
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

    public SubsystemConstants withResetAngle(Angle resetAngle) {
        this.resetAngle=resetAngle;
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

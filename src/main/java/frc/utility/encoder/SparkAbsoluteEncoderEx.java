package frc.utility.encoder;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;

import frc.utility.motor.SparkMaxEx;

public class SparkAbsoluteEncoderEx extends EncoderBase{
    private final SparkMaxEx sparkMax;
    private final AbsoluteEncoder encoder;
    private final AbsoluteEncoderConfig config = new AbsoluteEncoderConfig();

    private SparkAbsoluteEncoderEx(SparkMaxEx sparkMax, AbsoluteEncoder encoder) {
        this.sparkMax=sparkMax;
        sparkMax.setAbsoluteEncoderConfig(config);
        this.encoder=encoder;

    }

    public static SparkAbsoluteEncoderEx create(SparkMaxEx sparkMax) {
        return new SparkAbsoluteEncoderEx(sparkMax, sparkMax.getAbsoluteEncoder());
    }

    public SparkAbsoluteEncoderEx withZeroOffset(double value) {
        config.zeroOffset(value);
        sparkMax.setAbsoluteEncoderConfig(config);
        return this;
    }

    
    public SparkAbsoluteEncoderEx withDirection(EncoderDirection direction) {
        switch (direction) {
            case Reversed -> config.inverted(true);
            case Forward -> config.inverted(false);
        }
        sparkMax.setAbsoluteEncoderConfig(config);
        return this;
    }

    public SparkAbsoluteEncoderEx withConversionFactor(double value) {
        config.positionConversionFactor(value);
        config.velocityConversionFactor(value);
        sparkMax.setAbsoluteEncoderConfig(config);
        return this;
    }
    
    @Override
    public int getDeviceId() {
        return sparkMax.getDeviceId();
    }

    /**
     * Used to get the position of the encoder. Default units with default 
     * conversion factor of 1 are in rotations. Zero offsets are automatically 
     * applied along with custom conversion factors.
     * 
     * @return the positon of the encoder
     */
    @Override
    public double getAbsolutePosition() {
        return encoder.getPosition();
    }

    /**
     * Used to get the velocity of the encoder. Default units
     * with default conversion factor of 1 are in rotations
     * per second. Custom conversion factors are automatically
     * applied.
     * 
     * @return the velocity of the encoder
     */
    @Override
    public double getVelocity() {
        return encoder.getVelocity()/60;
    }
}

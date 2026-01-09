package frc.utility;

import static edu.wpi.first.units.Units.Degrees;

import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoHub;
import com.revrobotics.servohub.ServoChannel.ChannelId;
import edu.wpi.first.units.measure.Angle;

public class ServoEx {
    private boolean isEnabled;
    private final ServoChannel channel;
    private double minPulse,maxPulse,centerPulse;
    private double maxDegree,degRange;
    private boolean centerZeroPoint = false;
    private double conversionFactor = 1;
    
    private ServoEx(ServoChannel channel, boolean isEnabled) {
        this.channel=channel;
        this.isEnabled=isEnabled;
        
        this.channel.setPowered(this.isEnabled);
        this.channel.setEnabled(this.isEnabled);
    }

    /**
     * @param hub the ServoHub object to initialize the servo channel with
     * @param channelId the ID of the channel that the servo is connected to on the servo hub
     * @return a ServoEx object
     */
    public static ServoEx create(ServoHub hub,int channelId, boolean isEnabled) {
        return new ServoEx(hub.getServoChannel(ChannelId.fromInt(channelId)),isEnabled);
    }

    /**
     * Used to enable/disable the motor.
     * @param isEnabled
     * @return ServoEx (for call chaining)
     */
    public ServoEx withIsEnabled(boolean isEnabled) {
        this.isEnabled=isEnabled;
        this.channel.setPowered(isEnabled);
        this.channel.setEnabled(isEnabled);
        return this;
    }

    /**
     * Used to set the minimum pulse width supported by the servo for use in calculations
     * Do not call this method when using a servo with a center zero point.
     * @param value minimum pulse width
     * @throws UnsupportedOpperationExeption Cannot use this method with a center zero point!
     * @return ServoEx (for call chaining)
     */
    public ServoEx withMinPulseWidth(double value) {
        if(centerZeroPoint){
            throw new UnsupportedOperationException("Cannot use this method with a center zero point!");
        } else {  
            minPulse=value;
        }
        return this;
    }
    
    /**
     * Used to set the maximum pulse width supported by the servo for use in calculations
     * Do not call this method when using a servo with a center zero point.
     * @param value maximum pulse width
     * @throws UnsupportedOpperationExeption Cannot use this method with a center zero point!
     * @return ServoEx (for call chaining)
     */
    public ServoEx withMaxPulseWidth(double value) {
        if(centerZeroPoint){
            throw new UnsupportedOperationException("Cannot use this method with a center zero point!");
        } else {        
            maxPulse=value;
        }
        return this;
    }

    /**
     * Used to set a conversion factor to be applied to
     * the raw position/velocity of the servo. Defaults
     * to 1.
     * @param conversionFactor
     * @return ServoEx (for call chaining)
     */
    public ServoEx withConversionFactor(double conversionFactor) {
        this.conversionFactor=conversionFactor;
        return this;
    }

    /**
     * Used to set the maximum degree of rotation supported by the servo for use in pulse width calculations. 
     * Do not call this method when using a servo with a center zero point.
     * @param value maximum degree of rotation
     * @throws UnsupportedOpperationExeption Cannot use this method with a center zero point!
     * @return ServoEx (for call chaining)
     */
    public ServoEx withDegRange(double value) {
        if(centerZeroPoint){
            throw new UnsupportedOperationException("Cannot use this method with a center zero point!");
        } else {
            degRange=value;
        }
        return this;
    }
    
    /**
     * Used to set the pulse width that represents the zero point on a servo with a center zero point.
     * @param value pulse width, in microseconds
     * @return ServoEx (for call chaining)
     */
    public ServoEx withCenterPulseWidth(double value) {
        centerPulse=value;
        centerZeroPoint=true;
        return this;
    }

    /**
     * Used when the servo has a center zero point.
     * An example is the REV SRS with -135 deg as min, 0 deg as center, and 135 deg as max. 
     * The value would be set to 135 in this case.
     * @param value the max degrees of rotation in one direction from center
     * @return ServoEx (for call chaining)
     */
    public ServoEx withMaxDegree(double value) {
        maxDegree=value;
        return this;
    }

    /**
     * Used to set the position that the servo should
     * go to.
     * @param deg the position to go to (in degrees)
     */
    public void setDegrees(double deg) {
        if(centerZeroPoint){
            // Clamp to range -maxDegree .. +maxDegree
            deg = Math.max(-maxDegree, Math.min(maxDegree, deg));

            double pulse;
            if (deg >= 0) {
                pulse = centerPulse + (deg / maxDegree) * (maxPulse - centerPulse);
            } else {
                pulse = centerPulse + (deg / maxDegree) * (centerPulse - minPulse);
            }

            channel.setPulseWidth((int)pulse);
        } else {
            deg = Math.max(0, Math.min(degRange, deg));
            double norm = deg / degRange;
            double pulse = minPulse + norm * (maxPulse - minPulse);
            
            channel.setPulseWidth((int)pulse);
        }  
    }

    public void setRadians(double rad) {
        setDegrees(Math.toDegrees(rad));
    }

    public void setAngle(Angle angle) {
        setDegrees(angle.in(Degrees));
    }

    /**
     * Used to get the position of the servo. Units with default conversion
     * factor of 1 are in degrees.
     * @return the position of the servo.
     */
    public double getPosition() {
        int pulse = channel.getPulseWidth(); // assume this returns the current pulse width
        double position;
        
        if (centerZeroPoint) {
            if (pulse >= centerPulse) {
                // positive side
                position = (pulse - centerPulse) * maxDegree / (maxPulse - centerPulse);
            } else {
                // negative side
                position = (pulse - centerPulse) * maxDegree / (centerPulse - minPulse);
            }
        } else {
            // Normal (non-centered) servo
            double norm = (pulse - minPulse) / (maxPulse - minPulse);
            position = norm * degRange;
        }

        return position * conversionFactor;
    }
}
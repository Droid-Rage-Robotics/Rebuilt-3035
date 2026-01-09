package frc.utility.encoder;

import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.Timer;
import frc.utility.DashboardUtils.Periodic;

public class AbsoluteDutyEncoderRIO extends EncoderBase implements Sendable, AutoCloseable, Periodic {
	private final DutyCycle dutyCycle;
	private final DigitalInput digitalInput;
	private final int deviceId;
	private EncoderDirection direction = EncoderDirection.Forward;
	private double conversionFactor = 1;
	
	private int m_frequencyThreshold = 100;
	private double range = 1;
	private double offset = 0;
	private double m_periodNanos;
	private double m_sensorMin;
	private double m_sensorMax = 1.0;

	private SimDevice m_simDevice;
	private SimDouble m_simPosition;
	private SimBoolean m_simIsConnected;

	private double velocity = 0;

	private double lastPos = 0;
    private double lastTime;
	
	private AbsoluteDutyEncoderRIO(int deviceId) {
		this.deviceId = deviceId;
		this.digitalInput = new DigitalInput(deviceId);
		this.dutyCycle = new DutyCycle(digitalInput);

		m_simDevice = SimDevice.create("DutyCycle:DutyCycleEncoder", dutyCycle.getSourceChannel());

		if (m_simDevice != null) {
				m_simPosition = m_simDevice.createDouble("Position", SimDevice.Direction.kInput, 0.0);
				m_simIsConnected = m_simDevice.createBoolean("Connected", SimDevice.Direction.kInput, true);
		}

		SendableRegistry.addLW(this, "DutyCycle Encoder", dutyCycle.getSourceChannel());
        
		lastTime = Timer.getFPGATimestamp();

		// encoder.setAssumedFrequency(975.6);
	}

	@Override
	public void periodic() {
		double currentTime = Timer.getFPGATimestamp();
        double currentPos = getAbsolutePosition();  // 0–1 rev

        double dt = currentTime - lastTime;
        if (dt <= 0) return;

        // Compute shortest-wrap-around difference in revolutions
        double delta = currentPos - lastPos;
        delta = (delta + 0.5) % 1.0 - 0.5;   // ensures result in [-0.5, 0.5)

        // Convert to rotations per second
        velocity = delta / dt;

        // Update history
        lastPos = currentPos;
        lastTime = currentTime;
	}
	
	public static AbsoluteDutyEncoderRIO create(int deviceId) {
		return new AbsoluteDutyEncoderRIO(deviceId);
	}

	public AbsoluteDutyEncoderRIO withDirection(EncoderDirection value) {
		direction=value;
		return this;
	}

	public AbsoluteDutyEncoderRIO withZeroOffset(double value) {
		offset=value;
		return this;
	}

	public AbsoluteDutyEncoderRIO withConversionFactor(double value) {
		conversionFactor=value;
		return this;
	}

	/**
	 * Used to set the value to report at maximum travel
	 * @param value
	 * @return
	 */
	public AbsoluteDutyEncoderRIO withRange(double value) {
		range=value;
		return this;
	}

	public void zero() {
		double raw = getRawPositionInternal();   // raw in rotations before offset/scale/direction

		offset = raw;  // shift so current position becomes 0

		System.out.println("=== Encoder " + deviceId + " Zeroed ===");
		System.out.println("Add this to your robot constants:");
		System.out.println("    .withZeroOffset(" + offset + ");");
	}

	private double getRawPositionInternal() {
		double pos;

		if (m_simPosition != null)
			pos = m_simPosition.get();
		else if (m_periodNanos == 0.0)
			pos = dutyCycle.getOutput();
		else
			pos = dutyCycle.getHighTimeNanoseconds() / m_periodNanos;

		// Map sensor range
		pos = mapSensorRange(pos);

		// Scale into [0..range)
		return pos * range;
	}


	/**
	 * Get the encoder value since the last reset.
	 *
	 * <p>This is reported in rotations since the last reset.
	 *
	 * @return the encoder value in rotations
	 */
	@Override
	public double getAbsolutePosition() {
		double raw = getRawPositionInternal();            // in [0..range)

		double shifted = MathUtil.inputModulus(
			raw - offset,
			0,
			range
		);

		// Optional inversion
		if (direction == EncoderDirection.Reversed) {
			shifted = range - shifted;
		}

		// Apply scaling last
		return shifted * conversionFactor;

  	}

	@Override
	public double getVelocity() {
		if(direction==EncoderDirection.Reversed){
			velocity = -velocity;
		}
		
		return velocity * conversionFactor;
	}

	@Override
	public int getDeviceId() {
		return deviceId;
	}

	private double mapSensorRange(double pos) {
		// map sensor range
		if (pos < m_sensorMin) {
			pos = m_sensorMin;
		}
		if (pos > m_sensorMax) {
			pos = m_sensorMax;
		}
		pos = (pos - m_sensorMin) / (m_sensorMax - m_sensorMin);
		return pos;
	}

	

  /**
   * Set the encoder duty cycle range. As the encoder needs to maintain a duty cycle, the duty cycle
   * cannot go all the way to 0% or all the way to 100%. For example, an encoder with a 4096 us
   * period might have a minimum duty cycle of 1 us / 4096 us and a maximum duty cycle of 4095 /
   * 4096 us. Setting the range will result in an encoder duty cycle less than or equal to the
   * minimum being output as 0 rotation, the duty cycle greater than or equal to the maximum being
   * output as 1 rotation, and values in between linearly scaled from 0 to 1.
   *
   * @param min minimum duty cycle (0-1 range)
   * @param max maximum duty cycle (0-1 range)
   */
  public void setDutyCycleRange(double min, double max) {
	m_sensorMin = MathUtil.clamp(min, 0.0, 1.0);
	m_sensorMax = MathUtil.clamp(max, 0.0, 1.0);
  }

  /**
   * Get the frequency in Hz of the duty cycle signal from the encoder.
   *
   * @return duty cycle frequency in Hz
   */
  public int getFrequency() {
	return dutyCycle.getFrequency();
  }

  /**
   * Get if the sensor is connected
   *
   * <p>This uses the duty cycle frequency to determine if the sensor is connected. By default, a
   * value of 100 Hz is used as the threshold, and this value can be changed with {@link
   * #setConnectedFrequencyThreshold(int)}.
   *
   * @return true if the sensor is connected
   */
  public boolean isConnected() {
	if (m_simIsConnected != null) {
	  return m_simIsConnected.get();
	}
	return getFrequency() > m_frequencyThreshold;
  }

	/**
	 * Change the frequency threshold for detecting connection used by {@link #isConnected()}.
	 *
	 * @param frequency the minimum frequency in Hz.
	 */
	public void setConnectedFrequencyThreshold(int frequency) {
		if (frequency < 0) {
			frequency = 0;
		}

		m_frequencyThreshold = frequency;
	}

	/**
	 * Sets the assumed frequency of the connected device.
	 *
	 * <p>By default, the DutyCycle engine has to compute the frequency of the input signal. This can
	 * result in both delayed readings and jumpy readings. To solve this, you can pass the expected
	 * frequency of the sensor to this function. This will use that frequency to compute the DutyCycle
	 * percentage, rather than the computed frequency.
	 *
	 * @param frequency the assumed frequency of the sensor
	 */
	public void setAssumedFrequency(double frequency) {
		if (frequency == 0.0) {
			m_periodNanos = 0.0;
		} else {
			m_periodNanos = 1000000000 / frequency;
		}
	}

	/**
	 * Get the FPGA index for the DutyCycleEncoder.
	 *
	 * @return the FPGA index
	 */
	public int getFPGAIndex() {
		return dutyCycle.getFPGAIndex();
	}

	/**
	 * Get the channel of the source.
	 *
	 * @return the source channel
	 */
	public int getSourceChannel() {
		return dutyCycle.getSourceChannel();
	}

	@Override
	public void close() {
		dutyCycle.close();

		if (digitalInput != null) {
			digitalInput.close();
		}
		if (m_simDevice != null) {
			m_simDevice.close();
		}
	}

 	@Override
	public void initSendable(SendableBuilder builder) {
		builder.setSmartDashboardType("AbsoluteEncoder");
		builder.addDoubleProperty("Position", this::getAbsolutePosition, null);
		builder.addBooleanProperty("Is Connected", this::isConnected, null);
		builder.addBooleanProperty("Zero Encoder", () -> false, (x) -> { if (x) zero(); });
		builder.addDoubleProperty("Zero Offset", () -> offset, null);
	}
}
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.DroidRageConstants;
import frc.utility.devices.motor.MotorConstants;
import frc.utility.devices.motor.MotorConstants.Direction;
import frc.utility.io.FlywheelIOTalonFX;
import frc.utility.template.FlywheelTemplate;
import frc.utility.template.Constants.FlywheelConstants;
import lombok.Getter;

public class Kicker extends FlywheelTemplate {
    public enum KickerValue {
        INTAKE(170), //55
        OUTTAKE(-150),
        STOP(0);
        // HOLD(0);

        @Getter private final AngularVelocity kickerValue;

        private KickerValue(double kickerValue) {
            this.kickerValue = RotationsPerSecond.of(kickerValue);
        }
    } 

    private static final FlywheelConstants constants = new FlywheelConstants()
        // .withConversionFactor(1)
        .withPID(0.2, 0, 0) //.24
        .withFeedforward(0.038, 0.14, 0.25) //.043254, 0.1229, 0.02026
        // .withEncoderType(EncoderType.INTEGRATED)
        .withVelocityLimits(RotationsPerSecond.of(-100), RotationsPerSecond.of(100))
        .withName("Kicker")
        // .withOffset(0)
        .withMainNum(0);
    
    private static final MotorConstants motor = new MotorConstants() 
        .withDeviceId(18)
        .withCANBus(DroidRageConstants.rioCanBus)
        .withDirection(Direction.Forward)
        .withIdleMode(NeutralModeValue.Coast)
        .withConversionFactor( 1)
        .withSupplyCurrentLimit(45)
        .withStatorCurrentLimit(50);

    public Kicker(boolean isEnabled) {
        super(constants, new FlywheelIOTalonFX(isEnabled, constants, motor));
    }

    
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty("IndexerOn", () -> isKickerOn(), null);
    }

    @Override
    public void elasticInit() {
        SmartDashboard.putData("Shooter", this);
    }

    @Override
    public void practiceWriters() {}

     public boolean isKickerOn(){
        return !(getTargetVelocity() == KickerValue.STOP.getKickerValue());
    }
}

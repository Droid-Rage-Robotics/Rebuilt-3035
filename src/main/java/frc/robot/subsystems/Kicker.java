package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.DroidRageConstants;
import frc.utility.devices.motor.MotorConstants;
import frc.utility.devices.motor.MotorConstants.Direction;
import frc.utility.template.FlywheelTemplate;
import frc.utility.template.SubsystemConstants;
import frc.utility.template.SubsystemConstants.EncoderType;
import lombok.Getter;

public class Kicker extends FlywheelTemplate {
    public enum KickerValue {
        INTAKE(55),
        OUTTAKE(-150),
        STOP(0);
        // HOLD(0);

        @Getter private final AngularVelocity kickerValue;

        private KickerValue(double kickerValue) {
            this.kickerValue = RotationsPerSecond.of(kickerValue);
        }
    } 

    private static final SubsystemConstants constants = new SubsystemConstants()
        .withConversionFactor(1)
        .withPID(0.2, 0, 0) //.24
        .withFeedforward(0.038, 0.14, 0.25) //.043254, 0.1229, 0.02026
        .withEncoderType(EncoderType.INTEGRATED)
        .withMinVelocity(RotationsPerSecond.of(-300))
        .withMaxVelocity(RotationsPerSecond.of(300))
        .withName("Kicker")
        .withOffset(0)
        .withMainNum(0);
    
    private static final MotorConstants motor = new MotorConstants() 
        .withDeviceId(18)
        .withCANBus(DroidRageConstants.rioCanBus)
        .withDirection(Direction.Reversed)
        .withIdleMode(NeutralModeValue.Coast)
        .withConversionFactor( 1)
        .withSupplyCurrentLimit(35)
        .withStatorCurrentLimit(35);

    public Kicker(boolean isEnabled) {
        super(isEnabled, constants, motor);
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

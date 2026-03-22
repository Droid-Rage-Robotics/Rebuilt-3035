package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

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

public class Indexer extends FlywheelTemplate {
    public enum IndexerValue {
        INTAKE(30), //100
        OUTTAKE(-30),//-25
        STOP(0),
        HOLD(0);

        @Getter private final AngularVelocity indexerValue;

        private IndexerValue(double indexerValue) {
            this.indexerValue = RotationsPerSecond.of(indexerValue);
        }
    }
    
    private static final SubsystemConstants constants = new SubsystemConstants()
        .withGearRatio(3.0)
        .withPID(0.014761, 0, 0) //0.032889
        .withFeedforward(0.14827, 1.0665, 0.014828) //0.34224, 0.37116, 0.0095347
        .withEncoderType(EncoderType.INTEGRATED)
        .withMinVelocity(RotationsPerSecond.of(-150))
        .withMaxVelocity(RotationsPerSecond.of(150))
        .withName("Indexer")
        .withOffset(0)
        .withMainNum(0);
    
    private static final MotorConstants motor = new MotorConstants() 
        .withDeviceId(14)
        .withCANBus(DroidRageConstants.driveCanBus)
        .withDirection(Direction.Forward)
        .withIdleMode(NeutralModeValue.Coast)
        .withStatorCurrentLimit(80)//100
        .withSupplyCurrentLimit(75);//100
        //SUPERNERDS have 40 stator

    public Indexer(boolean isEnabled) {
        super(isEnabled, constants, motor);
    }
        
    @Override
    public void periodic(){
        super.periodic();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty("IndexerOn", () -> isIndexerOn(), null);
    }

    @Override
    public void elasticInit() {
        SmartDashboard.putData("Shooter", this);
    }

    @Override
    public void practiceWriters() {}

    @Override
    public void alerts() {}
    public boolean isStalling() {
        return Math.abs(getCurrent().in(Amps)) >=60;
    }

    public boolean isIndexerOn(){
        return !(getTargetVelocity() == IndexerValue.STOP.indexerValue);
    }
}
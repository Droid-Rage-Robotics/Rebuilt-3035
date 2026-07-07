package frc.utility.io;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.utility.devices.motor.MotorConstants;
import frc.utility.io.devices.MotorIO;
import frc.utility.io.devices.MotorIOTalonFX;
import frc.utility.template.Constants.FlywheelConstants;

public class FlywheelIOTalonFX implements FlywheelIO {
    private final MotorIOTalonFX[] motors;
    private final MotorIO.MotorIOInputs mainMotorInputs = new MotorIO.MotorIOInputs();

    private final TalonFX mainMotor;
    private final int mainNum;

    private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0);
    private final VoltageOut voltageRequest = new VoltageOut(0.0);

    private final StatusSignal<Double> closedLoopReference;
    private final StatusSignal<Double> closedLoopError;

    private boolean isEnabled;

    public FlywheelIOTalonFX(
            boolean isEnabled,
            FlywheelConstants constants,
            MotorConstants... motorConstants
    ) {
        this.mainNum = constants.mainNum;
        this.isEnabled=isEnabled;

        motors = new MotorIOTalonFX[motorConstants.length];

        for (int i = 0; i < motorConstants.length; i++) {
            motorConstants[i].isEnabled = isEnabled;
            motors[i] = new MotorIOTalonFX(motorConstants[i]);
        }

        mainMotor = motors[mainNum].getMotor();

        for (int i = 0; i < motors.length; i++) {
            if (i == mainNum) continue;

            motors[i].setControl(
                new Follower(
                    mainMotor.getDeviceID(),
                    motorConstants[i].alignment
                )
            );
        }

        var config = motorConstants[mainNum].getConfig();

        config.Feedback.SensorToMechanismRatio = constants.gearRatio;

        config.Slot0.kP = constants.kP;
        config.Slot0.kI = constants.kI;
        config.Slot0.kD = constants.kD;
        config.Slot0.kV = constants.kV;
        config.Slot0.kA = constants.kA;
        config.Slot0.kS = constants.kS;

        mainMotor.getConfigurator().apply(config, 0.25);

        closedLoopReference = mainMotor.getClosedLoopReference();
        closedLoopError = mainMotor.getClosedLoopError();

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            closedLoopReference,
            closedLoopError
        );

        ParentDevice.optimizeBusUtilizationForAll(mainMotor);
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        motors[mainNum].updateInputs(mainMotorInputs);

        BaseStatusSignal.refreshAll(
            closedLoopReference,
            closedLoopError
        );

        inputs.mainMotorConnected = mainMotorInputs.connected;

        inputs.positionRotations = mainMotorInputs.positionRotations;
        inputs.velocityRotationsPerSecond = mainMotorInputs.velocityRotationsPerSecond;
        inputs.appliedVolts = mainMotorInputs.appliedVolts;
        inputs.statorCurrentAmps = mainMotorInputs.statorCurrentAmps;
        inputs.torqueCurrentAmps = mainMotorInputs.torqueCurrentAmps;

        inputs.closedLoopReferenceRotationsPerSecond =
            closedLoopReference.getValueAsDouble();

        inputs.closedLoopErrorRotationsPerSecond =
            closedLoopError.getValueAsDouble();
    }

    @Override
    public void setVelocity(AngularVelocity velocity) {
        setVelocityRotationsPerSecond(velocity.in(RotationsPerSecond));
    }

    @Override
    public void setVelocityRotationsPerSecond(double velocityRotationsPerSecond) {
        if (isEnabled) {
            mainMotor.setControl(velocityRequest.withVelocity(velocityRotationsPerSecond));
        }
    }

    @Override
    public void setVoltage(Voltage voltage) {
        if (isEnabled) {
            mainMotor.setControl(voltageRequest.withOutput(voltage));
        }
    }

    public TalonFX getMainMotor() {
        return mainMotor;
    }

    public MotorIOTalonFX[] getMotors() {
        return motors;
    }
}

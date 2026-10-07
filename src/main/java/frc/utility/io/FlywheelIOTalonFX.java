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
    private final MotorIO.MotorIOInputs[] motorInputs;

    private final TalonFX mainMotor;
    private final int mainNum;

    private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0);
    private final VoltageOut voltageRequest = new VoltageOut(0.0);

    private final StatusSignal<Double> closedLoopReference;
    private final StatusSignal<Double> closedLoopError;

    private final Follower[] followerRequests;

    public FlywheelIOTalonFX(
            boolean isEnabled,
            FlywheelConstants constants,
            MotorConstants... motorConstants
    ) {
        this.mainNum = constants.mainNum;

        motors = new MotorIOTalonFX[motorConstants.length];
        motorInputs = new MotorIO.MotorIOInputs[motorConstants.length];

        for (int i = 0; i < motorConstants.length; i++) {
            motorConstants[i].isEnabled = isEnabled;
            motors[i] = new MotorIOTalonFX(motorConstants[i]);
            motorInputs[i] = new MotorIO.MotorIOInputs();
        }

        mainMotor = motors[mainNum].getMotor();

        followerRequests = new Follower[motors.length];

        for (int i = 0; i < motors.length; i++) {
            if (i == mainNum) continue;

            followerRequests[i] = new Follower(
                mainMotor.getDeviceID(),
                motorConstants[i].alignment
            );

            motors[i].setControl(followerRequests[i]);
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
        inputs.mainMotorIndex = mainNum;
        if (inputs.motorIds.length != motors.length) {
            inputs.motorIds = new int[motors.length];
            inputs.motorConnected = new boolean[motors.length];
            inputs.motorAppliedVolts = new double[motors.length];
            inputs.motorStatorCurrentAmps = new double[motors.length];
            inputs.motorSupplyCurrentAmps = new double[motors.length];
            inputs.motorTorqueCurrentAmps = new double[motors.length];
            inputs.motorTempCelsius = new double[motors.length];
        }

        for (int i = 0; i < motors.length; i++) {
            motors[i].updateInputs(motorInputs[i]);
            var measured = motorInputs[i];
            inputs.motorIds[i] = motors[i].getMotor().getDeviceID();
            inputs.motorConnected[i] = measured.connected;
            inputs.motorAppliedVolts[i] = measured.appliedVolts;
            inputs.motorStatorCurrentAmps[i] = measured.statorCurrentAmps;
            inputs.motorSupplyCurrentAmps[i] = measured.supplyCurrentAmps;
            inputs.motorTorqueCurrentAmps[i] = measured.torqueCurrentAmps;
            inputs.motorTempCelsius[i] = measured.tempCelsius;
        }

        BaseStatusSignal.refreshAll(
            closedLoopReference,
            closedLoopError
        );

        var mainMotorInputs = motorInputs[mainNum];

        inputs.positionRotations = mainMotorInputs.positionRotations;
        inputs.velocityRotationsPerSecond = mainMotorInputs.velocityRotationsPerSecond;

        inputs.closedLoopReferenceRotationsPerSecond =
            closedLoopReference.getValueAsDouble();

        inputs.closedLoopErrorRotationsPerSecond =
            closedLoopError.getValueAsDouble();
    }

    @Override
    public void setEnabled(boolean isEnabled) {
        // Change permission on every motor.
        // Disabling also stops each motor through the wrapper.
        for (var motor : motors) {
            motor.setEnabled(isEnabled);
        }

        // Restore follower mode after enabling their wrappers.
        if (isEnabled) {
            for (int i = 0; i < motors.length; i++) {
                if (i == mainNum) continue;

                motors[i].setControl(followerRequests[i]);
            }
        }
    }

    @Override
    public void setVelocity(AngularVelocity velocity) {
        setVelocityRotationsPerSecond(velocity.in(RotationsPerSecond));
    }

    @Override
    public void setVelocityRotationsPerSecond(double velocityRotationsPerSecond) {
        motors[mainNum].setControl(velocityRequest.withVelocity(velocityRotationsPerSecond));
    }

    @Override
    public void setVoltage(Voltage voltage) {
        motors[mainNum].setControl(voltageRequest.withOutput(voltage));
    }

    public MotorIOTalonFX[] getMotors() {
        return motors;
    }
}

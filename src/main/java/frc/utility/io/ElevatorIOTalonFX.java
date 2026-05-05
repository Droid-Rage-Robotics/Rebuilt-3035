package frc.utility.io;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import frc.utility.devices.motor.MotorConstants;
import frc.utility.devices.motor.MotorIO;
import frc.utility.devices.motor.MotorIOTalonFX;
import frc.utility.template.Constants.ElevatorConstants;

public class ElevatorIOTalonFX implements ElevatorIO {
    private final MotorIOTalonFX[] motors;
    private final MotorIO.MotorIOInputs mainMotorInputs = new MotorIO.MotorIOInputs();

    private final TalonFX mainMotor;
    private final int mainNum;
    private final double metersPerMotorRotation;

    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0.0);
    private final VoltageOut voltageRequest = new VoltageOut(0.0);

    private final StatusSignal<Double> closedLoopReference;
    private final StatusSignal<Double> closedLoopReferenceSlope;
    private final StatusSignal<Double> closedLoopError;

    public ElevatorIOTalonFX(
            boolean isEnabled,
            ElevatorConstants constants,
            MotorConstants... motorConstants
    ) {
        this.mainNum = constants.mainNum;
        this.metersPerMotorRotation = constants.metersPerMotorRotation;

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

        config.Feedback.SensorToMechanismRatio = 1.0;

        config.Slot0.kP = constants.kP;
        config.Slot0.kI = constants.kI;
        config.Slot0.kD = constants.kD;
        config.Slot0.kV = constants.kV;
        config.Slot0.kA = constants.kA;
        config.Slot0.kS = constants.kS;
        config.Slot0.kG = constants.kG;

        config.MotionMagic.MotionMagicCruiseVelocity =
            constants.maxVelocity.in(MetersPerSecond) / metersPerMotorRotation;

        config.MotionMagic.MotionMagicAcceleration =
            constants.maxAcceleration.in(MetersPerSecondPerSecond) / metersPerMotorRotation;

        config.MotionMagic.MotionMagicJerk = constants.maxJerk;

        mainMotor.getConfigurator().apply(config, 0.25);

        closedLoopReference = mainMotor.getClosedLoopReference();
        closedLoopReferenceSlope = mainMotor.getClosedLoopReferenceSlope();
        closedLoopError = mainMotor.getClosedLoopError();

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            closedLoopReference,
            closedLoopReferenceSlope,
            closedLoopError
        );

        ParentDevice.optimizeBusUtilizationForAll(mainMotor);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        motors[mainNum].updateInputs(mainMotorInputs);

        BaseStatusSignal.refreshAll(
            closedLoopReference,
            closedLoopReferenceSlope,
            closedLoopError
        );

        inputs.mainMotorConnected = mainMotorInputs.connected;

        inputs.position =
            Meters.of(mainMotorInputs.position.in(Rotations) * metersPerMotorRotation);

        inputs.velocity =
            MetersPerSecond.of(mainMotorInputs.velocity.in(RotationsPerSecond) * metersPerMotorRotation);

        inputs.appliedVoltage = mainMotorInputs.appliedVolts;
        inputs.statorCurrent = mainMotorInputs.statorCurrent;

        inputs.closedLoopReference =
            Meters.of(closedLoopReference.getValueAsDouble() * metersPerMotorRotation);

        inputs.closedLoopReferenceVelocity =
            MetersPerSecond.of(closedLoopReferenceSlope.getValueAsDouble() * metersPerMotorRotation);

        inputs.closedLoopError =
            Meters.of(closedLoopError.getValueAsDouble() * metersPerMotorRotation);

        inputs.encoderConnected = true;
    }

    @Override
    public void setPosition(Distance position) {
        double motorRotations = position.in(Meters) / metersPerMotorRotation;
        mainMotor.setControl(motionMagicRequest.withPosition(motorRotations));
    }

    @Override
    public void setVoltage(Voltage voltage) {
        mainMotor.setControl(voltageRequest.withOutput(voltage));
    }

    @Override
    public void resetEncoder(Distance position) {
        double motorRotations = position.in(Meters) / metersPerMotorRotation;
        motors[mainNum].setPosition(motorRotations);
    }
}
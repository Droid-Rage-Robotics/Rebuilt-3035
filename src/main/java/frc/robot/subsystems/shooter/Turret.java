package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.utility.motor.TalonEx;
import frc.utility.motor.MotorBase;
import frc.utility.motor.MotorBase.Direction;
import frc.utility.motor.MotorBase.ZeroPowerMode;
import frc.utility.template.TurretTemplate;

public class Turret extends TurretTemplate {
    private static final TalonEx motor = TalonEx.create(0)
        .withDirection(Direction.Forward)
        .withIdleMode(ZeroPowerMode.Brake)
        .withConversionFactor(1);

    public Turret(boolean isEnabled) {
        super(
            new MotorBase[]{motor},
            new ProfiledPIDController(0, 0, 0,
            new TrapezoidProfile.Constraints(0, 0)), 
            0, 0,
            1, 0, 0, isEnabled);
    }   
}

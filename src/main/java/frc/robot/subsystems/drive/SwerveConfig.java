package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import lombok.Getter;

public class SwerveConfig {
    public static final Distance WHEEL_RADIUS = Inches.of(2);
    
    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    @Getter private final Rotation2d blueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    @Getter private final Rotation2d redAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);
}

package frc.utility;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class ControllerUtils {
    public static double getRightStickDeg(CommandXboxController controller) {
        double x = controller.getRightX();
        double y = controller.getRightY();

        // atan2 gives the angle of the vector (y, x)
        double angleRadians = Math.atan2(-y, -x);
        double angleDegrees = Math.toDegrees(angleRadians)-90;

        // Normalize to [0, 360)
        if (angleDegrees < 0) {
            angleDegrees += 360;
        }
        
        return angleDegrees;
    }

    public static double getLeftStickDeg(CommandXboxController controller) {
        double x = controller.getLeftX();
        double y = controller.getLeftY();

        // atan2 gives the angle of the vector (y, x)
        double angleRadians = Math.atan2(-y, -x);
        double angleDegrees = Math.toDegrees(angleRadians)-90;

        // Normalize to [0, 360)
        if (angleDegrees < 0) {
            angleDegrees += 360;
        }
        
        return angleDegrees;
    }
}

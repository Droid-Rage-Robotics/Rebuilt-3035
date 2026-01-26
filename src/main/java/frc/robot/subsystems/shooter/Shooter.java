package frc.robot.subsystems.shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.subsystems.vision.Vision;
import frc.utility.LimelightEx;

public class Shooter {
    private enum TargetHeight {
    TAG_2(2, 0),
    TAG_3(3, 0),
    TAG_4(4, 0),
    TAG_5(5, 0),
    TAG_8(8, 0),
    TAG_9(9, 0),
    TAG_10(10, 0),
    TAG_11(11, 0),
    TAG_18(18, 0),
    TAG_19(19, 0),
    TAG_20(20, 0),
    TAG_21(21, 0),
    TAG_24(24, 0),
    TAG_25(25, 0),
    TAG_26(26, 0),
    TAG_27(27, 0);

    private final int id;
    private final double height;

    private TargetHeight(int id, double height) {
        this.id = id;
        this.height = height;
    }

    public static double getHeight(double tagId) {
        for (TargetHeight target : values()) {
            if (target.id == tagId) {
                return target.height;
            }
        }
        return 0; // Default height if tag ID not found - TODO: Fix
    }
}
    
    private static final InterpolatingDoubleTreeMap SCORE_SPEED_MAP = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap HOARD_SPEED_MAP = new InterpolatingDoubleTreeMap();

    static{
        //Include all speed values for the shooter (LL Distance, Speed)

        //Scoring
        SCORE_SPEED_MAP.put(0.0,0.0);
        SCORE_SPEED_MAP.put(4.1,100.0);


        //Hoarding
        HOARD_SPEED_MAP.put(0.0,0.0);
        HOARD_SPEED_MAP.put(4.1,100.0);
    }

    private final Turret turret;
    private final Hood hood;
    private final ShooterWheel shooter;
    private final Vision vision;

    private final LimelightEx limelight;

    private static final double LIMELIGHT_HEIGHT=0;
    private static final double LIMELIGHT_PITCH=0;

    public Shooter (
        Turret turret,
        Hood hood,
        ShooterWheel shooter,
        Vision vision
    ) {
        this.turret=turret;
        this.hood=hood;
        this.shooter=shooter;
        this.vision=vision;
        
        this.limelight=vision.getLeftLimelight();
    }
    
    
    public void aim() {
        // Get distance from Limelight
        double ty = limelight.getTY();

        double horizontalDistance = (TargetHeight.getHeight(limelight.getID()) - LIMELIGHT_HEIGHT) / 
                         Math.tan(Math.toRadians(LIMELIGHT_PITCH + ty));

        // Calculate hood angle (using lookup table example)
        // double hoodAngle = interpolateHoodAngle(distance);

        // Calculate turret angle
        double tx = limelight.getTX();
        double turretAdjustment = tx;

        // Command subsystems
        // hood.setAngle(hoodAngle);
        // turret.adjustAngle(turretAdjustment);
    }
}

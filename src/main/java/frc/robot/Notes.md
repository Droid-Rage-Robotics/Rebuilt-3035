Notes:
- Connecting to Roborio using FileZilla
   Host: sftp://roboRIO-3035-frc.local
   Username:lvuser
   Port:22
   Location of SysID Logs: /home/lvuser/logs 
- Without any PID, the arm should stay upright, basically sayign that the kg is good
- System.out.println("setting command."+ targetPosition.name()); <-- Use this to troubleshoot>
- Connect in Pathplanner with Robot: 10.30.35.2
- Color: 5V - Multi COlor; Strip/12V - Single COlor Strip
- CycleTracker: https://docs.wpilib.org/en/stable/docs/software/telemetry/datalog.html; https://www.chiefdelphi.com/t/roborio-wpilib-logging/159434/17
- Drive Feedforward: kS - Increase until the robot very slowly moves forward (this means too much,so lower) Then kV until the robot goes forward the amount needed
- Constants for Motor
   /** Max RPM of NEO */
   public static final int NEO_MAX_RPM = 5676;
   /** Max RPM of Vortex */
   public static final int VORTEX_MAX_RPM = 6784;
   /** Ticks per revolution of NEO built-in encoder */
   public static final int NEO_ENCODER_TICKS_PER_ROTATION = 42;
   /** Ticks per rotation of Vortex built-in encoder */
   public static final int VORTEX_ENCODER_TICKS_PER_ROTATION = 7168;
   /** Ticks per revolution of REV through bore encoder */
   public static final int REV_ENCODER_TICKS_PER_ROTATION = 8192;


Auto Setup
- Align wheels straight or in direction of robot travel



Wiring
- Kraken has a 12V40A
- Talon FX has a 12V30A
- Drive Kraken has a 12V40A
- Turn NEO has a 12V40A
- Turn Kraken has a 12V30A
- CTR Absolute encoder has a 5A mini fuse
- Pigeon has a 5A mini fuse
- Limelight is powered by a 10A mini fuse
- mini power module powered my a 12V 20A fuse
- ethernet switch is powered by a 10A mini fuse - DO NOT USE
- Radio power module is powered by a 10A mini fuse - DO NOT USE
- RoboRio has a 10A mini fuse
- CANivore has a 10A mini fuse

Setting Up Telemetry
- Add FMS Info: In the left hand pane, select the Network Tables Tab. Scroll down to block of data titled FMSinfo, and drag
package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

public final class DriveConstants {
    public static final double ksVolts = 0.929;
    public static final double kvVoltSecondsPerMeter = 6.33;
    public static final double kaVoltSecondsSquaredPerMeter = 0.0389;

    public static final double kPDriveVel = 1.0;
    public static final double kIDriveVel = 0.0;
    public static final double kDDriveVel = 0.0;

    public static final double kTrackwidthMeters = 0.142072613;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
        kTrackwidthMeters);

    public static final double maxVelocityMetersPerSecond = 0.6;
    public static final double kWheelDiameterInch = 2.75591; // 70 mm
  }
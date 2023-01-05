// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class RobotConstants {
    public static final double kCountsPerRevolution = 1440.0;

    public static final Transform3d RobotToCamera =  new Transform3d(
        new Translation3d(-5.0/100.0, -2.0/100.0, 6.5/100.0),
        new Rotation3d(0.0, 0.0, 0.0));
    public static final String APRILTAG_CAMERA = "front";
    //                                                   P     I     D    F
    public final static Gains kGainsDistance = new Gains(0.35, 0.05, 0.05, 0.0, 0, 0.0);
    public final static Gains kGainsVelocity = new Gains(1.0, 0.05, 0.05, 0.0, 0, 0.0);
    public static final double kTrackwidthMeters = 0.142072613;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
        kTrackwidthMeters);
}

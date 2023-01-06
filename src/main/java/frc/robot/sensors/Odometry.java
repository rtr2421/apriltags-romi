// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.photonvision.PhotonCamera;
import org.photonvision.RobotPoseEstimator;
import org.photonvision.RobotPoseEstimator.PoseStrategy;

import edu.wpi.first.math.ComputerVisionUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.DriveConstants;
import frc.robot.FieldConstants;
import frc.robot.RobotConstants;

/** Add your docs here. */
public class Odometry {

    RobotPoseEstimator robotPoseEstimator;

    public DifferentialDrivePoseEstimator poseEstimator;
    public Pose2d pose;
    public final Field2d field = new Field2d();
    public double lastVisionTimestamp = 0;

    public Odometry(PhotonCamera camera) {

        robotPoseEstimator = new RobotPoseEstimator(FieldConstants.layout, PoseStrategy.AVERAGE_BEST_TARGETS, List.of(Pair.of(camera, RobotConstants.RobotToCamera)));
        poseEstimator = new DifferentialDrivePoseEstimator(
                DriveConstants.kDriveKinematics,
                new Rotation2d(),
                0, 0,
                new Pose2d(), // TODO update initial pose
                VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
                VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

        field.setRobotPose(new Pose2d());
        SmartDashboard.putData(field);
        Map<Integer, Pose3d> apriltags = new HashMap<>();
        apriltags.put(1, FieldConstants.tag1.pose);
    }

    /**
     * Reset the robot pose to something we know
     * @param pose the pose
     */
    public void setRobotPose(Pose2d pose) {
        this.pose = pose;
    }

    /**
     * Returns the current pose of the robot
     * @return
     */
    public Pose2d getRobotPose() {
        return pose;
    }
    /**
     * 
     * @param rotation - the current gyro heading
     * @param leftDistance - the distance in meters of the left encoder
     * @param rightDistance - the distance in meters of the right encoder
     */
    public void updateOdometry(Rotation2d rotation, Double leftDistance, Double rightDistance) {
        pose = poseEstimator.update(rotation, leftDistance, rightDistance);
        field.setRobotPose(pose);
    }

    /**
     * Asks for the latest list of seen apriltags, and updates the pose estimator
     * with the pose we figure from that
     * 
     * @param Camera - a Camera object that can return targets
     */
    public void updatePoseEstimateFromCamera(Camera camera) {
        // Only update if there's been an update since we last checked
        var last = camera.getLastMeasurementTimestamp();
        if (last <= lastVisionTimestamp) {
            return; // nothing new for us
        }
        lastVisionTimestamp = last;
        /* The RobotPoseEstimator will ask the camera for all found targets and come up with
        a Pose3d for the robot based on the discovered AprilTags. Feed this into the odometry's
        pose estimator */

        robotPoseEstimator.update().ifPresent(p -> {
            poseEstimator.addVisionMeasurement(p.getFirst().toPose2d(), p.getSecond());
        });
    }
}

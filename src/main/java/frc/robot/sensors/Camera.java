// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;

import java.util.Collections;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;

/**
 * Represents the camera on the bot, specifically the front facing one
 * This camera is intended for the odometry system to ask for a vision
 * based pose, and for the targeting system to help with bearing/range
 * to a particular target
 */
public class Camera extends SubsystemBase {
    private PhotonCamera camera = new PhotonCamera(RobotConstants.APRILTAG_CAMERA);
    private PhotonPipelineResult result;

    public Camera() {
        camera.setPipelineIndex(0);
    }

    /** Returns the underlying photoncamera */
    public PhotonCamera getPhotonCamera() {
        return camera;
    }
    /**
     * Does the camera see any targets?
     * @return if the camera sees targets
     */
    public boolean hasTargets() {
        return result != null && result.hasTargets();
    }

    /**
     * Gets the timestamp of the last time we took a measurement, for preventing duplication
     * @return timestamp in seconds
     */
    public double getLastMeasurementTimestamp() {
        return result.getTimestampSeconds();
    }

    public double getLastPipelineLatency() {
        return result.getLatencyMillis();
    }
    @Override
    public void periodic() {
        result = camera.getLatestResult();
    }

}

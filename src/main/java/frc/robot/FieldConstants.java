package frc.robot;

import java.util.Map;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

public final class FieldConstants {
    public static final AprilTag tag1 = new AprilTag(1, new Pose3d(Units.inchesToMeters(64.0), Units.inchesToMeters(27.5), 37.0/100, new Rotation3d(0,0, Math.PI) ));
    public static final Map<Integer, AprilTag> tags = Stream.of(new AprilTag[] { 
        tag1
    }).collect(Collectors.toMap(data -> data.ID, data -> data));

}
package frc.robot.util;

import java.util.ArrayList;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Util {

    public static boolean isRedAlliance () {

        Optional<Alliance> alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get().equals(Alliance.Red);
    }

    public static Translation2d getHubPosition () {

        Translation2d hubPosition = FieldConstants.Hub.innerCenterPoint.toTranslation2d();
        if (isRedAlliance()) { hubPosition = FieldConstants.Hub.oppTopCenterPoint.toTranslation2d(); }

        return hubPosition;
    }

    public static Rotation2d rotationToNearestCorner (Pose2d robotPose) {

        ArrayList<Translation2d> corners = new ArrayList<>();

        if (!isRedAlliance()) {

            corners.add(new Translation2d(0.0, 0.0));
            corners.add(new Translation2d(0.0, FieldConstants.fieldWidth));
        } else {

            corners.add(new Translation2d(FieldConstants.fieldLength, 0.0));
            corners.add(new Translation2d(FieldConstants.fieldLength, FieldConstants.fieldWidth));
        }

        Translation2d target = robotPose.getTranslation().nearest(corners);
        return target.minus(robotPose.getTranslation()).getAngle();
    }
}

package frc.robot.util;

import java.util.ArrayList;
import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;

public class Util {

    public static boolean isRedAlliance () {

        Optional<Alliance> alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get().equals(Alliance.Red);
    }

    public static Translation2d getLinearVelocityFromJoysticks(double x, double y) {

        // Apply deadband
        double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), Constants.kJoystickDeadband);
        Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

        // Square magnitude for more precise control
        linearMagnitude = linearMagnitude * linearMagnitude;

        // Return new linear velocity
        return new Pose2d(Translation2d.kZero, linearDirection)
            .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.kZero))
            .getTranslation();
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

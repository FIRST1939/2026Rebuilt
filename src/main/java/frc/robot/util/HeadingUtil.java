package frc.robot.util;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class HeadingUtil {

    public static Rotation2d headingToHub (Pose2d robotPose) {

        Translation2d hubPosition = FieldConstants.Hub.innerCenterPoint.toTranslation2d();
        Optional<Alliance> alliance = DriverStation.getAlliance();

        boolean flip = alliance.isPresent() && alliance.get().equals(Alliance.Red);
        if (flip) { hubPosition = FieldConstants.Hub.oppTopCenterPoint.toTranslation2d(); }

        return hubPosition.minus(robotPose.getTranslation()).getAngle();
    }
}

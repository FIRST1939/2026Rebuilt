package frc.robot.util;

import java.util.Optional;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Util {

    public static Translation2d getHubPosition () {

        Translation2d hubPosition = FieldConstants.Hub.innerCenterPoint.toTranslation2d();
        Optional<Alliance> alliance = DriverStation.getAlliance();

        boolean flip = alliance.isPresent() && alliance.get().equals(Alliance.Red);
        if (flip) { hubPosition = FieldConstants.Hub.oppTopCenterPoint.toTranslation2d(); }

        return hubPosition;
    }
}

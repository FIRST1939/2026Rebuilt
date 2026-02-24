package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * A solution finder whose RPM and hood position are tunable from the
 * dashboard via NetworkTables. Useful for quick iterative testing.
 */
public class QuickShotShooterSolutionFinder implements ShooterSolutionFinder {

    private final LoggedNetworkNumber m_flywheelRPM =
            new LoggedNetworkNumber("/QuickShot/Flywheel RPM", 3000);
    private final LoggedNetworkNumber m_hoodPosition =
            new LoggedNetworkNumber("/QuickShot/Hood Target Position", 0.05);

    private final Solution m_latestSolution = new Solution(3000, 0.05);

    @Override
    public Solution update(Pose2d robotPose, ChassisSpeeds robotSpeeds) {
        m_latestSolution.flywheelRPM = m_flywheelRPM.getAsDouble();
        m_latestSolution.hoodPositionRotations = m_hoodPosition.getAsDouble();
        return m_latestSolution;
    }

    @Override
    public Solution getLatestSolution() {
        return m_latestSolution;
    }
}

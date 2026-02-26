package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class IdleShooterSolutionFinder implements ShooterSolutionFinder {

    private Solution m_latestSolution = new Solution(1500.0, 0.0);

    @Override
    public Solution update(Pose2d robotPose, ChassisSpeeds robotSpeeds) {
        m_latestSolution.flywheelRPM = 1500.0;
        m_latestSolution.hoodPositionRotations = 0.0;
        return m_latestSolution;
    }

    @Override
    public Solution getLatestSolution() {
        return m_latestSolution;
    }
}

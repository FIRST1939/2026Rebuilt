package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class FixedShooterSolutionFinder implements ShooterSolutionFinder {

    
    private Solution m_latestSolution = new Solution(3500.0, 0.1);

    @Override
    public Solution update(Pose2d robotPose, ChassisSpeeds robotSpeeds) {
        m_latestSolution.flywheelRPM = 3500.0;
        m_latestSolution.hoodPositionRotations = 0.1;
        return m_latestSolution;
    }

    @Override
    public Solution getLatestSolution() {
        return m_latestSolution;
    }
}

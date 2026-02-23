package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;


public interface ShooterSolutionFinder {

    /** The solution produced by a solution finder. */
    public static class Solution {

        public double flywheelRPM;
        public double hoodPositionRotations;

        public Solution(double flywheelRPM, double hoodPositionRotations) {
            this.flywheelRPM = flywheelRPM;
            this.hoodPositionRotations = hoodPositionRotations;
        }
    }

    /** Calculate a new solution from the current robot state. */
    Solution update(Pose2d robotPose, ChassisSpeeds robotSpeeds);

    /** Calculate a solution using values read from NetworkTables (for simulation). */
    Solution updateSimSolution();

    /** Return the most recently computed solution. */
    Solution getLatestSolution();
}

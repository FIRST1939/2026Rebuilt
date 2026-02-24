package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;


public interface ShooterSolutionFinder {

    // Sim-tunable pose/velocity values shared by all solution finders.
    LoggedNetworkNumber SIM_ROBOT_POS_X = new LoggedNetworkNumber("/ShooterSolution/Robot Pos X", 0.0);
    LoggedNetworkNumber SIM_ROBOT_POS_Y = new LoggedNetworkNumber("/ShooterSolution/Robot Pos Y", 0.0);
    LoggedNetworkNumber SIM_ROBOT_VEL_X = new LoggedNetworkNumber("/ShooterSolution/Robot Vel X", 0.0);
    LoggedNetworkNumber SIM_ROBOT_VEL_Y = new LoggedNetworkNumber("/ShooterSolution/Robot Vel Y", 0.0);

    
    public static class Solution {

        public double flywheelRPM;
        public double hoodPositionRotations;

        public Solution(double flywheelRPM, double hoodPositionRotations) {
            this.flywheelRPM = flywheelRPM;
            this.hoodPositionRotations = hoodPositionRotations;
        }
    }

    // Calculate a new solution from the current robot state. 
    Solution update(Pose2d robotPose, ChassisSpeeds robotSpeeds);

    default Solution updateSimSolution() {
        Pose2d robotPose = new Pose2d(
                SIM_ROBOT_POS_X.getAsDouble(),
                SIM_ROBOT_POS_Y.getAsDouble(),
                new Rotation2d());
        ChassisSpeeds robotSpeeds = new ChassisSpeeds(
                SIM_ROBOT_VEL_X.getAsDouble(),
                SIM_ROBOT_VEL_Y.getAsDouble(),
                0.0);
        return update(robotPose, robotSpeeds);
    }
    
    
    Solution getLatestSolution();
}

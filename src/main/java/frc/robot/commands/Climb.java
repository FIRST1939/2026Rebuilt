package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.climber.Climber;
import frc.robot.commands.drive.DriveToPose;
import frc.robot.commands.climber.*;
import frc.robot.Constants.*;
import frc.robot.util.FieldConstants;

// TODO Climbing Sequence
public class Climb extends SequentialCommandGroup {
    
    public Climb (Drive drive, Climber climber) {

        super(
            Commands.parallel(
                new DriveToPose(drive, new Pose2d(1.27 + (2 * (FieldConstants.LinesVertical.center - 1.27)), FieldConstants.Tower.centerPoint.getY() - (4.74 - FieldConstants.Tower.centerPoint.getY()), new Rotation2d(Math.PI / 2))),
                new RaiseClimberToHeight(climber, ClimberConstants.kRaisingClimberSetpoint, ClimberConstants.kRaisingClimberPercentage)
            ),
            new DriveToPose(drive, new Pose2d(1.067 + (2 * (FieldConstants.LinesVertical.center - 1.067)), FieldConstants.Tower.centerPoint.getY() - (4.74 - FieldConstants.Tower.centerPoint.getY()), new Rotation2d(Math.PI / 2))),
            new DriveToPose(drive, new Pose2d(1.067 + (2 * (FieldConstants.LinesVertical.center - 1.067)), FieldConstants.Tower.centerPoint.getY() - (4.617 - FieldConstants.Tower.centerPoint.getY()), new Rotation2d(Math.PI / 2))),
            new LowerClimberToHeight(climber, ClimberConstants.kLoweringClimberSetpoint, ClimberConstants.kLoweringClimberPercentage)
        );
    }
}

package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.ShotSolver;

public class PPShootOnTheMoveRotation extends Command {
    
    private final Drive m_drive;
    private final ShotSolver m_shotSolver;
    private final ProfiledPIDController m_angleController;

    public PPShootOnTheMoveRotation(Drive drive, ShotSolver shotSolver) {

        m_drive = drive;
        m_shotSolver = shotSolver;

        m_angleController = new ProfiledPIDController(
            20.0,
            0.0,
            0.5,
            new TrapezoidProfile.Constraints(8.0, 20.0)
        );

        m_angleController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {

        m_angleController.reset(m_drive.getRotation().getRadians());

        DoubleSupplier omegaSupplier = () -> m_angleController.calculate(m_drive.getRotation().getRadians(), m_shotSolver.getShotSolution().aimHeading.getRadians());
        PPHolonomicDriveController.overrideRotationFeedback(omegaSupplier);
    }

    @Override
    public void end(boolean interrupted) {

        PPHolonomicDriveController.clearRotationFeedbackOverride();
    }
}

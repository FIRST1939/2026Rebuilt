package frc.robot.bindings;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.intake.IntakeStateManager;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.util.ShotSolver;

public class BindingParams {
    
    public final Drive drive;
    public final Intake intake;
    public final Spindexer spindexer;
    public final Feeder feeder;
    public final Shooter shooter;
    public final Climber climber;

    public final IntakeStateManager intakeStateManager;
    public final ShotSolver shotSolver;

    public final CommandXboxController driverController;
    public final CommandXboxController operatorController;

    public BindingParams(
        Drive drive,
        Intake intake,
        Spindexer spindexer,
        Feeder feeder,
        Shooter shooter,
        Climber climber,
        IntakeStateManager intakeStateManager,
        ShotSolver shotSolver,
        CommandXboxController driverController,
        CommandXboxController operatorController
    ) {

        this.drive = drive;
        this.intake = intake;
        this.spindexer = spindexer;
        this.feeder = feeder;
        this.shooter = shooter;
        this.climber = climber;

        this.intakeStateManager = intakeStateManager;
        this.shotSolver = shotSolver;

        this.driverController = driverController;
        this.operatorController = operatorController;
    }
}

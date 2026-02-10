package frc.robot.commands.intake;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class RunIntake extends Command {
    
    private final Intake intake;

    public RunIntake (Intake intake) {

        this.intake = intake;
        this.addRequirements(this.intake);
    }

    @Override
    public void initialize () {

        this.intake.setRollerVoltage(5.0);
    }

    @Override
    public void end (boolean interrupted) {

        this.intake.setRollerVoltage(0.0);
    }
}
package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class Intake extends SubsystemBase {

    private final IntakeIO m_io;
    private final IntakeIOInputsAutoLogged m_inputs = new IntakeIOInputsAutoLogged();


    public Intake (IntakeIO io) {

        m_io = io;
    }

     @Override
    public void periodic() {

        m_io.updateInputs(m_inputs);
        Logger.processInputs("Intake", m_inputs);

    }

    public void setRollerPercentage (double percentage) {
        this.m_io.setRollerPercentage(percentage);
    }

    public void setRollerVelocity (double velocity) {
        this.m_io.setRollerVelocity(velocity);
    }
    
    public double getRollerVelocity () {
        return this.m_inputs.rollerVelocity;
    }

    public void setPivotPercentage (double percentage) {
        this.m_io.setPivotPercentage(percentage);
    }

    public void setPivotPosition (double position) {
        this.m_io.setPivotPosition(position);
    }
    
    public double getPivotPosition () {
        return ((this.m_inputs.pivotLeaderPosition + this.m_inputs.pivotFollowerPosition) / 2);
    }
}


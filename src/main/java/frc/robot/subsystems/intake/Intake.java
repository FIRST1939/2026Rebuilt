package frc.robot.subsystems.intake;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Intake extends SubsystemBase {

    private final IntakeIO m_io;
    private final IntakeIOInputsAutoLogged m_inputs = new IntakeIOInputsAutoLogged();
    private final SysIdRoutine sysIdRoutine;

    
    public Intake (IntakeIO io) {
        m_io = io;
        this.sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.per(Units.Second).of(IntakeConstants.kSysIdRampUpTime), 
                Volts.of(IntakeConstants.kSysIdVoltageIncrement), 
                Seconds.of(IntakeConstants.kSysIdDuration)),

            new SysIdRoutine.Mechanism(
                voltage -> io.setRollerVoltage(voltage.magnitude()), 
                log -> {
                    log
                        .motor("intakeRoller")
                        .voltage(Volts.of(this.m_inputs.rollerVoltage))
                        .angularPosition(Radians.of(this.m_inputs.rollerPosition))
                        .angularVelocity(RotationsPerSecond.of(this.m_inputs.rollerVelocity));
                },
                this, 
                "Intake")
        );
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

    public void setRollerVoltage(double magnitude) {
        this.m_io.setRollerVoltage(magnitude);
    }
    
     public Command sysIdQuasistaticForward() {
        return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward);
    }

    public Command sysIdQuasistaticReverse() {
        return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse);
    }

    public Command sysIdDynamicForward() {
        return sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward);
    }

    public Command sysIdDynamicReverse() {
        return sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse);
    }

}


package frc.robot.subsystems.intake;

import com.revrobotics.sim.SparkFlexSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class IntakeIOSim extends IntakeIOHardware {
    
    private final SparkFlexSim m_rollerMotorSim = new SparkFlexSim(m_roller, DCMotor.getNeoVortex(1));

    private final FlywheelSim m_rollerPhysicsSim = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(
            DCMotor.getNeoVortex(1), 
            0.008, 
            IntakeConstants.kRollerGearing
        ), 
        DCMotor.getNeoVortex(1)
    );

    @Override
    public void updateInputs(IntakeIOInputs inputs) {

        m_rollerPhysicsSim.setInputVoltage(m_rollerMotorSim.getAppliedOutput() * m_rollerMotorSim.getBusVoltage());
        m_rollerPhysicsSim.update(0.02);

        m_rollerMotorSim.iterate(
            m_rollerPhysicsSim.getAngularVelocityRPM(), 
            RobotController.getBatteryVoltage(), 
            0.02
        );

        super.updateInputs(inputs);
    }
}

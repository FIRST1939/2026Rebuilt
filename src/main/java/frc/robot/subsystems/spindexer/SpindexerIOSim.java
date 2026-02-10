package frc.robot.subsystems.spindexer;

import com.revrobotics.sim.SparkFlexSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class SpindexerIOSim extends SpindexerIOHardware {
    
    private final SparkFlexSim m_motorSim = new SparkFlexSim(m_motor, DCMotor.getNeoVortex(1));

    private final FlywheelSim m_physicsSim = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(
            DCMotor.getNeoVortex(1), 
            0.036, 
            SpindexerConstants.kSpindexerGearing
        ), 
        DCMotor.getNeoVortex(1)
    );

    @Override
    public void updateInputs(SpindexerIOInputs inputs) {

        m_physicsSim.setInputVoltage(m_motorSim.getAppliedOutput() * m_motorSim.getBusVoltage());
        m_physicsSim.update(0.02);

        m_motorSim.iterate(
            m_physicsSim.getAngularVelocityRPM(), 
            RobotController.getBatteryVoltage(), 
            0.02
        );

        super.updateInputs(inputs);
    }
}

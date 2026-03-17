package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.revrobotics.sim.SparkFlexSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class IntakeIOSim extends IntakeIOHardware {

    private final SparkFlexSim m_rollerMotorSim = new SparkFlexSim(m_rollerMotor, DCMotor.getNeoVortex(1));
    private final SparkFlexSim m_leftPivotMotorSim = new SparkFlexSim(m_leftPivotMotor, DCMotor.getNeoVortex(1));
    private final SparkFlexSim m_rightPivotMotorSim = new SparkFlexSim(m_rightPivotMotor, DCMotor.getNeoVortex(1));

    private final FlywheelSim m_rollerPhysicsSim = new FlywheelSim(
        LinearSystemId.identifyVelocitySystem(
            RPM.of(IntakeConstants.kRollerFeedforwardV).in(RadiansPerSecond),
            RPM.of(IntakeConstants.kRollerFeedforwardA).in(RadiansPerSecond)
        ), 
        DCMotor.getNeoVortex(1)
    );

    private final SingleJointedArmSim m_leftPivotPhysicsSim = new SingleJointedArmSim(
        DCMotor.getNeoVortex(1),
        1.0 / IntakeConstants.kPivotGearing,
        0.067,
        0.178,
        Degrees.of(27.570246).in(Radians),
        Degrees.of(27.570246 + 103.5).in(Radians),
        true,
        Degrees.of(27.570246 + 103.5).in(Radians)
    );

    private final SingleJointedArmSim m_rightPivotPhysicsSim = new SingleJointedArmSim(
        DCMotor.getNeoVortex(1),
        1.0 / IntakeConstants.kPivotGearing,
        0.067,
        0.178,
        Degrees.of(27.570246).in(Radians),
        Degrees.of(27.570246 + 103.5).in(Radians),
        true,
        Degrees.of(27.570246 + 103.5).in(Radians)
    );

    @Override
    public void updateInputs(IntakeIOInputs inputs) {

        double rollerVoltage = m_rollerMotorSim.getAppliedOutput() * m_rollerMotorSim.getBusVoltage();
        boolean rollerOvercomeFriction = Math.abs(rollerVoltage) > IntakeConstants.kRollerFeedforwardS;
        m_rollerPhysicsSim.setInputVoltage(rollerOvercomeFriction ? rollerVoltage - Math.copySign(IntakeConstants.kRollerFeedforwardS, rollerVoltage) : 0);
        m_rollerPhysicsSim.update(0.02);

        m_rollerMotorSim.iterate(
            m_rollerPhysicsSim.getAngularVelocityRPM(), 
            RobotController.getBatteryVoltage(), 
            0.02
        );

        double leftPivotVoltage = -m_leftPivotMotorSim.getAppliedOutput() * m_leftPivotMotorSim.getBusVoltage();
        boolean leftPivotOvercomeFriction = Math.abs(leftPivotVoltage) > IntakeConstants.kLeftPivotFeedforwardS;
        m_leftPivotPhysicsSim.setInputVoltage(leftPivotOvercomeFriction ? leftPivotVoltage - Math.copySign(IntakeConstants.kLeftPivotFeedforwardS, leftPivotVoltage) : 0);
        m_leftPivotPhysicsSim.update(0.02);

        m_leftPivotMotorSim.iterate(
            RadiansPerSecond.of(m_leftPivotPhysicsSim.getVelocityRadPerSec()).in(RPM),
            RobotController.getBatteryVoltage(),
            0.02
        );

        double rightPivotVoltage = -m_rightPivotMotorSim.getAppliedOutput() * m_rightPivotMotorSim.getBusVoltage();
        boolean rightPivotOvercomeFriction = Math.abs(rightPivotVoltage) > IntakeConstants.kRightPivotFeedforwardS;
        m_rightPivotPhysicsSim.setInputVoltage(rightPivotOvercomeFriction ? rightPivotVoltage - Math.copySign(IntakeConstants.kRightPivotFeedforwardS, rightPivotVoltage) : 0);
        m_rightPivotPhysicsSim.update(0.02);

        m_rightPivotMotorSim.iterate(
            RadiansPerSecond.of(m_rightPivotPhysicsSim.getVelocityRadPerSec()).in(RPM),
            RobotController.getBatteryVoltage(),
            0.02
        );

        super.updateInputs(inputs);
    }
}

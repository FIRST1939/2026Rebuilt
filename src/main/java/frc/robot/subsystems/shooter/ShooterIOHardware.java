package frc.robot.subsystems.shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

// frc.robot.subsystems.spindexer.SpindexerConstants;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;


public class ShooterIOHardware implements ShooterIO {
    
    protected final SparkFlex m_flywheelLeader = new SparkFlex(ShooterConstants.kFlywheelLeaderCAN, MotorType.kBrushless);
    protected final RelativeEncoder m_flywheelLeaderEncoder = m_flywheelLeader.getEncoder();
    protected final SparkClosedLoopController m_flywheelController = m_flywheelLeader.getClosedLoopController();

    protected final SparkFlex m_flywheelFollower = new SparkFlex(ShooterConstants.kFlywheelFollowerCAN, MotorType.kBrushless);
    protected final RelativeEncoder m_flywheelFollowerEncoder = m_flywheelFollower.getEncoder();

    protected final SparkFlex m_hood = new SparkFlex(ShooterConstants.kHoodCAN, MotorType.kBrushless);
    protected final RelativeEncoder m_hoodEncoder = m_hood.getEncoder();
    protected final SparkClosedLoopController m_hoodController = m_hood.getClosedLoopController();

    public ShooterIOHardware () {
       
        SparkFlexConfig globalConfig = new SparkFlexConfig();

        globalConfig
            .idleMode(IdleMode.kCoast);

        globalConfig.encoder
            .positionConversionFactor(1.0 / ShooterConstants.kFlywheelGearReduction)
            .velocityConversionFactor(1.0 / ShooterConstants.kFlywheelGearReduction);


        SparkFlexConfig flywheelLeaderConfig = new SparkFlexConfig();
        flywheelLeaderConfig
            .apply(globalConfig)
            .inverted(ShooterConstants.kFlywheelLeaderInverted);
        m_flywheelLeader.configure(flywheelLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


        SparkFlexConfig flywheelFollowerConfig = new SparkFlexConfig();
        flywheelFollowerConfig
            .apply(globalConfig)
            .inverted(ShooterConstants.kFlywheelFollowerInverted);
            //.follow(m_flywheelLeader);
        m_flywheelFollower.configure(flywheelFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


        SparkFlexConfig hoodConfig = new SparkFlexConfig();

        hoodConfig
            .idleMode(IdleMode.kBrake);

        hoodConfig.encoder
            .positionConversionFactor(1.0 / ShooterConstants.kHoodGearReduction)
            .velocityConversionFactor(1.0 / ShooterConstants.kHoodGearReduction);

         hoodConfig
         .closedLoop
               .pid(0.0010000000474974513, 0, 0,ClosedLoopSlot.kSlot0)
               .maxMotion.maxAcceleration(24000000,ClosedLoopSlot.kSlot0)
               .cruiseVelocity(2400000,ClosedLoopSlot.kSlot0)
             .allowedProfileError(.02,ClosedLoopSlot.kSlot0);
              
        
            // .feedForward
            //     .kS(SpindexerConstants.kSpindexerFeedforwardS, ClosedLoopSlot.kSlot0)
            //     .kV(SpindexerConstants.kSpindexerFeedforwardV, ClosedLoopSlot.kSlot0)
            //     .kA(SpindexerConstants.kSpindexerFeedforwardA, ClosedLoopSlot.kSlot0);    
        m_hood.configure(hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }


    @Override
    public void updateInputs (ShooterIOInputs inputs) {

        inputs.flywheelLeaderPosition = m_flywheelLeaderEncoder.getPosition();
        inputs.flywheelLeaderVelocity = m_flywheelLeaderEncoder.getVelocity();
        inputs.flywheelLeaderVoltage = m_flywheelLeader.getAppliedOutput() * m_flywheelLeader.getBusVoltage();
        inputs.flywheelLeaderCurrent = m_flywheelLeader.getOutputCurrent();
        inputs.flywheelLeaderTemperature = m_flywheelLeader.getMotorTemperature();

        inputs.flywheelFollowerPosition = m_flywheelFollowerEncoder.getPosition();
        inputs.flywheelFollowerVelocity = m_flywheelFollowerEncoder.getVelocity();
        inputs.flywheelFollowerVoltage = m_flywheelFollower.getAppliedOutput() * m_flywheelFollower.getBusVoltage();
        inputs.flywheelFollowerCurrent = m_flywheelFollower.getOutputCurrent();
        inputs.flywheelFollowerTemperature = m_flywheelFollower.getMotorTemperature();

        inputs.hoodPosition = m_hoodEncoder.getPosition();
        inputs.hoodVelocity = m_hoodEncoder.getVelocity();
        inputs.hoodVoltage = m_hood.getAppliedOutput() * m_hood.getBusVoltage();
        inputs.hoodCurrent = m_hood.getOutputCurrent();
        inputs.hoodTemperature = m_hood.getMotorTemperature();
    }

    @Override
    public void setFlywheelPercentage (double percent) {
        m_flywheelLeader.set(percent);
        m_flywheelFollower.set(percent);
        //m_flywheelController.setSetpoint(percent, ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot0);
    }

    @Override
    public void setHoodPercentage (double percentage) {
        m_hood.set(percentage);
    }   

    @Override
    public void setHoodPosition (double position) {
       // m_hoodController.setSetpoint(position, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
        m_hoodController.setSetpoint(position, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
    }    
}
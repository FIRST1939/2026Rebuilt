package frc.robot.subsystems.climber;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ClimberIOHardware implements ClimberIO {
    
    protected final SparkFlex m_motor = new SparkFlex(ClimberConstants.kClimberCAN,MotorType.kBrushless);
}

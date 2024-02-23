package frc.robot.utils;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;

public class SparkMaxMotor extends CanSparkMotor {
    
    public SparkMaxMotor(int canID) {
        super(new CANSparkMax(canID, CANSparkLowLevel.MotorType.kBrushless));
    }
    
}

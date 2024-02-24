package frc.robot.utiils;


import com.revrobotics.CANSparkMax;
import frc.robot.utils.CanSparkMotor;
import com.revrobotics.CANSparkLowLevel;

public class SparkMaxMotor extends CanSparkMotor {
    
    public SparkMaxMotor(int canID) {
        super(new CANSparkMax(canID, CANSparkLowLevel.MotorType.kBrushless));
    }
    
}

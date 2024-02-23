package frc.robot.utils;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class SparkFlexMotor extends CanSparkMotor {

    public SparkFlexMotor(int canID) {
        super(new CANSparkFlex(canID, MotorType.kBrushless));
    }
}
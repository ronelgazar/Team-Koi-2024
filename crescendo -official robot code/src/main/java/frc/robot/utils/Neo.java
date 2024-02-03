package frc.robot.utils;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkLimitSwitch;

import frc.robot.Constants;

public class Neo {
    private CANSparkMax motorController;
    private SparkPIDController pidController;
    private AbsoluteEncoder absEncoder;
    private RelativeEncoder relEncoder;
    private boolean isRelative = true;

    public Neo(int deviceID, boolean isRelative){
        motorController = new CANSparkMax(deviceID, MotorType.kBrushless);
        motorController.restoreFactoryDefaults();
        pidController = motorController.getPIDController();
        this.isRelative = isRelative;
        if(isRelative){
            this.relEncoder = motorController.getAlternateEncoder(Constants.MotorConstants.NEO_MOTOR_COUNTS_PER_REV);
            pidController.setFeedbackDevice(relEncoder);
        } else{
            this.absEncoder = motorController.getAbsoluteEncoder(Type.kDutyCycle);
            pidController.setFeedbackDevice(absEncoder);
        }
    }

    public void setPIDValues(PIDValues pidValues){
        pidController.setP(pidValues.P());
        pidController.setI(pidValues.I());
        pidController.setD(pidValues.D());
        pidController.setIZone(pidValues.IZ());
        pidController.setFF(pidValues.FF());
    }

    public void setIdleMode(boolean coast){
        if(coast){
            motorController.setIdleMode(IdleMode.kCoast);
        } else{
            motorController.setIdleMode(IdleMode.kBrake);
        }
    }

    public void setPower(double power){
        if(power > 1){
            motorController.set(1);
        } else if(power < -1){
            motorController.set(-1);
        } else{
            motorController.set(power);
        }
    }

    public void setTarget(double target, ControlType gControlType){
        pidController.setReference(target, gControlType);
    }

    public static void follow(){

    }

    
}
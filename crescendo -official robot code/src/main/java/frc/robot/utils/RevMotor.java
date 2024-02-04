package frc.robot.utils;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;


public abstract class RevMotor {
    private CANSparkBase motorController;
    private SparkPIDController pidController;
    private AbsoluteEncoder absEncoder;
    private RelativeEncoder relEncoder;
    private boolean isRelative = true;

    public RevMotor(int deviceID, boolean isRelative, boolean isNeo){
        if(isNeo){
            motorController = new CANSparkMax(deviceID, MotorType.kBrushless);
        } else{
            motorController = new CANSparkFlex(deviceID, MotorType.kBrushless);
        }
        motorController.restoreFactoryDefaults();
        pidController = motorController.getPIDController();
        this.isRelative = isRelative;
        if(isRelative){
            this.relEncoder = motorController.getEncoder();
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
        motorController.set(power);
    }

    public void setTarget(double target, ControlType gControlType){
        pidController.setReference(target, gControlType);
    }

    public void follow(CANSparkBase leader){
        motorController.follow(leader);
    }

    public void setInverted(boolean inverted){
        motorController.setInverted(inverted);
    }

    public CANSparkBase getMotorController(){
        return motorController;
    }

    public void setFeedbackDevice(MotorFeedbackSensor sensor){
        pidController.setFeedbackDevice(sensor);
    }
}

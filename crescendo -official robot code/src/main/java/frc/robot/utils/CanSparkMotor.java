package frc.robot.utils;

import com.revrobotics.*;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import frc.robot.Constants.PIDConstants;


public abstract class CanSparkMotor {

    private CANSparkBase motorController;
    private SparkPIDController pidController;
    private RelativeEncoder encoder;

    public CanSparkMotor(CANSparkBase motorController) {
        this.motorController = motorController;
        this.pidController = motorController.getPIDController();
        this.encoder = motorController.getEncoder();
    }

    public void set(double speed) {
        motorController.set(speed);
    }

    public void setInverted(boolean isInverted) {
        motorController.setInverted(isInverted);
    }

    public void follow(CanSparkMotor leader) {
        motorController.follow(leader.motorController);
    }

    public void setPIDConstants(PIDConstants pidConstants) {
        pidController.setP(pidConstants.P);
        pidController.setI(pidConstants.I);
        pidController.setD(pidConstants.D);
        pidController.setFF(pidConstants.FF);
        pidController.setIZone(pidConstants.IZONE);

    }

    public void setSetPoint(double setPoint, ControlType controlType) {
        pidController.setReference(setPoint, controlType);

    }

    public void setFeedbackDevice(RelativeEncoder encoder) {

        pidController.setFeedbackDevice(encoder);
    }

    public void setFeedbackDevice(AbsoluteEncoder encoder) {
        pidController.setFeedbackDevice(encoder);
    }

    public void setIdleMode(IdleMode mode) {
        motorController.setIdleMode(mode);
    }

}

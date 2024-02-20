package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.utiils.BrushedMotor;
import frc.robot.utiils.SparkFlexMotor;
import frc.robot.utiils.SparkMaxMotor;

public class ShootingSubSystem {
    private SparkFlexMotor shootingMotorFront;
    private SparkFlexMotor shootingMotorBack;
    private SparkMaxMotor aimingMotor;
    private BrushedMotor deliveryToShootingMotor;

    public ShootingSubSystem(){

        shootingMotorFront = new SparkFlexMotor(Constants.ShooterMotorConstants.kShootingMotorRightPort);
        shootingMotorBack = new SparkFlexMotor(Constants.ShooterMotorConstants.kShootingMotorLeftPort);
        shootingMotorBack.follow(shootingMotorFront);
        aimingMotor = new SparkMaxMotor(Constants.ShooterMotorConstants.kAimingMotorPort);
        deliveryToShootingMotor = new BrushedMotor(Constants.ShooterMotorConstants.kShootingMotorRightPort);

    }
    public void setShootersVelocity (double Velocity){

        shootingMotorFront.set(Velocity);
    }
    public void setAimPosition (double Position){
        aimingMotor.set(Position);
    }
    public void setDeliveryVelocity (double Velocity){
        deliveryToShootingMotor.set(Velocity);
    }
}
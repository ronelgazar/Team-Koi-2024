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

        shootingMotorFront = new SparkFlexMotor(Constants.ShooterMotorPortConstants.kShootingMotorFrontPort);
        shootingMotorBack = new SparkFlexMotor(Constants.ShooterMotorPortConstants.kShootingMotorBackPort);
        shootingMotorBack.follow(shootingMotorFront);
        aimingMotor = new SparkMaxMotor(Constants.ShooterMotorPortConstants.kAimingMotorPort);
        deliveryToShootingMotor = new BrushedMotor(Constants.ShooterMotorPortConstants.DeliveryToShootingMotorPort);

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
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.BrushedMotor;
import frc.robot.utils.SparkFlexMotor;
import frc.robot.utils.SparkMaxMotor;
public class ShootingSubSystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  
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
    // Legit no idea how that's supposed to work, it's going to stay here incase the one who wrote this explains what the hell it's doing here.
    public void setAimPosition (double Position){
        aimingMotor.set(Position);
    }

    //The small wheels that deliver to the shooting system.
    public void setDeliveryVelocity (double Velocity){
        deliveryToShootingMotor.set(Velocity);
    }

}
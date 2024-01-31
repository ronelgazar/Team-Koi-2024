// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShootingSubsystem extends SubsystemBase {

    private CANSparkMax[] m_shooters; // i think we have 2 shooting motors
    private CANSparkMax m_delivery;

    public ShootingSubsystem() {
        m_shooters = new CANSparkMax[] { 
            new CANSparkMax(Constants.Id.Motors.Shooter.SHOOTER_MOTOR_1, MotorType.kBrushless), 
            new CANSparkMax(Constants.Id.Motors.Shooter.SHOOTER_MOTOR_2, MotorType.kBrushless)
        };
        
        m_shooters[2].follow(m_shooters[1]);
        m_delivery = new CANSparkMax(Constants.Id.Motors.Shooter.FEED_MOTOR, MotorType.kBrushless);
    }

    public void shoot(double height, double distance) {
      //need to calculate and implement accordingly
    }

    public void shoot() {
      //automatic shooter function
      //should get target distance and height from the cameras
    }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    return false;
  }

    @Override
    public void periodic() {
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulationz
    }
}
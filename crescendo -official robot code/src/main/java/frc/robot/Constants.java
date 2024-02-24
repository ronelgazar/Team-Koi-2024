// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class Convert {
    public static final double KPH_TO_MPH = 0.621371; // blame google if this is wrong

    public static final double MPH_TO_KPH = 1 / KPH_TO_MPH;

    public static final double INCHES_TO_METERS = 0.0254;

    public static final double METERS_TO_INCHES = 1 / INCHES_TO_METERS;

    public static final double RADIANS_TO_DEGREES = 57.2957795;

    public static final double DEGREES_TO_RADIANS = 1 / RADIANS_TO_DEGREES;
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class MotorAttributes {
    public static class Neo {
      public static final double COUNTS_PER_REVOLUTION_SparkMax = 42;
      public static final double GEAR_RATIO = GearRatio.SwerveModule.STEERING_GEAR_RATIO;
      public static final double CONVERSION_FACTOR = GEAR_RATIO / COUNTS_PER_REVOLUTION_SparkMax;
      public static final double MAX_OUTPUT_ENCODER = 10000; //rpm
      public static final double MIN_OUTPUT_ENCODER = 0; //rpm
    }

    public static class Vortex {
      public static final double COUNTS_PER_REVOLUTION_SparkMax = 42;
      public static final double COUNTS_PER_REVOLUTION_SparkFlex = 7168;
    }
  }

  public static class GearRatio {
    public static class SwerveModule {
      public static final double STEERING_GEAR_RATIO = 12.8; // website says 12.8 : 1
                                                             // https://www.swervedrivespecialties.com/products/mk4-swerve-module
      public static final double DRIVE_GEAR_RATIO = 6.75; // website says 6.75 : 1
    }
  }

  public static class Id {
    public static class Motors {
      public static class Shooter {
        public static final int SHOOTER_MOTOR_1 = 0;
        public static final int SHOOTER_MOTOR_2 = 0;
        public static final int FEED_MOTOR = 0;
        public static final int AIMING_MOTOR = 0;
      }

      public static class Drive {
        public static class Modules {
          public static class FrontLeft {
            public static final int ROTATION_MOTOR = 0;
            public static final int DRIVE_MOTOR = 0;
          }

          public static class FrontRight {
            public static final int ROTATION_MOTOR = 0;
            public static final int DRIVE_MOTOR = 0;
          }

          public static class BackLeft {
            public static final int ROTATION_MOTOR = 0;
            public static final int DRIVE_MOTOR = 0;
          }

          public static class BackRight {
            public static final int ROTATION_MOTOR = 0;
            public static final int DRIVE_MOTOR = 0;
          }
        }
      }
      public static class Delivery {
        public static final int MOTOR1 = 0;
        public static final int MOTOR2 = 0;
      }
    }
  }

  public class PIDConstants {
    public static final double P = 0;
    public static final double I = 0;
    public static final double D = 0;
    public static final double FF = 0;
    public static final double IZONE = 0;
    
  public static class MotorConstants{
    public static final int NEO_MOTOR_COUNTS_PER_REV = 42;
  }

  public static class NeoMotorConstants {
    public static final double POS_CONVERSION_FACTOR = 0; //NEED TO CALCULATE
    public static final double VEL_CONVERSION_FACTOR = 0; //NEED TO CALCULATE
  }

  public static class ShooterMotorPortConstants {
    public static final int kShootingMotorFrontPort = 1;
    public static final int kShootingMotorBackPort = 2;
    public static final int kAimingMotorPort = 3;
    public static final int DeliveryToShootingMotorPort = 4;
  }
  public static class DeliveryMotorPortConstants {
    public static final int kDeliveryMotorRightPort = 5;
    public static final int kDeliveryMotorLeftPort = 6;

  }
}

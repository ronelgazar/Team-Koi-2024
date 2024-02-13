package frc.robot.subsystems;

import java.io.File;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;

public class DriveSubsystem extends SubsystemBase {
    private SwerveDrive drive;

    private double maximumSpeed = 10.8; // in kilometers per hour

    public double getMaximumSpeed() {
        return maximumSpeed;
    }

    public void setMaximumSpeed(double maximumSpeed) {
        this.maximumSpeed = maximumSpeed;
    }

    public Pose2d getPose() {
        return drive.getPose();
    }

    public void resetOdometry(Pose2d initial) {
        drive.resetOdometry(initial);
    }

    public ChassisSpeeds getRobotVelocity() {
        return drive.getRobotVelocity();
    }

    public void setChassisSpeeds(ChassisSpeeds cs) {
        drive.setChassisSpeeds(cs);
    }

    public DriveSubsystem(File directory) {
        double angleConversionFactor = SwerveMath.calculateDegreesPerSteeringRotation(
                Constants.GearRatio.SwerveModule.STEERING_GEAR_RATIO,
                Constants.MotorAttributes.Neo.COUNTS_PER_REVOLUTION_SparkMax /* encoder resolution i think */
        );

        double wheelDiameter = Constants.Convert.INCHES_TO_METERS * 4;

        double driveConversionFactor = SwerveMath.calculateMetersPerRotation(
                Constants.Convert.INCHES_TO_METERS * 4,
                Constants.GearRatio.SwerveModule.DRIVE_GEAR_RATIO,
                Constants.MotorAttributes.Vortex.COUNTS_PER_REVOLUTION_SparkFlex);

        try {
            drive = new SwerveParser(directory).createSwerveDrive(maximumSpeed);
        } catch (Exception e) {
            throw new RuntimeException(
                    "Error with directory. Please make sure the give File object is a directory and not a file");
        }

        drive.setHeadingCorrection(false);

        setupPathPlanner();
    }

    public void setupPathPlanner() {
        AutoBuilder.configureHolonomic(
                this::getPose,
                this::resetOdometry,
                this::getRobotVelocity,
                this::setChassisSpeeds,
                new HolonomicPathFollowerConfig(
                        new PIDConstants(5.0, 0.0, 0.0),
                        new PIDConstants(drive.swerveController.config.headingPIDF.p,
                                drive.swerveController.config.headingPIDF.i,
                                drive.swerveController.config.headingPIDF.d),
                        4.5,
                        drive.swerveDriveConfiguration.getDriveBaseRadiusMeters(),
                        new ReplanningConfig()),
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent())
                        return alliance.get() == DriverStation.Alliance.Red;
                    return false;
                },
                (Subsystem) this);
    }

    public Command driveToPose(Pose2d pose) {
        // Create the constraints to use while pathfinding
        PathConstraints constraints = new PathConstraints(
                /* drive.getMaximumVelocity() */ maximumSpeed, 4.0,
                15, Constants.Convert.DEGREES_TO_RADIANS * 720);

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        return AutoBuilder.pathfindToPose(
                pose,
                constraints,
                0.0, // Goal end velocity in meters/sec
                0.0 // Rotation delay distance in meters. This is how far the robot should travel
                    // before attempting to rotate.
        );
    }

   public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
                              DoubleSupplier headingY)
  {
    // swerveDrive.setHeadingCorrection(true); // Normally you would want heading correction for this kind of control.
    return run(() -> {
      double xInput = Math.pow(translationX.getAsDouble(), 3); // Smooth controll out
      double yInput = Math.pow(translationY.getAsDouble(), 3); // Smooth controll out
      // Make the robot move
      driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(xInput, yInput,
                                                                      headingX.getAsDouble(),
                                                                      headingY.getAsDouble(),
                                                                      swerveDrive.getOdometryHeading().getRadians(),
                                                                      swerveDrive.getMaximumVelocity()));
    });
  }

  /**
   * Command to drive the robot using translative values and heading as a setpoint.
   *
   * @param translationX Translation in the X direction.
   * @param translationY Translation in the Y direction.
   * @param rotation     Rotation as a value between [-1, 1] converted to radians.
   * @return Drive command.
   */
  public Command simDriveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotation)
  {
    // swerveDrive.setHeadingCorrection(true); // Normally you would want heading correction for this kind of control.
    return run(() -> {
      // Make the robot move
      driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(translationX.getAsDouble(),
                                                                      translationY.getAsDouble(),
                                                                      rotation.getAsDouble() * Math.PI,
                                                                      swerveDrive.getOdometryHeading().getRadians(),
                                                                      swerveDrive.getMaximumVelocity()));
    });
  }

  /**
   * Command to drive the robot using translative values and heading as angular velocity.
   *
   * @param translationX     Translation in the X direction. Cubed for smoother controls.
   * @param translationY     Translation in the Y direction. Cubed for smoother controls.
   * @param angularRotationX Angular velocity of the robot to set. Cubed for smoother controls.
   * @return Drive command.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX)
  {
    return run(() -> {
      drive.drive(new Translation2d(Math.pow(translationX.getAsDouble(), 3) * drive.getMaximumVelocity(),
                                          Math.pow(translationY.getAsDouble(), 3) * drive.getMaximumVelocity()),
                        Math.pow(angularRotationX.getAsDouble(), 3) * drive.getMaximumAngularVelocity(),
                        true,
                        false);
    });
  }

  /**
   * The primary method for controlling the drivebase.  Takes a {@link Translation2d} and a rotation rate, and
   * calculates and commands module states accordingly.  Can use either open-loop or closed-loop velocity control for
   * the wheel velocities.  Also has field- and robot-relative modes, which affect how the translation vector is used.
   *
   * @param translation   {@link Translation2d} that is the commanded linear velocity of the robot, in meters per
   *                      second. In robot-relative mode, positive x is torwards the bow (front) and positive y is
   *                      torwards port (left).  In field-relative mode, positive x is away from the alliance wall
   *                      (field North) and positive y is torwards the left wall when looking through the driver station
   *                      glass (field West).
   * @param rotation      Robot angular rate, in radians per second. CCW positive.  Unaffected by field/robot
   *                      relativity.
   * @param fieldRelative Drive mode.  True for field-relative, false for robot-relative.
   */
  public void drive(Translation2d translation, double rotation, boolean fieldRelative)
  {
    drive.drive(translation,
                      rotation,
                      fieldRelative,
                      false); // Open loop is disabled since it shouldn't be used most of the time.
  }

  /**
   * Drive the robot given a chassis field oriented velocity.
   *
   * @param velocity Velocity according to the field.
   */
  public void driveFieldOriented(ChassisSpeeds velocity)
  {
    drive.driveFieldOriented(velocity);
  }

  /**
   * Drive according to the chassis robot oriented velocity.
   *
   * @param velocity Robot oriented {@link ChassisSpeeds}
   */
  public void drive(ChassisSpeeds velocity)
  {
    drive.drive(velocity);
  }

  public void periodic()
  {
  }

  public void simulationPeriodic()
  {
  }

  /**
   * Get the swerve drive kinematics object.
   *
   * @return {@link SwerveDriveKinematics} of the swerve drive.
   */
  public SwerveDriveKinematics getKinematics()
  {
    return drive.kinematics;
  }

  /**
   * Post the trajectory to the field.
   *
   * @param trajectory The trajectory to post.
   */
  public void postTrajectory(Trajectory trajectory)
  {
    drive.postTrajectory(trajectory);
  }

  /**
   * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0.
   */
  public void zeroGyro()
  {
    drive.zeroGyro();
  }

  /**
   * Sets the drive motors to brake/coast mode.
   *
   * @param brake True to set motors to brake mode, false for coast.
   */
  public void setMotorBrake(boolean brake)
  {
    drive.setMotorIdleMode(brake);
  }

  /**
   * Gets the current yaw angle of the robot, as reported by the swerve pose estimator in the underlying drivebase.
   * Note, this is not the raw gyro reading, this may be corrected from calls to resetOdometry().
   *
   * @return The yaw angle
   */
  public Rotation2d getHeading()
  {
    return getPose().getRotation();
  }

  /**
   * Get the chassis speeds based on controller input of 2 joysticks. One for speeds in which direction. The other for
   * the angle of the robot.
   *
   * @param xInput   X joystick input for the robot to move in the X direction.
   * @param yInput   Y joystick input for the robot to move in the Y direction.
   * @param headingX X joystick which controls the angle of the robot.
   * @param headingY Y joystick which controls the angle of the robot.
   * @return {@link ChassisSpeeds} which can be sent to th Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY)
  {
    xInput = Math.pow(xInput, 3);
    yInput = Math.pow(yInput, 3);
    return drive.swerveController.getTargetSpeeds(xInput,
                                                        yInput,
                                                        headingX,
                                                        headingY,
                                                        getHeading().getRadians(),
                                                        maximumSpeed);
  }

  /**
   * Get the chassis speeds based on controller input of 1 joystick and one angle. Control the robot at an offset of
   * 90deg.
   *
   * @param xInput X joystick input for the robot to move in the X direction.
   * @param yInput Y joystick input for the robot to move in the Y direction.
   * @param angle  The angle in as a {@link Rotation2d}.
   * @return {@link ChassisSpeeds} which can be sent to th Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle)
  {
    xInput = Math.pow(xInput, 3);
    yInput = Math.pow(yInput, 3);
    return drive.swerveController.getTargetSpeeds(xInput,
                                                        yInput,
                                                        angle.getRadians(),
                                                        getHeading().getRadians(),
                                                        maximumSpeed);
  }

  /**
   * Gets the current field-relative velocity (x, y and omega) of the robot
   *
   * @return A ChassisSpeeds object of the current field-relative velocity
   */
  public ChassisSpeeds getFieldVelocity()
  {
    return drive.getFieldVelocity();
  }

  /**
   * Get the {@link SwerveController} in the swerve drive.
   *
   * @return {@link SwerveController} from the {@link SwerveDrive}.
   */
  public SwerveController getSwerveController()
  {
    return drive.swerveController;
  }

  /**
   * Get the {@link SwerveDriveConfiguration} object.
   *
   * @return The {@link SwerveDriveConfiguration} fpr the current drive.
   */
  public SwerveDriveConfiguration getSwerveDriveConfiguration()
  {
    return drive.swerveDriveConfiguration;
  }

  /**
   * Lock the swerve drive to prevent it from moving.
   */
  public void lock()
  {
    drive.lockPose();
  }

  /**
   * Gets the current pitch angle of the robot, as reported by the imu.
   *
   * @return The heading as a {@link Rotation2d} angle
   */
  public Rotation2d getPitch()
  {
    return drive.getPitch();
  }

  /**
   * Add a fake vision reading for testing purposes.
   */
  public void addFakeVisionReading()
  {
    drive.addVisionMeasurement(new Pose2d(3, 3, Rotation2d.fromDegrees(65)), Timer.getFPGATimestamp());
  }
}
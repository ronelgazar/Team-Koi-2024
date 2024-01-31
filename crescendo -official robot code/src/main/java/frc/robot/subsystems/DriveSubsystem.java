package frc.robot.subsystems;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;

public class DriveSubsystem {
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
}
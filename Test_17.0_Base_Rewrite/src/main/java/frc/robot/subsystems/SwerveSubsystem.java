package frc.robot.subsystems;

import java.io.File;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {
  private final SwerveDrive swerveDrive;

  public SwerveSubsystem() {
    // Directory containing swervedrive.json and module configs
    File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");

    // Keep telemetry light by default
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.LOW;

    try {
      // Create the swerve drive from YAGSL configs
      swerveDrive =
          new SwerveParser(swerveJsonDirectory)
              .createSwerveDrive(Constants.Drive.MAX_LINEAR_SPEED_MPS);
    } catch (Exception e) {
      throw new RuntimeException("Failed to create SwerveDrive from configuration", e);
    }
  }

  /** Direct drive using translation and rotation (radians/sec). */
  public void drive(Translation2d translation, double rotationRadPerSec, boolean fieldRelative) {
    swerveDrive.drive(translation, rotationRadPerSec, fieldRelative, false);
  }

  /** Drive directly from chassis speeds (robot- or field-relative chosen by caller). */
  public void drive(ChassisSpeeds speeds, boolean fieldRelative) {
    if (fieldRelative) {
      swerveDrive.driveFieldOriented(speeds);
    } else {
      swerveDrive.drive(speeds);
    }
  }

  /** Build a Command that continuously drives field-relative from a ChassisSpeeds supplier. */
  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
    return run(() -> swerveDrive.driveFieldOriented(velocity.get()));
  }

  /** Current robot pose from YAGSL's internal odometry. */
  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  /** Heading from the swerve pose. */
  public Rotation2d getHeading() {
    return swerveDrive.getPose().getRotation();
  }

  /** Reset gyro heading to 0. */
  public void zeroGyro() {
    swerveDrive.zeroGyro();
  }

  /** Expose underlying YAGSL SwerveDrive (used by SwerveInputStream). */
  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }

  @Override
  public void periodic() {
    // Keep YAGSL odometry updated
    swerveDrive.updateOdometry();
  }
}

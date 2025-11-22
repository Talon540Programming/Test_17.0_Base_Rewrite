package frc.robot.subsystems;

import java.io.File;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {

  private final SwerveDrive swerveDrive;

  //  Added: Field2d object for AdvantageScope / dashboards
  private final Field2d field = new Field2d();

  public SwerveSubsystem() {
    File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");

    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.LOW;

    try {
      swerveDrive =
          new SwerveParser(swerveJsonDirectory)
              .createSwerveDrive(Constants.Drive.MAX_LINEAR_SPEED_MPS);
    } catch (Exception e) {
      throw new RuntimeException("Failed to create SwerveDrive from configuration", e);
    }

    //  Publish the field to NetworkTables so AdvantageScope can see it
    SmartDashboard.putData("Field", field);
  }

  public void drive(Translation2d translation, double rotationRadPerSec, boolean fieldRelative) {
    swerveDrive.drive(translation, rotationRadPerSec, fieldRelative, false);
  }

  public void drive(ChassisSpeeds speeds, boolean fieldRelative) {
    if (fieldRelative) {
      swerveDrive.driveFieldOriented(speeds);
    } else {
      swerveDrive.drive(speeds);
    }
  }

  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
    return run(() -> swerveDrive.driveFieldOriented(velocity.get()));
  }

  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  public Rotation2d getHeading() {
    return swerveDrive.getPose().getRotation();
  }

  public void zeroGyro() {
    swerveDrive.zeroGyro();
  }

  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }

  //  Optional getter if you ever want direct access to the field
  public Field2d getField() {
    return field;
  }

  @Override
  public void periodic() {
    // Keep YAGSL odometry updated
    swerveDrive.updateOdometry();

    //  Update the Field2d pose so AdvantageScope can draw the robot
    field.setRobotPose(getPose());
  }
}

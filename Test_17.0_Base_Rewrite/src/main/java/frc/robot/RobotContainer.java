package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

public class RobotContainer {

  // Subsystems
  private final SwerveSubsystem driveBase = new SwerveSubsystem();

  // Driver controller (command-based wrapper)
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);

  // Rotation scale: 30% in sim, 100% on real robot
  private static final double ROTATION_SCALE =
      RobotBase.isSimulation() ? 0.05 : 1.0;

  // YAGSL drive input stream
  private final SwerveInputStream driveAngularVelocity =
      SwerveInputStream.of(
              driveBase.getSwerveDrive(),
              () -> -m_driverController.getLeftY(),  // forward/back
              () -> -m_driverController.getLeftX()) // strafe
          .withControllerRotationAxis(() -> -m_driverController.getRightX()) // rotation
          .deadband(OperatorConstants.DEADBAND)
          .scaleTranslation(1.0)          // full translation speed
          .scaleRotation(ROTATION_SCALE)  // 30% in sim, 100% on real
          .allianceRelativeControl(true); // always field-relative

  // You can copy() and rescale driveRegular later for slow mode, etc.
  private final SwerveInputStream driveRegular = driveAngularVelocity.copy();

  // Default drive command: field-oriented from driver sticks
  private final Command driveFieldOrientedAngularVelocity =
      driveBase.driveFieldOriented(driveRegular);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    // Default command: field-relative swerve drive
    driveBase.setDefaultCommand(driveFieldOrientedAngularVelocity);

    // Start button: reset gyro
    m_driverController
        .start()
        .onTrue(Commands.runOnce(driveBase::zeroGyro, driveBase));
  }

  /** Placeholder auto; replace with your real auto later. */
  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured.");
  }
}

package frc.robot;

import edu.wpi.first.math.util.Units;

public final class Constants {
  private Constants() {}

  public static final class Drive {
    // From your DriveConstants
    public static final double TRACK_WIDTH_METERS = Units.inchesToMeters(20.75);
    public static final double WHEEL_BASE_METERS = Units.inchesToMeters(20.75);

    // From your DriveConstants.maxLinearVelocityMetersPerSec (15.1 ft/s)
    public static final double MAX_LINEAR_SPEED_MPS = Units.feetToMeters(15.1);
  }

  public static final class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final double DEADBAND = 0.1;
  }
}

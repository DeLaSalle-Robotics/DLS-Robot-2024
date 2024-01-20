package frc.robot;

public final class Constants {
  public static class Shooter {
    public static final int kShooterMotorID1 = 1;
    public static final int kShooterMotorID2 = 7;
  }

  public static class Intake {
    public static final int kIntakeMotorID = 2;
    @Deprecated
    /** Marked as deprecated as this has not yet been determined */
    public static final int kIntakeLimitSwitchID = 0;
  }

  public static final double kShooterMaxVolts = 5.0;
}

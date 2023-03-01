// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kDriverControllerPort2 = 1;
  }

  public final class driveMotors {
    public static final int m_topLeftMotor = 0;
    public static final int m_bottomLeftMotor = 1;
    // placeholder values.
    public static final int m_topRightMotor = 2;
    public static final int m_bottomRightMotor = 3;

    public static final double distancePerPulse = 1024;
  }

  public final class armMotors {
    public static final int leftPull = 4;
    public static final int rightPull = 5;
  }

  public final class pneumaticSolenoids{
    public static final int solenoid1_F = 0;
    public static final int solenoid1_R = 1;
  }

  public final static class voltConstants {
    public static final double ksVolts = 0.96585;
    public static final double kvVolts = 5.7548;
    public static final double kaVolts = 3.2737;

    public static final double kpDrive = 9.4721;

    public static final double kTrackwidthMeters = 0.33779;
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);

    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;

    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
  }
  public static final double lengthInInches = 21.75;
}

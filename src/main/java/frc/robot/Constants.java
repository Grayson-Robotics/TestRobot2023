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
    
    public static final double gearRatio = 1; //change to 12.75 for velocity i believe unneeded for now

    public static final double distancePerPulse = (1f/1024f) * (gearRatio) * (0.1524 * Math.PI); // this is the distance traveled for every pulse of the encoder, calculated from 1/cpr (srx encoder is normally 4096 but since we have it in the 1x encoder mode it's divided by 4)* (diameter of wheel in preferred units * pi)
    public static final double distancePerPulse2 = 0.0000399940171;
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
    public static final double ksVolts = 0.87806;
    public static final double kvVolts = 5.5398;
    public static final double kaVolts = 2.5303;

    public static final double kpDrive = 8.9162;

    public static final double kTrackwidthMeters = 0.33779;
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);

    public static final double kMaxSpeedMetersPerSecond = 1;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;

    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
  }
  public static final double lengthInInches = 21.75;
}

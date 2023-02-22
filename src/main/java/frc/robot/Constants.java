// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
    public static final int m_topLeftMotor = 3;
    public static final int m_bottomLeftMotor = 2;
    // placeholder values.
    public static final int m_topRightMotor = 1;
    public static final int m_bottomRightMotor = 0;
  }

  public final class armMotors {
    public static final int leftPull = 4;
    public static final int rightPull = 5;
  }

  public final class pneumaticSolenoids{
    public static final int solenoid1_F = 0;
    public static final int solenoid1_R = 1;
  }
}

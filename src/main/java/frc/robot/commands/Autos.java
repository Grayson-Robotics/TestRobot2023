// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.voltConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Pneumatics;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static CommandBase exampleAuto(ExampleSubsystem subsystem) {
    return Commands.sequence(subsystem.exampleMethodCommand());
  }
  
  public static CommandBase autonomous(DriveTrain drive, Arm arm, Pneumatics pneumatics, String side){
    return Commands.sequence(new resetEncoders(drive),new AutoTest(drive, arm, pneumatics), new PIDauto(drive, side));
  }

  public static CommandBase autonomous2electricboogaloo(DriveTrain drive, Arm arm, Pneumatics pneumatics){
    return Commands.sequence(new resetEncoders(drive),new AutoTest(drive, arm, pneumatics), new PIDautoconversion(drive));
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

public static Command ramseteCommand(DriveTrain drive) {
  var autoVoltageConstraint =
  new DifferentialDriveVoltageConstraint(
      new SimpleMotorFeedforward(
          voltConstants.ksVolts,
          voltConstants.kvVolts,
          voltConstants.kaVolts),
      voltConstants.kDriveKinematics,
      10);

// Create config for trajectory
TrajectoryConfig config =
  new TrajectoryConfig(
          voltConstants.kMaxSpeedMetersPerSecond, 
          voltConstants.kMaxAccelerationMetersPerSecondSquared)
      // Add kinematics to ensure max speed is actually obeyed
      .setKinematics(voltConstants.kDriveKinematics)
      // Apply the voltage constraint
      .addConstraint(autoVoltageConstraint);

// An example trajectory to follow.  All units in meters.
Trajectory exampleTrajectory =
  TrajectoryGenerator.generateTrajectory(
      // Start at the origin facing the +X direction
      new Pose2d(0, 0, new Rotation2d(0)),
      // Pass through these two interior waypoints, making an 's' curve path
      List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
      // End 3 meters straight ahead of where we started, facing forward
      new Pose2d(3, 0, new Rotation2d(0)),
      // Pass config
      config);

drive.returnField2d().getObject("traj").setTrajectory(exampleTrajectory);

RamseteCommand ramseteCommand =
  new RamseteCommand(
      exampleTrajectory,
      drive::getPose,
      new RamseteController(voltConstants.kRamseteB, voltConstants.kRamseteZeta),
      new SimpleMotorFeedforward(
          voltConstants.ksVolts,
          voltConstants.kvVolts,
          voltConstants.kaVolts),
      voltConstants.kDriveKinematics,
      drive::getSpeeds,
      new PIDController(voltConstants.kpDrive, 0, 0),
      new PIDController(voltConstants.kpDrive, 0, 0),
      // RamseteCommand passes volts to the callback
      drive::tankDriveVolts,
      drive);

// Reset odometry to the starting pose of the trajectory.
drive.resetOdometry(exampleTrajectory.getInitialPose());

// Run path following command, then stop at the end.
return ramseteCommand.andThen(() -> drive.tankDriveVolts(0, 0));
}
}

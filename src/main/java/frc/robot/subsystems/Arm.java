// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Arm extends SubsystemBase {
  private final WPI_VictorSPX leftPull = new WPI_VictorSPX(Constants.armMotors.leftPull);
  private final WPI_VictorSPX rightPull = new WPI_VictorSPX(Constants.armMotors.rightPull);

  private final CANSparkMax inPull = new CANSparkMax(6, MotorType.kBrushed);

  private final MotorControllerGroup pullMotors = new MotorControllerGroup(leftPull, rightPull);

  private final double speed = 0.45;
  private final double upSpeed = 0.65;
  /** Creates a new Arm. */
  public Arm() {}

  public CommandBase raiseArm(){
    return runEnd(
      () -> {
        pullMotors.set(-1 * upSpeed);
      },
      () -> {
        pullMotors.set(0);
      }
      );
  }

  public CommandBase dropArm(){
    return runEnd(
      () -> {
        pullMotors.set(1 * speed);
      },
      () -> {
        pullMotors.set(0);
      }
      );
  }

  public CommandBase pullArmIn(){
    return runEnd(
      () -> {
        inPull.set(1 * speed);
      },
      () -> {
        inPull.set(0);
      }
      );
  }

  public CommandBase pushArmOut(){
    return runEnd(
      () -> {
        inPull.set(-1 * speed);
      },
      () -> {
        inPull.set(0);
      }
      );
  }

  public void dropArmManual(){
    pullMotors.set(1 * speed);
  }

  public void raiseArmManual(){
    pullMotors.set(-1 * speed);
  }

  public void stopArm(){
    pullMotors.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

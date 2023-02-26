// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pneumatics extends SubsystemBase {
  private final DoubleSolenoid solenoid1 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.pneumaticSolenoids.solenoid1_F, Constants.pneumaticSolenoids.solenoid1_R);
  private Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);
  private ShuffleboardTab main = Shuffleboard.getTab("Main Data");

  /** Creates a new Pneumatics. */
  public Pneumatics() {
    main.add(compressor);
    main.add("Compressor Pressure", 0);
    main.add(solenoid1);

    
    
    solenoid1.set(Value.kOff);
  }

  public void toggle(){
    solenoid1.toggle();
  }

  public void setForward(){
    solenoid1.set(Value.kForward);
  }

  public void setReverse(){
    solenoid1.set(Value.kReverse);
  }

  public void setOff(){
    solenoid1.set(Value.kOff);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}

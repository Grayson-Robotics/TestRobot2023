// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.PIDauto;
import frc.robot.commands.driveStraightPID;
import frc.robot.commands.movePneumaticArm;
import frc.robot.commands.resetEncoders;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Pneumatics;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveTrain driveTrain = new DriveTrain();
  private final Pneumatics pneumatics = new Pneumatics();
  private final Arm arm = new Arm();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
  new CommandXboxController(OperatorConstants.kDriverControllerPort);
  
  private final CommandXboxController m_driverController2 =
   new CommandXboxController(OperatorConstants.kDriverControllerPort2);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    
    Shuffleboard.getTab("Main Data").add("reset", new resetEncoders(driveTrain));
    Shuffleboard.getTab("Main Data").add("armUP", arm.raiseArm());
    Shuffleboard.getTab("Main Data").add("armDown", arm.dropArm());

    driveTrain.setDefaultCommand(new DriveCommand(driveTrain,
    () -> m_driverController.getLeftY(),
    () -> m_driverController.getRightX()));
  }
  
  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    //new Trigger(m_exampleSubsystem::exampleCondition)
    //    .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    //m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    // m_driverController.a().toggleOnTrue(new movePneumaticArm(pneumatics));
    // m_driverController.pov(0).whileTrue(arm.raiseArm());
    // m_driverController.pov(180).whileTrue(arm.dropArm());
    // m_driverController.leftTrigger().whileTrue(arm.pushArmOut());
    // m_driverController.rightTrigger().whileTrue(arm.pullArmIn());

    //controls for the second controller
    /*
    m_driverController.a().toggleOnTrue(new movePneumaticArm(pneumatics));
    m_driverController.pov(0).whileTrue(arm.raiseArm());
    m_driverController.pov(180).whileTrue(arm.dropArm());
    m_driverController.leftTrigger().whileTrue(arm.pushArmOut());
    m_driverController.rightTrigger().whileTrue(arm.pullArmIn());
    */

    m_driverController.rightBumper().whileTrue(new resetEncoders(driveTrain));

    m_driverController2.a().toggleOnTrue(new movePneumaticArm(pneumatics));
    m_driverController2.pov(0).whileTrue(arm.raiseArm());
    m_driverController2.pov(180).whileTrue(arm.dropArm());
    m_driverController2.leftTrigger().whileTrue(arm.pushArmOut());
    m_driverController2.rightTrigger().whileTrue(arm.pullArmIn());

    m_driverController.leftBumper().whileTrue(new driveStraightPID(driveTrain, () -> m_driverController.getLeftY()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new PIDauto(driveTrain); //Autos.exampleAuto(m_exampleSubsystem);
  }

  /*
  public CommandXboxController whichDrivesY(CommandXboxController controller1, CommandXboxController controller2){
    CommandXboxController controller = controller1;

    if(Math.abs(controller2.getLeftY()) > Math.abs(controller1.getLeftY())){
      controller = controller2;
    }

    return controller;
  }

  public CommandXboxController whichDrivesX(CommandXboxController controller1, CommandXboxController controller2){
    CommandXboxController controller = controller1;

    if(Math.abs(controller2.getRightX()) > Math.abs(controller1.getLeftY())){
      controller = controller2;
    }

    return controller;
  }
  */
}

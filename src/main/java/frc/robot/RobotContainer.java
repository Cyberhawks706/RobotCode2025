// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveConstants;
import lib.frc706.cyberlib.subsystems.SwerveSubsystem;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
  */
  public class RobotContainer {

  private final SendableChooser<Command> autoChooser;

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem swerveSubsystem;
    

    private Command teleopCommand;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverController;
  private final CommandXboxController manipulatorController;
    // The robot's subsystems and commands are defined here...

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController m_driverController =
        new CommandXboxController(OperatorConstants.kDriverControllerPort);
  
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
          File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");

    swerveSubsystem = new SwerveSubsystem(swerveJsonDirectory, OperatorConstants.kMaxVelTele, SwerveConstants.pathFollowerConfig);                                                                                                                                            //poopy-Bryce
      // Configure the trigger bindings
      configureBindings();
    
  


    /** Set controller variables */
    if (DriverStation.isJoystickConnected(OperatorConstants.kDriverControllerPortBT)) {
      driverController = new CommandXboxController(OperatorConstants.kDriverControllerPortBT);
    } else {
      driverController = new CommandXboxController(OperatorConstants.kDriverControllerPortUSB);
    }
    if (DriverStation.isJoystickConnected(OperatorConstants.kManipulatorControllerPortBT)) {
      manipulatorController = new CommandXboxController(OperatorConstants.kManipulatorControllerPortBT);
    } else {
      manipulatorController = new CommandXboxController(OperatorConstants.kManipulatorControllerPortUSB);
    }
  
    swerveSubsystem.setDefaultCommand(getTeleopCommand());
    // Configure the trigger bindings
    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Do all other initialization
    configureBindings();

  }

  /**
   * Use this method to define your trigger->command . Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSuppmappingslier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
  }

  public Command getTeleopCommand() {
    swerveSubsystem.swerveDrive.setHeadingCorrection(false);
    return teleopCommand;
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */





  






  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}




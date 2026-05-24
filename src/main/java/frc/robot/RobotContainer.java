// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Drivetrain;

public class RobotContainer {
  public static CommandXboxController controllerDriver = new CommandXboxController(0);
  public static CommandXboxController controllerOper = new CommandXboxController(1);

  private final SendableChooser<Command> autoChooser;

  public static Drivetrain drivetrain = Drivetrain.getInstance();


  public RobotContainer() {
    // drivetrain.setDefaultCommand(
    // //   new FieldDrive(        
    // //     () -> -controllerDriver.getLeftY(), 
    // //     () -> -controllerDriver.getLeftX(), 
    // //     () -> -controllerDriver.getRightX(),
    // //     false
    // //   )
    // );


    configureBindings();



    
    autoChooser = new SendableChooser<Command>();
    for(String name : AutoBuilder.getAllAutoNames()) {
      
      autoChooser.addOption(name, new PathPlannerAuto(name));
      autoChooser.addOption(name + "Mirrored", new PathPlannerAuto(name, true));

  }
      SmartDashboard.putData("Auto Chooser", autoChooser);

  }


  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}

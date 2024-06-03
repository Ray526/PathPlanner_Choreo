// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.fasterxml.jackson.core.sym.Name;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.robotConstants;
import frc.robot.commands.DoNothing;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Swerve;

public class RobotContainer {

  // private final Swerve m_swerve = new Swerve();

  private final XboxController controller = new XboxController(robotConstants.DriverControllerID);

  // private final TeleopSwerve teleopSwerve = new TeleopSwerve(m_swerve, controller);

  private final SendableChooser<Command> autoChooser;

  private final Command DoNothing = new DoNothing();

  public RobotContainer() {

    NamedCommands.registerCommand("SPEAKER", DoNothing);
    NamedCommands.registerCommand("GROUND", DoNothing);
    NamedCommands.registerCommand("SHOOT", DoNothing);
    NamedCommands.registerCommand("AutoAim", DoNothing);
    NamedCommands.registerCommand("AutoUpper", DoNothing);
    
    configureBindings();

    // m_swerve.setDefaultCommand(teleopSwerve);

    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Mode", autoChooser);
  }

  private void configureBindings() {
    SmartDashboard.putData("Test-Choreo-F1", new PathPlannerAuto("Test-Choreo-F1"));
    SmartDashboard.putData("Test-PP-F1", new PathPlannerAuto("Test-PP-F1"));
    SmartDashboard.putData("PP-F1-T90", new PathPlannerAuto("PP-F1-T90"));

    SmartDashboard.putData("Move (1,0)", AutoBuilder.pathfindToPose(
      new Pose2d(-.0, 0, Rotation2d.fromDegrees(0)),
      new PathConstraints(
        Constants.SwerveConstants.maxModuleSpeed,
        Constants.SwerveConstants.maxModuleAccleration,
        Constants.SwerveConstants.maxAngularVelocity,
        Constants.SwerveConstants.maxAngularAccleration
      )
    ));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
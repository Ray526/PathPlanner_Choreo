// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.robotConstants;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Swerve;

public class RobotContainer {

  private final XboxController controller = new XboxController(robotConstants.DriverControllerID);

  private final Swerve m_swerve = new Swerve();
  private final TeleopSwerve teleopSwerve = new TeleopSwerve(m_swerve, controller);

  private final SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser();

  public RobotContainer() {
    
    configureBindings();

    m_swerve.setDefaultCommand(teleopSwerve);

    SmartDashboard.putData("Auto Mode", autoChooser);
  }

  private void configureBindings() {
    SmartDashboard.putData("Test-Choreo-F1", new PathPlannerAuto("Test-Choreo-F1"));
    SmartDashboard.putData("NiggaPath", new PathPlannerAuto("NiggaPath"));
    SmartDashboard.putData("Test-PP-F1", new PathPlannerAuto("Test-PP-F1"));
    SmartDashboard.putData("Test-PP-F1-R", new PathPlannerAuto("Test-PP-F1-R"));

    SmartDashboard.putData("Move (2,0)", AutoBuilder.pathfindToPose(
      new Pose2d(-2.0, 0, Rotation2d.fromDegrees(0)),
      new PathConstraints(
        Constants.SwerveConstants.maxModuleSpeed,
        Constants.SwerveConstants.maxModuleAccleration,
        Constants.SwerveConstants.maxAngularVelocity,
        Constants.SwerveConstants.maxAngularAccleration
      )
    ));

    SmartDashboard.putData("Move ft 1", Commands.runOnce(() -> {
      Pose2d currentPose = m_swerve.getPose();
      List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
        new Pose2d(currentPose.getTranslation(), new Rotation2d()),
        new Pose2d(currentPose.getTranslation().plus(new Translation2d(1.0, 0.0)), new Rotation2d())
      );

      PathPlannerPath path = new PathPlannerPath(
        bezierPoints, 
        new PathConstraints(
          Constants.SwerveConstants.maxModuleSpeed,
          Constants.SwerveConstants.maxModuleAccleration,
          Constants.SwerveConstants.maxAngularVelocity,
          Constants.SwerveConstants.maxAngularAccleration
        ),  
        new GoalEndState(0.0, currentPose.getRotation())
      );

      path.preventFlipping = true;
      AutoBuilder.followPath(path).schedule();
    }));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
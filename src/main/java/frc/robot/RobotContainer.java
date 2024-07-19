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
import frc.robot.Constants.robotConstants;
import frc.robot.commands.AimbotSwerve;
import frc.robot.commands.SHOOT;
import frc.robot.commands.BASE;
import frc.robot.commands.GROUND;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.TeleopUpper;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.UpperSub;
import frc.robot.subsystems.VisionSub;

public class RobotContainer {

  private final Swerve m_Swerve = new Swerve();
  private final UpperSub m_Upper = new UpperSub();
  private final VisionSub m_Vision = new VisionSub();

  private final XboxController driverController = new XboxController(robotConstants.DriverControllerID);

  private final TeleopSwerve teleopSwerve = new TeleopSwerve(m_Swerve, driverController);
  private final TeleopUpper teleopUpper = new TeleopUpper(m_Upper, driverController);
  private final AimbotSwerve aimbotSwerve = new AimbotSwerve(m_Swerve, m_Vision, driverController);

  public final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    NamedCommands.registerCommand("GROUND", new GROUND(m_Upper));
    NamedCommands.registerCommand("BASE",new BASE(m_Upper));
    NamedCommands.registerCommand("SHOOT", new SHOOT(m_Swerve, m_Upper, m_Vision));
    NamedCommands.registerCommand("AIM", null);
    NamedCommands.registerCommand("SPEAKER", null);
    
    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser();
    
    SmartDashboard.putData("Autos", autoChooser);
    
    m_Swerve.setDefaultCommand(teleopSwerve);
    m_Upper.setDefaultCommand(teleopUpper);
  }

  private void configureBindings() {
    // SmartDashboard.putData("Test-Choreo-F1", new PathPlannerAuto("Test-Choreo-F1"));
    // SmartDashboard.putData("Test-PP-S", new PathPlannerAuto("Test-PP-S"));
    // SmartDashboard.putData("PP-F1", new PathPlannerAuto("PP-F1"));
    // SmartDashboard.putData("S-curve", new PathPlannerAuto("S-curve"));
    // SmartDashboard.putData("S-curve-2", new PathPlannerAuto("S-curve-2"));
    // SmartDashboard.putData("X1", new PathPlannerAuto("X1"));
    // SmartDashboard.putData("X1-X2-X3", new PathPlannerAuto("X1-X2-X3"));
    SmartDashboard.putData("M-X2", new PathPlannerAuto("M-X2"));
    SmartDashboard.putData("Aiming", new PathPlannerAuto("Aiming"));

    // SmartDashboard.putData("Move (1,0)", AutoBuilder.pathfindToPose(
    //   new Pose2d(-.0, 0, Rotation2d.fromDegrees(0)),
    //   new PathConstraints(
    //     Constants.SwerveConstants.maxModuleSpeed,
    //     Constants.SwerveConstants.maxModuleAccleration,
    //     Constants.SwerveConstants.maxAngularVelocity,
    //     Constants.SwerveConstants.maxAngularAccleration
    //   )
    // ));


    //     SmartDashboard.putData("On-the-fly path", Commands.runOnce(() -> {
    //   Pose2d currentPose = m_Swerve.getPose();
      
    //   // The rotation component in these poses represents the direction of travel
    //   Pose2d startPos = new Pose2d(currentPose.getTranslation(), new Rotation2d());
    //   Pose2d targetPos = new Pose2d(currentPose.getTranslation().plus(new Translation2d(2.0, 0.0)), new Rotation2d());

    //   List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(startPos, targetPos);
    //   PathPlannerPath path = new PathPlannerPath(
    //     bezierPoints, 
    //     new PathConstraints(
    //       4.0, 4.0, 
    //       Units.degreesToRadians(360), Units.degreesToRadians(540)
    //     ),  
    //     new GoalEndState(0.0, currentPose.getRotation())
    //   );

    //   // Prevent this path from being flipped on the red alliance, since the given positions are already correct
    //   path.preventFlipping = true;

    //   AutoBuilder.followPath(path).schedule();
    // }));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}

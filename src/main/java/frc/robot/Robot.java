// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import com.choreo.lib.Choreo;
// import com.choreo.lib.ChoreoTrajectory;
// import java.util.HashMap;
// import java.util.Map;

import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
// import frc.robot.Constants.UpperState;
import frc.robot.Constants.robotConstants;
// import frc.robot.commands.AutoUpper;
import frc.robot.commands.TeleopSwerve;
// import frc.robot.commands.TeleopUpper;
import frc.robot.subsystems.Swerve;
// import frc.robot.subsystems.UpperSub;
// import frc.robot.subsystems.VisionSub;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final Swerve m_Swerve = new Swerve();
  // private final UpperSub m_upper = new UpperSub();
  // private final VisionSub m_vision = new VisionSub();

  private final XboxController driverController = new XboxController(robotConstants.DriverControllerID);

  private final TeleopSwerve teleopSwerve = new TeleopSwerve(m_Swerve, driverController);
  // private final TeleopUpper teleopUpper = new TeleopUpper(m_upper, driverController);

  public static String alliance;

  SendableChooser<String> m_Alliance = new SendableChooser<>();
  SendableChooser<Command> m_PathPlanner = new SendableChooser<>();

  @Override
  public void robotInit() {

    // ChoreoTrajectory traj;

    // Field2d m_field = new Field2d();

    // traj = Choreo.getTrajectory("Trajectory");

    // m_field.getObject("traj").setPoses(
    //   traj.getInitialPose(), traj.getFinalPose()
    // );
    // m_field.getObject("trajPoses").setPoses(
    //   traj.getPoses()
    // );

    // SmartDashboard.putData(m_field);
    
    configurePathPlannerCommand();

    m_PathPlanner = AutoBuilder.buildAutoChooser("PathPlanner");
    
    m_Alliance.setDefaultOption("RED", "RED");
    m_Alliance.addOption("BLUE", "BLUE");
    m_Alliance.addOption("PATHPLANNER", "PATHPLANNER");

    m_PathPlanner.setDefaultOption("", m_autonomousCommand);

    SmartDashboard.putData("Alliance Team", m_Alliance);
    SmartDashboard.putData("PATHPLANNER", m_PathPlanner);
  }

  public void configurePathPlannerCommand() {
    // NamedCommands.registerCommand("GROUND", new AutoUpper(m_upper, UpperState.GROUND));
    // NamedCommands.registerCommand("SPEAKER", new AutoUpper(m_upper, UpperState.BASE));
    // NamedCommands.registerCommand("SHOOT", new AutoUpper(m_upper, UpperState.SHOOT));
  }

  @Override
  public void robotPeriodic() {
    alliance = m_Alliance.getSelected();
    // X = m_X.getSelected();
    // Y = m_Y.getSelected();
    // Z = m_Z.getSelected();

    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    robotConstants.mode = "DISABLED";
  }

  @Override
  public void disabledPeriodic() {
    // m_upper.charge(255, 0, 0, (driverController.getLeftY() + driverController.getRightY()));
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    robotConstants.mode = "AUTO";
    switch(alliance){
      case "RED":
        System.out.println("Not This ONE!!!");
      case "BLUE":
        System.out.println("Not This ONE!!!");
      case "PATHPLANNER":
        m_autonomousCommand = m_PathPlanner.getSelected();
    } 

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    } else {
      System.out.println("U don't choose the autos!!!");
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    robotConstants.mode = "TELE";
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_Swerve.setDefaultCommand(teleopSwerve);
    // m_upper.setDefaultCommand(teleopUpper);
    // m_vision.setDefaultCommand(null);
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
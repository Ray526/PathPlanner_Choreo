// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.UpperState;
import frc.robot.Constants.robotConstants;
import frc.robot.commands.AutoUpper;
import frc.robot.commands.Autos;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.TeleopUpper;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.UpperSub;
import frc.robot.subsystems.VisionSub;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final Swerve m_Swerve = new Swerve();
  private final UpperSub m_upper = new UpperSub();
  private final VisionSub m_vision = new VisionSub();

  private final XboxController driverController = new XboxController(robotConstants.DriverControllerID);

  private final TeleopSwerve teleopSwerve = new TeleopSwerve(m_Swerve, m_vision, driverController);
  private final TeleopUpper teleopUpper = new TeleopUpper(m_upper, driverController);

  public static String alliance;
  private String command;

  SendableChooser<String> m_Alliance = new SendableChooser<>();
  SendableChooser<String> m_AutoCommand = new SendableChooser<>();
  SendableChooser<Command> m_PathPlanner = new SendableChooser<>();
  SendableChooser<String> m_ChoreoChooser = new SendableChooser<>();
  Map<String, ChoreoTrajectory> m_ChoreoTrajectories = new HashMap<>();

  @Override
  public void robotInit() {

    ChoreoTrajectory traj;

    Field2d m_field = new Field2d();

    traj = Choreo.getTrajectory("Trajectory");

    m_field.getObject("traj").setPoses(
      traj.getInitialPose(), traj.getFinalPose()
    );
    m_field.getObject("trajPoses").setPoses(
      traj.getPoses()
    );

    SmartDashboard.putData(m_field);
    
    configurePathPlannerCommand();
    
    m_Alliance.setDefaultOption("RED", "RED");
    m_Alliance.addOption("BLUE", "BLUE");
    m_Alliance.addOption("PATHPLANNER", "PATHPLANNER");
    m_Alliance.addOption("CHOREO", "CHOREO");

    m_PathPlanner = AutoBuilder.buildAutoChooser("PathPlanner");

    m_PathPlanner.setDefaultOption("", m_autonomousCommand);

    // left=L mid=M right=R base=B red=r blue=b   combination -> initial pos(L,M,B)+base or not(B or N)+preload and note place(PXaYb,(a=1,2,3... , b=1,2,3...))+how many notes(xn, x=1, 2, 3...)+red or blue(r,b)
    // 1 note
    m_AutoCommand.setDefaultOption("LBP1nrb", "LBP1nrb");
    m_AutoCommand.addOption("MBP1nrb", "MBP1nrb");
    m_AutoCommand.addOption("RBP1nrb", "RBP1nrb");

    // 2 note
    m_AutoCommand.addOption("MBPX12n", "MBPX12n");
    m_AutoCommand.addOption("MBPX22n", "MBPX22n");
    m_AutoCommand.addOption("MBPX32n", "MBPX32n");

    // 3 note
    m_AutoCommand.addOption("MBPX2X13n", "MBPX2X13n");
    m_AutoCommand.addOption("MBPX2X33n", "MBPX2X33n");

    // 4 note
    m_AutoCommand.addOption("MBPX2X1X34n", "MBPX2X1X34n");

    // Choreo
    // 添加ChoreoTrajectory到m_ChoreoTrajectories映射中
    m_ChoreoTrajectories.put("F1B", Choreo.getTrajectory("F1"));
    m_ChoreoTrajectories.put("F1B", Choreo.getTrajectory("F1B"));

    // 將ChoreoTrajectory的名稱添加到m_ChoreoChooser中
    for (String name : m_ChoreoTrajectories.keySet()) {
      m_ChoreoChooser.addOption(name, name);
    }

    SmartDashboard.putData("Alliance Team", m_Alliance);
    SmartDashboard.putData("Auto Path", m_AutoCommand);
    SmartDashboard.putData("PATHPLANNER", m_PathPlanner);
    SmartDashboard.putData("Choreo Trajectories", m_ChoreoChooser);
  }

  public void configurePathPlannerCommand() {
    NamedCommands.registerCommand("GROUND", new AutoUpper(m_upper, UpperState.GROUND));
    NamedCommands.registerCommand("SPEAKER", new AutoUpper(m_upper, UpperState.BASE));
    NamedCommands.registerCommand("SHOOT", new AutoUpper(m_upper, UpperState.SHOOT));
  }

  @Override
  public void robotPeriodic() {
    alliance = m_Alliance.getSelected();
    command = m_AutoCommand.getSelected();
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
    m_upper.charge(255, 0, 0, (driverController.getLeftY() + driverController.getRightY()));
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    robotConstants.mode = "AUTO";
    switch(alliance){
      case "RED":
        switch(command){
          // 1 note
          case "LBP1nrb":
            m_autonomousCommand = new Autos().Left_1n(m_Swerve, m_upper);
            break;
          case "MBP1nrb":
            m_autonomousCommand = new Autos().Mid_1n(m_Swerve, m_upper);
            break;
          case "RBP1nrb":
            m_autonomousCommand = new Autos().Right_1n(m_Swerve, m_upper);
            break;

          // 2 notes
          case "MBPX12n":
            m_autonomousCommand = new Autos().MidX1Base_2n(m_Swerve, m_upper);
            break;
          case "MBPX22n":
            m_autonomousCommand = new Autos().MidX2Base_2n(m_Swerve, m_upper);
            break;
          case "MBPX32n":
            m_autonomousCommand = new Autos().MidX3Base_2n(m_Swerve, m_upper);
            break;

          // 3 note
          case "MBPX2X13n":
            m_autonomousCommand = new Autos().MidX2BaseX1Base_3n(m_Swerve, m_upper);
            break;
          case "MBPX2X33n":
            m_autonomousCommand = new Autos().MidX2BaseX3Base_3n(m_Swerve, m_upper);
            break;

          // 4 note
          case "MBPX2X1X34n":
            m_autonomousCommand = new Autos().MidX2BaseX1BaseX3Base_4n(m_Swerve, m_upper);
            break;

          default:
            m_autonomousCommand = null;
            break;
        }
        break;
      case "BLUE":
        switch(command){
          // 1 note
          case "LBP1nrb":
            m_autonomousCommand = new Autos().Left_1n(m_Swerve, m_upper);
            break;
          case "MBP1nrb":
            m_autonomousCommand = new Autos().Mid_1n(m_Swerve, m_upper);
            break;
          case "RBP1nrb":
            m_autonomousCommand = new Autos().Right_1n(m_Swerve, m_upper);
            break;

          // 2 note
          case "MBPX12n":
            m_autonomousCommand = new Autos().MidX1Base_2n(m_Swerve, m_upper);
            break;
          case "MBPX22n":
            m_autonomousCommand = new Autos().MidX2Base_2n(m_Swerve, m_upper);
            break;
          case "MBPX32n":
            m_autonomousCommand = new Autos().MidX3Base_2n(m_Swerve, m_upper);
            break;    

          // 3 note
          case "MBPX2X13n":
            m_autonomousCommand = new Autos().MidX2BaseX1Base_3n(m_Swerve, m_upper);
            break;
          case "MBPX2X33n":
            m_autonomousCommand = new Autos().MidX2BaseX3Base_3n(m_Swerve, m_upper);
            break;

          // 4 note
          case "MBPX2X1X34n":
            m_autonomousCommand = new Autos().MidX2BaseX1BaseX3Base_4n(m_Swerve, m_upper);
            break;
          
          default:
            m_autonomousCommand = null;
            break;
        }
        break;
      case "PATHPLANNER":
        m_autonomousCommand = m_PathPlanner.getSelected();

      case "CHOREO":
        String selectedTrajectoryName = m_ChoreoChooser.getSelected();
        ChoreoTrajectory selectedTrajectory = m_ChoreoTrajectories.get(selectedTrajectoryName);
        // 使用選擇的ChoreoTrajectory對象執行自動模式邏輯
    } 

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
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
    m_upper.setDefaultCommand(teleopUpper);
    m_vision.setDefaultCommand(null);
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
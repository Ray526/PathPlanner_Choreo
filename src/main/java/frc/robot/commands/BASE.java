// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.ChenryLib.PID;
import frc.robot.Constants.UpperConstants;
import frc.robot.subsystems.UpperSub;

public class BASE extends Command {
  private final UpperSub s_Upper;

  private double elbowAngle;
  private double intakeSpeed;
  private double shooterSpeed;

  private boolean m_CancelCommand;
  private boolean shot;

  private final PID elbowPID = new PID(
      UpperConstants.elbowKP,
      UpperConstants.elbowKI,
      UpperConstants.elbowKD,
      UpperConstants.elbowiWindup,
      UpperConstants.elbowiLimit
  );
  public BASE(UpperSub upper) {
    this.s_Upper = upper;
    addRequirements(upper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_CancelCommand = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elbowAngle = UpperConstants.ELBOW_BASE_POS;
    intakeSpeed = 0;
    shooterSpeed = UpperConstants.SHOOTER_SHOOT_SPEED;
    if(Math.abs(s_Upper.getShooterRPM()) > UpperConstants.SHOOTER_LEGAL_SPEED) {
      s_Upper.setLED(0,255,0);
      intakeSpeed = 1;
      Timer.delay(2);
      shot = true;
    }
    else s_Upper.charge(255,0,0, false);

    if (shot) {
      m_CancelCommand = true;
    }

    s_Upper.setElbow(-elbowPID.calculate(elbowAngle - s_Upper.getElbowRotation()));
    s_Upper.setShooter(shooterSpeed);
    s_Upper.setIntake(intakeSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_CancelCommand;
  }
}

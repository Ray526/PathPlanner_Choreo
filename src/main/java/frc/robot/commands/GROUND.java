// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.ChenryLib.PID;
import frc.robot.Constants.UpperConstants;
import frc.robot.subsystems.UpperSub;

public class GROUND extends Command {
  private final UpperSub s_Upper;

  private double elbowAngle;
  private double intakeSpeed;
  private double shooterSpeed;

  private final PID elbowPID = new PID(
      UpperConstants.elbowKP,
      UpperConstants.elbowKI,
      UpperConstants.elbowKD,
      UpperConstants.elbowiWindup,
      UpperConstants.elbowiLimit
  );
  public GROUND(UpperSub upper) {
    this.s_Upper = upper;
    addRequirements(upper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elbowAngle = UpperConstants.ELBOW_GROUND_POS;
    intakeSpeed = s_Upper.hasNote() ? 0: UpperConstants.INTAKE_GROUND_SPEED;
    shooterSpeed = UpperConstants.SHOOTER_GROUND_SPEED;
    if(s_Upper.hasNote()) s_Upper.setLED(12,41,235);
    else s_Upper.blink(12,41,235);

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
    if (s_Upper.hasNote()) {
      return true;
    } else {
      return false;
    }
  }
}

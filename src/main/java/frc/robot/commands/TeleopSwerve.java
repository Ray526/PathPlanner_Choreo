package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
// import frc.robot.subsystems.VisionSub;

public class TeleopSwerve extends Command {
  private Swerve s_Swerve;
  // private VisionSub s_Vision;
  private XboxController driver;

  private SlewRateLimiter translationLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter strafeLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter rotationLimiter = new SlewRateLimiter(3.0);

  private double translationVal;
  private double strafeVal;
  private double rotationVal;

  public TeleopSwerve(Swerve s_Swerve, XboxController controller) {
    this.s_Swerve = s_Swerve;
    // this.s_Vision = s_Vision;
    this.driver = controller;
    addRequirements(s_Swerve);
  }

  @Override
  public void execute() {
    /* Get Values, Deadband */
    translationVal = translationLimiter.calculate(
        MathUtil.applyDeadband(driver.getLeftY(), Constants.SwerveConstants.axisDeadBand));
    strafeVal = strafeLimiter.calculate(
        MathUtil.applyDeadband(driver.getLeftX(), Constants.SwerveConstants.axisDeadBand));
    rotationVal = 
      rotationLimiter
            .calculate(MathUtil.applyDeadband(driver.getRightX() * 0.5, Constants.SwerveConstants.axisDeadBand));

    if (driver.getBackButton()) {
      s_Swerve.zeroGyro(); 
      s_Swerve.resetPose(new Pose2d(new Translation2d(0, 0), new Rotation2d(0)));
    }

    /* Drive */
    s_Swerve.drive(
        new Translation2d(translationVal, strafeVal).times(Constants.SwerveConstants.maxModuleSpeed),
        rotationVal * Constants.SwerveConstants.maxAngularVelocity, true,
        true);
  }

  @Override
  public void end(boolean interrupted){
    System.out.println("bro ur teleop swerve is fucked");
  }

  @Override
  public boolean isFinished(){
    return false;
  }
}
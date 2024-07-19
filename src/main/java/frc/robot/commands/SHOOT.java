package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.ChenryLib.PID;
import frc.lib.math.PolynomialRegression;
import frc.robot.Constants;
import frc.robot.Constants.LimeLight;
import frc.robot.Constants.UpperConstants;
import frc.robot.Constants.UpperState;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.UpperSub;
import frc.robot.subsystems.VisionSub;

public class SHOOT extends Command {
  /* Chassis */
  private Swerve s_Swerve;
  private VisionSub s_Vision;

  private SlewRateLimiter translationLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter strafeLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter rotationLimiter = new SlewRateLimiter(3.0);

  private double translationVal;
  private double strafeVal;
  private double rotationVal;
  private double ema;

  public double KP = Constants.LimeLight.KPDefault;
  public double KI = Constants.LimeLight.KIDefault;
  public double KD = Constants.LimeLight.KDDefault;
  public double WindUp = Constants.LimeLight.WindupDefault;
  public double Limit = Constants.LimeLight.LimitDefault;
  public double Smooth = Constants.LimeLight.SmoothDefault;

  private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  private NetworkTableEntry tx = table.getEntry("tx");
  private NetworkTableEntry ty = table.getEntry("ty");
  private NetworkTableEntry tpcs = table.getEntry("targetpose_cameraspace");
  
  private PID facingPID = new PID(KP, KI, KD, WindUp, Limit);
  
  /* Upper */
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

  /* Timer */
  private Timer timer = new Timer();

  public SHOOT(Swerve s_Swerve, UpperSub s_Upper, VisionSub s_Vision) {
    this.s_Swerve = s_Swerve;
    this.s_Vision = s_Vision;
    this.s_Upper = s_Upper;
    addRequirements(s_Upper);
    addRequirements(s_Swerve);
    addRequirements(s_Vision);
  }

  @Override
  public void initialize() {
    Constants.state = UpperState.DEFAULT;
    elbowAngle = UpperConstants.ELBOW_DEFAULT_POS;
    timer.reset();
  }

  @Override
  public void execute() {
    Preferences.initDouble(LimeLight.KPKey, KP);
    Preferences.initDouble(LimeLight.KIKey, KI);
    Preferences.initDouble(LimeLight.KDKey, KD);
    Preferences.initDouble(LimeLight.WindupKey, WindUp);
    Preferences.initDouble(LimeLight.LimKey, Limit);
    Preferences.initDouble(LimeLight.SmoothKey, Smooth);

    KP = Preferences.getDouble(LimeLight.KPKey, KP);
    KI = Preferences.getDouble(LimeLight.KIKey, KI);
    KD = Preferences.getDouble(LimeLight.KDKey, KD);
    WindUp = Preferences.getDouble(LimeLight.WindupKey, WindUp);
    Limit = Preferences.getDouble(LimeLight.LimKey, Limit);
    Smooth = Preferences.getDouble(LimeLight.SmoothKey, Smooth);

    SmartDashboard.putNumber("tx", tx.getDouble(0.0));
    SmartDashboard.putNumber("ty", ty.getDouble(0.0));
    SmartDashboard.putNumber("tz", tpcs.getDoubleArray(new double[6])[2]);
    SmartDashboard.putNumber("speed", rotationVal);

    ema = Smooth*facingPID.calculate(tx.getDouble(0.0))+(1-Smooth)*ema;
    rotationVal = ema;

    /* Drive */
    s_Swerve.drive(
        new Translation2d(translationVal, strafeVal).times(Constants.SwerveConstants.maxModuleSpeed),
        rotationVal * Constants.SwerveConstants.maxAngularVelocity, true,
        true);

    /* Upper */
    double tzInput = tx.getDouble(0.0);
    double degOutput = PolynomialRegression.predictDeg(tzInput);
    shooterSpeed = 0.7;

    s_Upper.setElbow(-elbowPID.calculate(degOutput - s_Upper.getElbowRotation()));
    s_Upper.setShooter(shooterSpeed);

    /* Timer */
    timer.restart();
  }

  @Override
  public void end(boolean interrupted){}

  @Override
  public boolean isFinished() {
    if (timer.get() == 2) {
    return false;
    } else {
      return false;
    }
  }
}
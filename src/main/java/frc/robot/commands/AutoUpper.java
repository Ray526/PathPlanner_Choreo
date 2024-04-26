// package frc.robot.commands;

// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.ChenryLib.PID;
// import frc.robot.Constants.UpperConstants;
// import frc.robot.Constants.UpperState;
// import frc.robot.subsystems.UpperSub;

// public class AutoUpper extends Command{
//     private final UpperSub s_Upper;

//     private UpperState state;
    
//     private double elbowAngle;
//     private double intakeSpeed;
//     private double shooterSpeed;

//     private double time, lastTime;
//     private boolean loadCheck;
//     private boolean loaded;

//     private final PID elbowPID = new PID(
//         UpperConstants.elbowKP, 
//         UpperConstants.elbowKI,
//         UpperConstants.elbowKD,
//         UpperConstants.elbowiWindup,
//         UpperConstants.elbowiLimit
//     );

//     public AutoUpper(UpperSub s_Upper, UpperState state) {
//         this.s_Upper = s_Upper;
//         this.state = state;
//         addRequirements(s_Upper);
//     }

//     public AutoUpper(UpperSub s_Upper, double elbowAngle, double shooterSpeed, double intakeSpeed, double time, boolean loadCheck) {
//         this.s_Upper = s_Upper;
//         this.elbowAngle = elbowAngle;
//         this.shooterSpeed = shooterSpeed;
//         this.intakeSpeed = intakeSpeed;
//         this.time = time;
//         this.loadCheck = loadCheck;
//     }

//     @Override
//     public void initialize() {
//         lastTime = Timer.getFPGATimestamp();
//         if(loadCheck) loaded = s_Upper.hasNote();
//     }

//     @Override
//     public void execute() {
//         if(state != null) {
//             switch (state) {
//                 case GROUND:
//                     elbowAngle = UpperConstants.ELBOW_GROUND_POS;
//                     intakeSpeed = s_Upper.hasNote() ? 0: UpperConstants.INTAKE_GROUND_SPEED;
//                     shooterSpeed = UpperConstants.SHOOTER_GROUND_SPEED;
//                     if(s_Upper.hasNote()) s_Upper.setLED(12,41,235);
//                     else s_Upper.blink(12,41,235);
//                     if(s_Upper.hasNote()) state = UpperState.DEFAULT;
//                     break;
//                 case BASE:
//                     elbowAngle = UpperConstants.ELBOW_BASE_POS;
//                     intakeSpeed = 0;
//                     shooterSpeed = UpperConstants.SHOOTER_SHOOT_SPEED;
//                     if(Math.abs(s_Upper.getShooterRPM()) > 5000) s_Upper.setLED(0,255,0);
//                     else s_Upper.charge(255,0,0, false);
//                     break;
//                 case SHOOT:
//                     intakeSpeed = UpperConstants.INTAKE_SHOOT_SPEED;
//                     shooterSpeed = UpperConstants.SHOOTER_SHOOT_SPEED;
//                     s_Upper.blink(0,255,0);
//                     break;
//                 default:
//                     break;
//             }
//         }

//         s_Upper.setElbow(-elbowPID.calculate(elbowAngle - s_Upper.getElbowRotation()));
//         s_Upper.setShooter(shooterSpeed);
//         s_Upper.setIntake(intakeSpeed);
//     }

//     @Override
//     public void end(boolean interrupted) {
//         s_Upper.setElbow(-elbowPID.calculate(elbowAngle - s_Upper.getElbowRotation()));
//         s_Upper.setShooter(shooterSpeed);
//         s_Upper.setIntake(intakeSpeed);
//     }

//     @Override
//     public boolean isFinished() {
//         if(state != null) {
//             if(Math.abs(elbowAngle - s_Upper.getElbowRotation()) <= 0.0075 && Math.abs(s_Upper.getShooterRPM()) >= UpperConstants.SHOOTER_LEGAL_SPEED) return true;
//         } else {
//             if(loadCheck){
//                 if(s_Upper.hasNote() == !loaded || Timer.getFPGATimestamp() - lastTime > time) return true;
//             }else if(Timer.getFPGATimestamp() - lastTime > time) return true;
//         }
//         return false;
//     }
// }

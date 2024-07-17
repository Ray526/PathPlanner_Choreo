// package frc.robot.commands;

// import frc.robot.subsystems.Swerve;
// import frc.robot.subsystems.UpperSub;

// import com.pathplanner.lib.path.PathPlannerPath;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.wpilibj2.command.Command;

// public class PathPlanning extends Command {
//     private final Swerve m_Swerve;
//     private final UpperSub m_upper;
//     private final Translation2d targetPosition;
//     private final Rotation2d targetRadians;

//     public PathPlanning(Swerve m_Swerve,UpperSub m_upper, Translation2d targetPosition, Rotation2d targetRadians) {
//         this.m_Swerve = m_Swerve;
//         this.m_upper = m_upper;
//         this.targetPosition = targetPosition;
//         this.targetRadians = targetRadians;
//         addRequirements(m_Swerve);
//     }

//     @Override
//     public void initialize() {
//     }

//     @Override
//     public void execute() {
//         Pose2d currentPos = m_Swerve.getPose();
//         Pose2d targetPos = new Pose2d(targetPosition, targetRadians);
//     }

//     @Override
//     public boolean isFinished() {
//         return m_upper.hasNote();
//     }

//     @Override
//     public void end(boolean interrupted) {
//         SmartDashboard.putBoolean("isInterrupted", interrupted);
//     }
// }
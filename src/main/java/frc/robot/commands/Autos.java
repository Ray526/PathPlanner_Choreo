// package frc.robot.commands;

// import java.util.Optional;

// import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.Constants.FieldConstants;
// import frc.robot.Constants.UpperConstants;
// import frc.robot.Constants.UpperState;
// import frc.robot.subsystems.Swerve;
// import frc.robot.subsystems.UpperSub;

// public class Autos {

//     // 1 note 
//     public SequentialCommandGroup Left_1n(Swerve s_Swerve, UpperSub s_Upper) {
//         return new SequentialCommandGroup(
//             new AutoResetEverything(s_Swerve),
//             new AutoUpper(s_Upper, UpperState.BASE),
//             new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_SHOOT_SPEED, FieldConstants.shootingTime, false),
//             new AutoUpper(s_Upper, UpperState.DEFAULT)
//         );
//     }
//     public SequentialCommandGroup Mid_1n(Swerve s_Swerve, UpperSub s_Upper) {
//         return new SequentialCommandGroup(
//             new AutoResetEverything(s_Swerve),
//             new AutoUpper(s_Upper, UpperState.BASE),
//             new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_SHOOT_SPEED, FieldConstants.shootingTime, false),
//             new AutoUpper(s_Upper, UpperState.DEFAULT)
//         );
//     }
//     public SequentialCommandGroup Right_1n(Swerve s_Swerve, UpperSub s_Upper) {
//         return new SequentialCommandGroup(
//             new AutoResetEverything(s_Swerve),
//             new AutoUpper(s_Upper, UpperState.BASE),
//             new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_SHOOT_SPEED, FieldConstants.shootingTime, false),
//             new AutoUpper(s_Upper, UpperState.DEFAULT)
//         );
//     }

//     /* 2 notes */
//     public SequentialCommandGroup MidX1Base_2n(Swerve s_Swerve, UpperSub s_Upper) { // this is where the problem occurs
//         return new SequentialCommandGroup(
//             new AutoResetEverything(s_Swerve),
//             new AutoResetWheels(s_Swerve),
//             new AutoUpper(s_Upper, UpperState.BASE),
//             new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_SHOOT_SPEED, FieldConstants.shootingTime, false),
//             new ParallelRaceGroup(
//                 new AutoSwerve(s_Swerve, -1.5577, -1.539, 0, 1.25, 0),
//                 new AutoUpper(s_Upper, UpperConstants.ELBOW_GROUND_POS, UpperConstants.SHOOTER_GROUND_SPEED, UpperConstants.INTAKE_GROUND_SPEED, FieldConstants.intakeTime, true)
//             ),
//             new ParallelRaceGroup(
//                 new AutoSwerve(s_Swerve, 0.15, -0.2, 0),
//                 new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_HOLD_SPEED, FieldConstants.holdTime, false)
//             ),
//             new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_HOLD_SPEED, FieldConstants.shootingTime, false),
//             new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_SHOOT_SPEED, FieldConstants.shootingTime, false),
//             new AutoUpper(s_Upper, UpperState.DEFAULT),
//             new AutoResetEverything(s_Swerve)
//         );
//     }

//     public SequentialCommandGroup MidX2Base_2n(Swerve s_Swerve, UpperSub s_Upper) {
//         return new SequentialCommandGroup(
//             new AutoResetEverything(s_Swerve),
//             new AutoResetWheels(s_Swerve),
//             new AutoUpper(s_Upper, UpperState.BASE),
//             new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_SHOOT_SPEED, FieldConstants.shootingTime, false),
//             new ParallelRaceGroup(
//                 new SequentialCommandGroup(
//                     new CustomizedDelay(0.25),
//                     new AutoSwerve(s_Swerve, -1.55, 0, 0)
//                 ),
//                 new AutoUpper(s_Upper, UpperConstants.ELBOW_GROUND_POS, UpperConstants.SHOOTER_GROUND_SPEED, UpperConstants.INTAKE_GROUND_SPEED, FieldConstants.intakeTime, true)
//             ),
//             new ParallelRaceGroup(
//                 new AutoSwerve(s_Swerve, 0.15, 0, 0),
//                 new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_HOLD_SPEED, FieldConstants.holdTime, false)
//             ),
//             new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_HOLD_SPEED, FieldConstants.shootingTime, false),
//             new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_SHOOT_SPEED, FieldConstants.shootingTime, false),
//             new AutoUpper(s_Upper, UpperState.DEFAULT),
//             new AutoResetEverything(s_Swerve)
//         );
//     }

//     public SequentialCommandGroup MidX3Base_2n(Swerve s_Swerve, UpperSub s_Upper) {
//         return new SequentialCommandGroup(
//             new AutoResetEverything(s_Swerve),
//             new AutoResetWheels(s_Swerve),
//             new AutoUpper(s_Upper, UpperState.BASE),
//             new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_SHOOT_SPEED, FieldConstants.shootingTime, false),
//             new ParallelRaceGroup(
//                 new AutoSwerve(s_Swerve, -1.5577, 1.639, 0, 1, 0),
//                 new AutoUpper(s_Upper, UpperConstants.ELBOW_GROUND_POS, UpperConstants.SHOOTER_GROUND_SPEED, UpperConstants.INTAKE_GROUND_SPEED, FieldConstants.intakeTime, true)
//             ),
//             new ParallelRaceGroup(
//                 new AutoSwerve(s_Swerve, 0.175, 0, 0),
//                 new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_HOLD_SPEED, FieldConstants.holdTime, false)
//             ),
//             new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_HOLD_SPEED, FieldConstants.shootingTime, false),
//             new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_SHOOT_SPEED, FieldConstants.shootingTime, false),
//             new AutoUpper(s_Upper, UpperState.DEFAULT),
//             new AutoResetEverything(s_Swerve)
//         );
//     }

//     /* 3 notes */
//     public SequentialCommandGroup MidX2BaseX1Base_3n(Swerve s_Swerve, UpperSub s_Upper, Optional<Alliance> optional, int side) {
//         return new SequentialCommandGroup(
//             // To X2 Then Back
//             new AutoResetEverything(s_Swerve),
//             new AutoResetWheels(s_Swerve),
//             new AutoUpper(s_Upper, UpperState.BASE),
//             new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_SHOOT_SPEED, FieldConstants.shootingTime, false),
//             new ParallelRaceGroup(
//                 new SequentialCommandGroup(
//                     new CustomizedDelay(0.35),
//                     new AutoSwerve(s_Swerve, -1.55, 0, 0)
//                 ),
//                 new AutoUpper(s_Upper, UpperConstants.ELBOW_GROUND_POS, UpperConstants.SHOOTER_GROUND_SPEED, UpperConstants.INTAKE_GROUND_SPEED, FieldConstants.intakeTime, true)
//             ),
//             new ParallelRaceGroup(
//                 new AutoSwerve(s_Swerve, 0.3, 0.05, 0),
//                 new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_HOLD_SPEED, FieldConstants.holdTime, false)
//             ),
//             new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_HOLD_SPEED, FieldConstants.shootingTime, false),
//             new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_SHOOT_SPEED, FieldConstants.shootingTime, false),
//             // To X1 Then Back
//             new AutoResetEverything(s_Swerve),
//             new ParallelRaceGroup(
//                 new AutoSwerve(s_Swerve, -1.595, -1.539, 0, 1, 0),
//                 new AutoUpper(s_Upper, UpperConstants.ELBOW_GROUND_POS, UpperConstants.SHOOTER_GROUND_SPEED, UpperConstants.INTAKE_GROUND_SPEED, FieldConstants.intakeTime, true)
//             ),
//             new ParallelRaceGroup(
//                 new AutoSwerve(s_Swerve, 0.15, 0.125, 0),
//                 new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_HOLD_SPEED, FieldConstants.holdTime, false)
//             ),
//             new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_HOLD_SPEED, FieldConstants.shootingTime, false),
//             new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_SHOOT_SPEED, FieldConstants.shootingTime, false),
//             // leave // got changed
//             new ParallelCommandGroup(
//                 new AutoSwerve(s_Swerve, -2.5, 
//                     optional.map(alliance -> alliance.name()).orElse("BLUE").equals("RED") 
//                         ? side == 0 ? -3.5 : 0.85 
//                         : side == 0 ? -0.85 : 3.5, 
//                     0, 0, 0),
//                 new AutoUpper(s_Upper, UpperState.DEFAULT)
//             )
//         );
//     }

//     public SequentialCommandGroup MidX2BaseX3Base_3n(Swerve s_Swerve, UpperSub s_Upper, String alliance, int side) {
//         return new SequentialCommandGroup(
//             // To X2 Then Back
//             new AutoResetEverything(s_Swerve),
//             new AutoResetWheels(s_Swerve),
//             new AutoUpper(s_Upper, UpperState.BASE),
//             new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_SHOOT_SPEED, FieldConstants.shootingTime, false),
//             new ParallelRaceGroup(
//                 new SequentialCommandGroup(
//                     new CustomizedDelay(0.35),
//                     new AutoSwerve(s_Swerve, -1.55, 0, 0)
//                 ),
//                 new AutoUpper(s_Upper, UpperConstants.ELBOW_GROUND_POS, UpperConstants.SHOOTER_GROUND_SPEED, UpperConstants.INTAKE_GROUND_SPEED, FieldConstants.intakeTime, true)
//             ),
//             new ParallelRaceGroup(
//                 new AutoSwerve(s_Swerve, 0.3, 0.05, 0),
//                 new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_HOLD_SPEED, FieldConstants.holdTime, false)
//             ),
//             new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_HOLD_SPEED, FieldConstants.shootingTime, false),
//             new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_SHOOT_SPEED, FieldConstants.shootingTime, false),
//             // To X3 Then Back
//             new AutoResetEverything(s_Swerve),
//             new ParallelRaceGroup(
//                 new AutoSwerve(s_Swerve, -1.575, 1.619, 0, 1, 0),
//                 new AutoUpper(s_Upper, UpperConstants.ELBOW_GROUND_POS, UpperConstants.SHOOTER_GROUND_SPEED, UpperConstants.INTAKE_GROUND_SPEED, FieldConstants.intakeTime, true)
//             ),
//             new ParallelRaceGroup(
//                 new AutoSwerve(s_Swerve, 0.15, 0.125, 0),
//                 new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_HOLD_SPEED, FieldConstants.holdTime, false)
//             ),
//             new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_HOLD_SPEED, FieldConstants.shootingTime, false),
//             new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_SHOOT_SPEED, FieldConstants.shootingTime, false),
//             // leave
//             new AutoResetEverything(s_Swerve),
//             new ParallelCommandGroup(
//                 new AutoSwerve(s_Swerve, -2.5, alliance == "RED" ? side == 0 ? -3.5 : 0.85 : side == 0 ? -0.85 : 3.5, 0, 0, 0),
//                 new AutoUpper(s_Upper, UpperState.DEFAULT)
//             )
//         );
//     }

//     /* 4 note */
//     public SequentialCommandGroup MidX2BaseX1BaseX3Base_4n(Swerve s_Swerve, UpperSub s_Upper, String alliance, int side) {
//         return new SequentialCommandGroup(
//             // To X2 Then Back
//             new AutoResetEverything(s_Swerve),
//             new AutoResetWheels(s_Swerve),
//             new AutoUpper(s_Upper, UpperState.BASE), 
//             new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_SHOOT_SPEED, FieldConstants.shootingTime, false),
//             new ParallelRaceGroup(
//                 new SequentialCommandGroup(
//                     new CustomizedDelay(0.35),
//                     new AutoSwerve(s_Swerve, -1.55, 0.6, 0)
//                 ),
//                 new AutoUpper(s_Upper, UpperConstants.ELBOW_GROUND_POS, UpperConstants.SHOOTER_GROUND_SPEED, UpperConstants.INTAKE_GROUND_SPEED, FieldConstants.intakeTime, true)
//             ),
//             new ParallelRaceGroup(
//                 new AutoSwerve(s_Swerve, 0.3, 0.05, 0),
//                 new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_HOLD_SPEED, FieldConstants.holdTime, false)
//             ),
//             new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_HOLD_SPEED, FieldConstants.shootingTime, false),
//             new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_SHOOT_SPEED, FieldConstants.shootingTime, false),
//             // To X1 Then Back
//             new AutoResetEverything(s_Swerve),
//             new ParallelRaceGroup(
//                 new AutoSwerve(s_Swerve, -1.595, -1.539, 0, 1, 0),
//                 new AutoUpper(s_Upper, UpperConstants.ELBOW_GROUND_POS, UpperConstants.SHOOTER_GROUND_SPEED, UpperConstants.INTAKE_GROUND_SPEED, FieldConstants.intakeTime, true)
//             ),
//             new ParallelRaceGroup(
//                 new AutoSwerve(s_Swerve, 0.15, 0.125, 0),
//                 new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_HOLD_SPEED, FieldConstants.holdTime, false)
//             ),
//             new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_HOLD_SPEED, FieldConstants.shootingTime, false),
//             new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_SHOOT_SPEED, FieldConstants.shootingTime, false),
//             // To X3 Then Back
//             new AutoResetEverything(s_Swerve),
//             new ParallelRaceGroup(
//                 new AutoSwerve(s_Swerve, -1.575, 1.619, 0, 1, 0), // x=-1.575
//                 new AutoUpper(s_Upper, UpperConstants.ELBOW_GROUND_POS, UpperConstants.SHOOTER_GROUND_SPEED, UpperConstants.INTAKE_GROUND_SPEED, FieldConstants.intakeTime, true)
//             ),
//             new ParallelRaceGroup(
//                 new AutoSwerve(s_Swerve, 0.15, 0, 0),
//                 new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_HOLD_SPEED, FieldConstants.holdTime, false)
//             ),
//             new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_HOLD_SPEED, FieldConstants.shootingTime, false),
//             new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_SHOOT_SPEED, FieldConstants.shootingTime, false),
//             // leave
//             new AutoResetEverything(s_Swerve),
//             new ParallelCommandGroup(
//                 new AutoSwerve(s_Swerve, -2.5, alliance == "RED" ? side == 0 ? -3.5 : 0.85 : side == 0 ? -0.85 : 3.5, 0, 0, 0),
//                 new AutoUpper(s_Upper, UpperState.DEFAULT)
//             )
//         );
//     }

//     /* leaving */
//     public SequentialCommandGroup LeftLeave_r1n(Swerve s_Swerve, UpperSub s_Upper, int delaySeconds) {
//         return new SequentialCommandGroup(
//             new AutoResetEverything(s_Swerve),
//             new AutoResetWheels(s_Swerve),
//             new AutoUpper(s_Upper, UpperState.BASE),
//             new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_SHOOT_SPEED, FieldConstants.shootingTime, false),
//             new CustomizedDelay(delaySeconds),
//             new ParallelCommandGroup(
//                 new AutoSwerve(s_Swerve, -3, 1.732, 0, 0, 0),
//                 new AutoUpper(s_Upper, UpperState.DEFAULT)
//             ),
//             new AutoResetEverything(s_Swerve)
//         );
//     }

//     public SequentialCommandGroup RightLeave_r1n(Swerve s_Swerve, UpperSub s_Upper, int delaySeconds) {
//         return new SequentialCommandGroup(
//             new AutoResetEverything(s_Swerve),
//             new AutoResetWheels(s_Swerve),
//             new AutoUpper(s_Upper, UpperState.BASE),
//             new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_SHOOT_SPEED, FieldConstants.shootingTime, false),
//             new CustomizedDelay(delaySeconds),
//             new ParallelCommandGroup(
//                 new AutoSwerve(s_Swerve, -1.5 , -1.732, 0, 0, 0),
//                 new AutoUpper(s_Upper, UpperState.DEFAULT)
//             ),
//             new AutoResetEverything(s_Swerve)
//         );
//     }

//     public SequentialCommandGroup LeftLeave_b1n(Swerve s_Swerve, UpperSub s_Upper, int delaySeconds) {
//         return new SequentialCommandGroup(
//             new AutoResetEverything(s_Swerve),
//             new AutoResetWheels(s_Swerve),
//             new AutoUpper(s_Upper, UpperState.BASE),
//             new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_SHOOT_SPEED, FieldConstants.shootingTime, false),
//             new CustomizedDelay(delaySeconds),
//             new ParallelCommandGroup(
//                 new AutoSwerve(s_Swerve, -2.5, -0.85, 0, 0, 0),
//                 new AutoUpper(s_Upper, UpperState.DEFAULT)
//             ),
//             new AutoResetEverything(s_Swerve)
//         );
//     }

//     public SequentialCommandGroup RightLeave_b1n(Swerve s_Swerve, UpperSub s_Upper, int delaySeconds) {
//         return new SequentialCommandGroup(
//             new AutoResetEverything(s_Swerve),
//             new AutoResetWheels(s_Swerve),
//             new AutoUpper(s_Upper, UpperState.BASE),
//             new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_SHOOT_SPEED, FieldConstants.shootingTime, false),
//             new CustomizedDelay(delaySeconds),
//             new ParallelCommandGroup(
//                 new AutoSwerve(s_Swerve, -2.5, 3.5, 0, 0, 0),
//                 new AutoUpper(s_Upper, UpperState.DEFAULT)
//             ),
//             new AutoResetEverything(s_Swerve)
//         );
//     }
//     public Command MidX2BaseX3Base_3n(Swerve m_Swerve, UpperSub m_upper, Optional<Alliance> alliance, int side) {
//         // TODO Auto-generated method stub
//         throw new UnsupportedOperationException("Unimplemented method 'MidX2BaseX3Base_3n'");
//     }
//     public Command MidX2BaseX1BaseX3Base_4n(Swerve m_Swerve, UpperSub m_upper) {
//         // TODO Auto-generated method stub
//         throw new UnsupportedOperationException("Unimplemented method 'MidX2BaseX1BaseX3Base_4n'");
//     }
//     public Command MidX2BaseX1Base_3n(Swerve m_Swerve, UpperSub m_upper) {
//         // TODO Auto-generated method stub
//         throw new UnsupportedOperationException("Unimplemented method 'MidX2BaseX1Base_3n'");
//     }
//     public Command MidX2BaseX3Base_3n(Swerve m_Swerve, UpperSub m_upper) {
//         // TODO Auto-generated method stub
//         throw new UnsupportedOperationException("Unimplemented method 'MidX2BaseX3Base_3n'");
//     }
// }
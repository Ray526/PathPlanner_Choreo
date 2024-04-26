// package frc.robot.subsystems;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.ChenryLib.PID;
// import frc.robot.Robot;
// import frc.robot.Constants.SwerveConstants;
// import frc.robot.Constants.robotConstants;

// public class VisionSub extends SubsystemBase{ // not using now
    
//     private final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

//     private final PID facingPID = new PID(
//       SwerveConstants.faceKP,
//       SwerveConstants.faceKI,
//       SwerveConstants.faceKD,
//       SwerveConstants.faceiWindup,
//       SwerveConstants.faceiLimit);

//     public double getTx() {
//         return table.getDoubleTopic("tx").subscribe(0.0).get();
//     }

//     public double getTy() {
//         return table.getDoubleTopic("ty").subscribe(0.0).get();
//     }

//     public double getTv() {
//         return table.getDoubleTopic("tv").subscribe(0.0).get();
//     }

//     public double getTa() {
//         return table.getDoubleTopic("ta").subscribe(0.0).get();
//     }

//     public double getTid() {
//         return table.getDoubleTopic("tid").subscribe(0.0).get();
//     }

//     public boolean hasTarget() { // need addition
//         if(Robot.alliance == "RED") {
//             if(robotConstants.mode == "AUTO" && getTid() == 4) return true;
//             else return false;
//         } else if(Robot.alliance == "BLUE") {
//             if(robotConstants.mode == "AUTO" && getTid() == 7) return true;
//             else return false;
//         } else return false;
//     }

//     public Pose2d getRobotPose() {
//         return new Pose2d(getRobotX(), getRobotY(), Rotation2d.fromDegrees(getRobotDEG()));
//     }

//     public double getRobotX() {
//         return table.getEntry("botpose").getDoubleArray(new double[6])[0];
//     }
    
//     public double getRobotY() {
//         return table.getEntry("botpose").getDoubleArray(new double[6])[1];
//     }

//     public double getRobotDEG() {
//         return table.getEntry("botpose").getDoubleArray(new double[6])[5];
//     }

//     public double calculateAutoFacing() {
//         return facingPID.calculate(getTx());
//     }

//     // public double calculateAutoAiming() {
//     //     double L1 = 0;
//     //     double L2 = 0;
//     //     double W1 = 0;
//     //     double W2 = 0;
//     //     if(Robot.alliance == "RED") {
//     //         if(FieldConstants.autoAimDataRed.get(new Translation2d(getRobotX(), getRobotY())) != null) return FieldConstants.autoAimDataRed.get(new Translation2d(getRobotX(), getRobotY()));
//     //         else {
//     //             outer:for(int i=0;i<FieldConstants.areaLengthCutPieces;i++){
//     //                 for(int j=0;j<FieldConstants.areaWidthCutPieces;j++){
//     //                     if((getRobotX()>=(FieldConstants.areaInitialXRed + i*FieldConstants.areaWidthPartial) && getRobotX()<=(FieldConstants.areaInitialXRed + (i+1)*FieldConstants.areaWidthPartial)) &&
//     //                         getRobotY()>=(FieldConstants.areaInitialYRed + j*FieldConstants.areaLengthPartial) && getRobotY()<=(FieldConstants.areaInitialYRed + (j+1)*FieldConstants.areaLengthPartial)){
//     //                             L1 = FieldConstants.areaInitialYRed + j*FieldConstants.areaLengthPartial;
//     //                             L2 = FieldConstants.areaInitialYRed + (j+1)*FieldConstants.areaLengthPartial;
//     //                             W1 = FieldConstants.areaInitialXRed + i*FieldConstants.areaWidthPartial;
//     //                             W2 = FieldConstants.areaInitialXRed + (i+1)*FieldConstants.areaWidthPartial;
//     //                             break outer;
//     //                     }
//     //                 }
//     //             }
//     //             return (FieldConstants.autoAimDataRed.get(new Translation2d(L1, W1)) + 
//     //                     FieldConstants.autoAimDataRed.get(new Translation2d(L1, W2)) + 
//     //                     FieldConstants.autoAimDataRed.get(new Translation2d(L2, W1)) +
//     //                     FieldConstants.autoAimDataRed.get(new Translation2d(L2, W2)) / 4);
//     //         }
//     //     } else if(Robot.alliance == "BLUE") {
//     //         if(FieldConstants.autoAimDataBlue.get(new Translation2d(getRobotX(), getRobotY())) != null) return FieldConstants.autoAimDataRed.get(new Translation2d(getRobotX(), getRobotY()));
//     //         else {
//     //             outer:for(int i=0;i<FieldConstants.areaLengthCutPieces;i++){
//     //                 for(int j=0;j<FieldConstants.areaWidthCutPieces;j++){
//     //                     if((getRobotX()>=(FieldConstants.areaInitialXBlue + i*FieldConstants.areaWidthPartial) && getRobotX()<=(FieldConstants.areaInitialXRed + (i+1)*FieldConstants.areaWidthPartial)) &&
//     //                         getRobotY()>=(FieldConstants.areaInitialYBlue + j*FieldConstants.areaLengthPartial) && getRobotY()<=(FieldConstants.areaInitialYRed + (j+1)*FieldConstants.areaLengthPartial)){
//     //                             L1 = FieldConstants.areaInitialYBlue + j*FieldConstants.areaLengthPartial;
//     //                             L2 = FieldConstants.areaInitialYBlue + (j+1)*FieldConstants.areaLengthPartial;
//     //                             W1 = FieldConstants.areaInitialXBlue + i*FieldConstants.areaWidthPartial;
//     //                             W2 = FieldConstants.areaInitialXBlue + (i+1)*FieldConstants.areaWidthPartial;
//     //                             break outer;
//     //                     }
//     //                 }
//     //             }
//     //             return (FieldConstants.autoAimDataBlue.get(new Translation2d(L1, W1)) + 
//     //                     FieldConstants.autoAimDataBlue.get(new Translation2d(L1, W2)) + 
//     //                     FieldConstants.autoAimDataBlue.get(new Translation2d(L2, W1)) +
//     //                     FieldConstants.autoAimDataBlue.get(new Translation2d(L2, W2)) / 4);
//     //         }
//     //     } else return 404;
//     // }

//     @Override
//     public void periodic() {
//         // SmartDashboard.putNumber("RobotPose_X", getRobotX());
//         // SmartDashboard.putNumber("RobotPose_Y", getRobotY());
//         // SmartDashboard.putNumber("RobotPose_r", getRobotDEG());
//         // SmartDashboard.putNumber("tx", getTx());
//         // SmartDashboard.putBoolean("HasTarget", hasTarget());
//     }
// }
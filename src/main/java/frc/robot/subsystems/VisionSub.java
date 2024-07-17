package frc.robot.subsystems;

import javax.swing.text.StyleContext.SmallAttributeSet;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.ChenryLib.PID;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.LimeLight;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.robotConstants;

public class VisionSub extends SubsystemBase{ //  using now
    
    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    private final NetworkTableEntry tx = table.getEntry("tx");
    private final NetworkTableEntry ty = table.getEntry("ty");
    private final NetworkTableEntry tpcs = table.getEntry("targetpose_cameraspace");

    @Override
    public void periodic() {
        SmartDashboard.putNumber("tx", tx.getDouble(0.0));
        SmartDashboard.putNumber("ty", ty.getDouble(0.0));
        SmartDashboard.putNumber("tz", tpcs.getDoubleArray(new double[6])[2]);
    }
}
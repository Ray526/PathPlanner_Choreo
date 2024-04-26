package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.ChenryLib.PID;
import frc.robot.subsystems.Swerve;

public class AutoSwerve extends Command{
    
    private final Swerve swerve;

    double x,y,z,XWindup=0,YWindup=0,X,Y,XStart,YStart;
    Translation2d trans2d;
    double rotate2d;
    boolean oneTimeX=false, oneTimeY=false;

    private final PID xyPID = new PID(3, 0, 1, 0, 0);
    private final PID zPID = new PID(5, 0, 4, 0, 0);

    public AutoSwerve(Swerve swerve, double x, double y, double z) {
        this.swerve = swerve;
        this.x = x;
        this.y = y;
        this.z = z;
        addRequirements(swerve);
    }

    public AutoSwerve(Swerve swerve, double x, double y, double z, double XWindup, double YWindup) { // XWindup use X activate Y, YWindup use Y activate X
        this.swerve = swerve;
        this.x = x;
        this.y = y;
        this.z = z;
        this.XWindup = XWindup;
        this.YWindup = YWindup;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        XStart = swerve.getPose().getX();
        YStart = swerve.getPose().getY();
    }

    @Override
    public void execute() {
        if(Math.abs(swerve.getPose().getY() - YStart) >= XWindup) {
            if(oneTimeX==true) X = xyPID.calculate(x - swerve.getPose().getX());
            else oneTimeX = true; X = xyPID.calculate(x - swerve.getPose().getX());
        }else X = 0;
        if(Math.abs(swerve.getPose().getX() - XStart) >= YWindup) {
            if(oneTimeY==true) Y = xyPID.calculate(y - swerve.getPose().getY());
            else oneTimeY = true; Y = xyPID.calculate(y - swerve.getPose().getY());
        }else Y = 0;
        trans2d = new Translation2d(X, Y);
        rotate2d =  -zPID.calculate(z - swerve.getPose().getRotation().getRotations());
        swerve.drive(trans2d, rotate2d, true, false);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(0, 0), 0, true, false);
    }

    @Override
    public boolean isFinished() {
        if( Math.abs(x - swerve.getPose().getX()) < 0.02 &&
            Math.abs(y - swerve.getPose().getY()) < 0.02 &&
            Math.abs(z - swerve.getPose().getRotation().getRotations()) < 0.005
        )  return true;
        return false;
    }
}

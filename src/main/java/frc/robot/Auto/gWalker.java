package frc.robot.Auto;

import org.team6083.lib.auto.GyroWalker;

import frc.robot.component.DriveBase;

public class gWalker {
    private static GyroWalker gyr;
    private static double kP = 0.001;
    private static double kI = 0.005;
    private static double kD = 0.002;


    public static void init(){
        gyr = new GyroWalker(DriveBase.gyro);
        gyr.setP(kP);
        gyr.setI(kI);
        gyr.setD(kD);
    }

    public static void walk(double angle){
        gyr.setTargetAngle(angle);
        gyr.calculate(0.5, 0.5);
        double leftSpeed = gyr.getLeftPower();
        double rightSpeed = gyr.getRightPower();
        DriveBase.directControl(leftSpeed, rightSpeed);
    }
}

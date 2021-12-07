package frc.robot.Auto;

import frc.robot.component.DriveBase;

import org.team6083.lib.auto.EncoderWalker;

public class eWalker {
    private static EncoderWalker eco;

    public static void walk(double distance){
        eco.walk(distance);
        eco.setWalkSpeed(0.7);
        double leftSpeed = eco.getLeftSpeed();
        double rightSpeed = eco.getRightSpeed();
        DriveBase.leftMotor1.set(leftSpeed);
        DriveBase.leftMotor2.set(leftSpeed);
        DriveBase.rightMotor1.set(rightSpeed);
        DriveBase.rightMotor2.set(rightSpeed);
    }
    public static void reset(){
        DriveBase.leftencoder.reset();
        DriveBase.rightencoder.reset();
        eco.reset();
    }
}

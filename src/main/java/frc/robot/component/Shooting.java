package frc.robot.component;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import frc.robot.Robot;

public class Shooting {
    private static WPI_VictorSPX shootLeft;
    private static final int SL = 7;

    public static void init() {
        shootLeft = new WPI_VictorSPX(SL);
    }

    public static void teleop() {
        if (Robot.maincontrol.getBButton()) {
            shootLeft.set(0.68);
        } else {
            shootLeft.set(0);
        }
    }

    public static void shoot(boolean doShoot) {
        if (doShoot) {
            shootLeft.set(0.68);
        } else {
            shootLeft.set(0);
        }
    }
    
}

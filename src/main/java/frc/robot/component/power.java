package frc.robot.component;

import org.team6083.lib.RobotPower;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class power {
    private static int port;
    private static RobotPower pow;
    
    public static void init(){
        RobotPower.init(1);//CAN_ID
        port = 10;
        pow = new RobotPower(port);
    }

    public static void Show(){
        double portCurrent = pow.getPortCurrent();//particular motor's current
        double totalCurrent = RobotPower.getTotalCurrent();//whole robot's current
        double robotVoltage = RobotPower.getRobotVoltage();//whole robot's voltage
        SmartDashboard.putNumber("ShootingMotor_Current", portCurrent);
        SmartDashboard.putNumber("WholeRobot_Current", totalCurrent);
        SmartDashboard.putNumber("WholeRobot_Voltage", robotVoltage);
    }
}

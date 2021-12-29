package frc.robot.system;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.component.DriveBase;

public class TestEngine {
    protected static Timer tim = new Timer();
    public static SendableChooser<String> choo;
    protected static String AutoSelected;
    protected static final String Front = "Front";
    protected static final String back = "back";
    protected static final String nothing = "nothing";


    public static void init(){
        choo = new SendableChooser<String>();
        choo.setDefaultOption("Knothing", nothing);
        choo.addOption("Front", Front);
        choo.addOption("back", back);
        SmartDashboard.putData("Auto choose", choo);
        System.out.println("init");
    }
    

    public static void Front(){
            DriveBase.directControl(0.3, -0.3);

    }

    public static void back(){

            DriveBase.directControl(-0.3, 0.3);
        
    }

    public static void loop(){
        AutoSelected = choo.getSelected();
        switch(AutoSelected){
            case "nothing":
            tim.start();
            SmartDashboard.putNumber("Timer", tim.get());
            DriveBase.directControl(0, 0);
            break;
            case "Front":
            Front();
            break;
            case "back":
            back();
            break;
        }
    }
}

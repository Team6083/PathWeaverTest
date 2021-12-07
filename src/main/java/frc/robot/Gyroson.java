package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;

public class Gyroson extends AHRS  {
    public Gyroson(SPI.Port port){
        super(port);
    }
}
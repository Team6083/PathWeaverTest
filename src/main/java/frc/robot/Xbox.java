package frc.robot;

import org.team6083.lib.util.XBoxController;

public class Xbox extends XBoxController {
    public Xbox(int port) {
        super(port);
    }

    @Override
    public boolean toggleReverseButton(){
        return this.getBackButton();
    }
}
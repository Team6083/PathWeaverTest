package frc.robot.Auto;


import frc.robot.component.Shooting;
import frc.robot.component.SuckSent;
import frc.robot.component.VisionTracking;

public class oldAutoStep extends oldAutoEngine {
    private static final int errAngle = 5;

    public static void loop(int angle, double dis) {
        switch (step) {
            case 0:
                /* turn "angle" */
                currentStep = "turn";
                gWalker.setTargetAngle(angle);
                gWalker.calculate(leftSpeed, rightSpeed);
                if (gWalker.getErrorAngle() < errAngle || angle == 0) {
                    nextStep();
                }
                leftSpeed = 0;
                rightSpeed = 0;
                break;
            case 1:
                /* walk "dis" inch */
                currentStep = "walk";
                eWalker.walk(dis);
                leftSpeed = eWalker.getLeftSpeed();
                rightSpeed = eWalker.getRightSpeed();
                if (eWalker.getLeftDis() > dis || eWalker.getRightDis() > dis || dis == 0) {
                    nextStep();
                }
                break;
            case 2:
                /* take aim */
                currentStep = "aim";
                isAiming = true;
                if (VisionTracking.detectIfTrackingFinished()) {
                    isAiming = false;
                    nextStep();
                } else {
                    VisionTracking.seeking();
                }
                break;
            case 3:
                /* shoot */
                currentStep = "shoot";
                leftSpeed = 0;
                rightSpeed = 0;
                gWalker.setTargetAngle(gWalker.getCurrentAngle());
                Shooting.shoot(autoTimer.get() < 5);
                SuckSent.autonomousSent();
                break;
            case 4:
                currentStep = "turn around";
                gWalker.setTargetAngle(180);
                gWalker.calculate(leftSpeed, rightSpeed);
                if (gWalker.getErrorAngle() < errAngle || angle == 0) {
                    nextStep();
                }
                leftSpeed = 0;
                rightSpeed = 0;
                break;
        }
    }
}
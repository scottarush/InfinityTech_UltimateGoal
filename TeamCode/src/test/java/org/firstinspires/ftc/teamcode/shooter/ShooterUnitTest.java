package org.firstinspires.ftc.teamcode.shooter;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.junit.Test;

/**
 * Test class to train the ring neural network.
 */
public class ShooterUnitTest {

    private Shooter mShooter = null;

    public ShooterUnitTest(){
        HardwareMap map = new HardwareMap(null);
        map.put("shooterL", new DummyDcMotor());
        map.put("shooterR", new DummyDcMotor());
        map.put("loader",new DummyServo());
        mShooter = new Shooter();
        try {
            mShooter.init(map);
        }
        catch(Exception e){
            e.printStackTrace();
        }
    }

    public void doTest(){
        mShooter.enableShooter();
        wait(2000);

     }

    public void wait(int timems){
        try {
            Thread.sleep(timems);
        }
        catch(InterruptedException e){

        }

    }

    @Test
    public void main() {
        ShooterUnitTest tester = new ShooterUnitTest();
        doTest();
    }


}
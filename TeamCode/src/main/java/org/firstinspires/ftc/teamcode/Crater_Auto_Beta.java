package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * This is an OpMode that uses a hardware robot class
 */
@Disabled
@Autonomous(name = "Crater Auto Beta", group = "IMU1")
public class Crater_Auto_Beta extends LinearOpMode {

    // this is the motor power so when you make changes you can just make here
    // feel free to define multiple like FULL_POWER, HALF_POWER, etc.
    static final double DRIVE_SPEED = 0.2;

    @Override
    public void runOpMode() {

        // -------------------------------------------------------------------------------
        // create an instance of the hardware robot class, pass an instance of THIS OpMode
        Beholder robot = new Beholder(this);

        // call the initialization method
        robot.init();

        // -------------------------------------------------------------------------------
        // Wait until the start button is clicked!
        waitForStart();

        // -------------------------------------------------------------------------------
        // Start the logging of measured acceleration
        robot.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        // -------------------------------------------------------------------------------
        // now do all of your driving and claiming depots and getting off lander or whatever
        // sleeps are not required
        // -------------------------------------------------------------------------------
        // HaHa You Cant do that
        // drive forward about 24 inches
        robot.Drive(DRIVE_SPEED, 32);
        sleep(100);



        robot.Drive(DRIVE_SPEED, -10);
        sleep(100);

        robot.Turn(85, DRIVE_SPEED);
        sleep(100);

        robot.Drive(DRIVE_SPEED, 46);
        sleep(100);

        robot.TurnL(43,DRIVE_SPEED);
        //robot.Turn(42, DRIVE_SPEED);
        //Old one that hit our outlying jewel.
        sleep(100);

        robot.Drive(DRIVE_SPEED, 40);
        sleep(100);

        robot.Drive(DRIVE_SPEED, -95);




        // turn LEFT 90 degrees
        /*robot.Turn(90, DRIVE_SPEED);
        sleep(1000);*/
        // turn RIGHT 90 degrees
        /*robot.Turn(-90, DRIVE_SPEED);
        sleep(1000);*/



        robot.StopDriving();
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by connorespenshade on 1/18/18.
 */

@Autonomous(name = "Blue Alliance")
public class JewelBlueAuton extends LinearOpMode {
    //TEST
    private ColorSensor colorSensor;
    private Servo armServo;
    private ColorType activatedColor;

    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;

    private boolean moved = false;
    private boolean driveOver = false;

    private double[] reds = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    private double[] blues = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    @Override
    public void runOpMode() throws InterruptedException {
        colorSensor = hardwareMap.colorSensor.get("color");
        armServo = hardwareMap.servo.get("arm");

        leftFront = hardwareMap.dcMotor.get(UniversalConstants.LEFT1NAME);
        leftBack = hardwareMap.dcMotor.get(UniversalConstants.LEFT2NAME);
        rightFront = hardwareMap.dcMotor.get(UniversalConstants.RIGHT1NAME);
        rightBack = hardwareMap.dcMotor.get(UniversalConstants.RIGHT2NAME);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            armServo.setPosition(1);
            sleep(5000);
            while (!moved) {
                Double red_sum = 0.0;
                Double blue_sum = 0.0;
                for (int i = reds.length - 1; i > 0; i--) {
                    reds[i] = reds[i - 1];
                    red_sum += reds[i];
                    blues[i] = blues[i - 1];
                    blue_sum += blues[i];
                }

                telemetry.addData("Red", colorSensor.red());
                telemetry.addData("Green", colorSensor.green());
                telemetry.addData("Blue", colorSensor.blue());

                reds[0] = ((double) colorSensor.red());
                blues[0] = ((double) colorSensor.blue());

                red_sum += reds[0];
                blue_sum += blues[0];
                Double red_avg = red_sum / reds.length;
                Double blue_avg = blue_sum / blues.length;
                telemetry.addData("RED_AVG", red_avg);
                telemetry.addData("BLUE_AVG", blue_avg);

                Double identifier = (red_avg - blue_avg) / red_avg;

                if (identifier <= 0) {
                    //Red
                    activatedColor = ColorType.Red;
                    sleep(500);
                    //Turns
                    gamepadDrive(0, 0, 1);
                    sleep(200);
                    gamepadDrive(0, 0, 0);
                    //Arm up
                    armServo.setDirection(Servo.Direction.REVERSE);
                    armServo.setPosition(-0.5);
                    sleep(500);
                    /*
                    //Continues to turn
                    gamepadDrive(0, 0, 1);
                    sleep(300);
                    //Stop and park
                    gamepadDrive(0, 1, 0);
                    sleep(1000);
                    gamepadDrive(0,0,0);
                    */
                    moved = true;

                } else if (identifier >= 0.4) {
                    //Blue
                    activatedColor = ColorType.Blue;
                    sleep(500);
                    //Turns
                    gamepadDrive(0, 0, -1);
                    sleep(200);
                    gamepadDrive(0, 0, 0);
                    //Arm up
                    armServo.setDirection(Servo.Direction.REVERSE);
                    armServo.setPosition(-0.5);
                    sleep(500);
                    /*
                    //Continues to turn
                    gamepadDrive(0, 0, -1);
                    sleep(300);
                    //Stop and park
                    gamepadDrive(0, -1, 0);
                    sleep(1000);
                    gamepadDrive(0,0,0);
*/
                    moved = true;
                } else {

                    activatedColor = ColorType.Unknown;
                }
                telemetry.addData("Activated Color", activatedColor);
                telemetry.update();
            }
/*
            if (!driveOver) {
                gamepadDrive(0,-1,0);
                sleep(1000);
                driveOver = true;
            }
*/
            gamepadDrive(0,0,0);
            /*
            gamepadDrive(0, 0, -1);
            sleep(250);
            gamepadDrive(0, 0, 0);
            sleep(1);
            gamepadDrive(0, 1, 0);
            sleep(1500);
            gamepadDrive(0, 0, 0);
*/
            idle();
        }
    }

    private void gamepadDrive(double leftX, double leftY, double rightX) {

        double DEADZONE = 0.05;

        double Ch3 = rightX;
        double Ch1 = -leftY;
        double Ch4 = leftX;

        double FrontLeft = Ch3 + Ch1 + Ch4;
        double RearLeft = Ch3 + Ch1 - Ch4;
        double RearRight = Ch3 - Ch1 - Ch4;
        double FrontRight = Ch3 - Ch1 + Ch4;


        //ADD DEADZONE
        leftFront.setPower(java.lang.Math.abs(FrontLeft) > DEADZONE ? FrontLeft : 0);
        leftBack.setPower(java.lang.Math.abs(RearLeft) > DEADZONE ? RearLeft : 0);
        rightFront.setPower(java.lang.Math.abs(FrontRight) > DEADZONE ? FrontRight : 0);
        rightBack.setPower(java.lang.Math.abs(RearRight) > DEADZONE ? RearRight : 0);
    }
}

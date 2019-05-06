package org.firstinspires.ftc.teamcode.teamcode2017;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.game.robot.Convert;
import org.firstinspires.ftc.teamcode.game.robot.StartPosition;

import java.util.concurrent.TimeUnit;

import static com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector.GoldLocation.CENTER;
import static com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector.GoldLocation.LEFT;
import static com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector.GoldLocation.RIGHT;


/**
 * Created by 22tjiang on 3/1/19.
 */

public class TianaTestDoc extends LinearOpMode {
    private Robot2017 robot;
    private ElapsedTime runtime = new ElapsedTime();
    private GoldAlignDetector detector;

    public void runOpMode() throws InterruptedException {
        robot = new Robot2017(true, StartPosition.marker);
        robot.init(hardwareMap);
        robot.setTelemetry(telemetry);
        robot.setTime(runtime);

        /*initDetector();*/

        inputGameConfig();

        waitForStart();
        robot.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        while (opModeIsActive()) {
            robot.composeIMUTelemetry();
            if(robot.isHooked) {
                unlatch();
            }

            robot.gyrodrive.turn(0.7, 180);

            if(robot.startPosition == StartPosition.crater){
                int angleToMarker1 = 135;
                int angleToMarker2 = 45;
                int angleToCrater = -135;
                int distToMarker1 = Convert.tileToYeetGV(2);
                int distToMarker2 = Convert.tileToYeetGV(3);
                int distToCrater1 = Convert.tileToYeetGV(2);
                int distToCrater2 = Convert.tileToYeetGV(2);

                robot.gyrodrive.turn(0.7, angleToMarker1);
                robot.gyrodrive.vertical(-0.7, distToMarker1, angleToMarker1);
                robot.gyrodrive.turn(0.7, angleToMarker2);
                robot.gyrodrive.vertical(-0.7, distToMarker2, angleToMarker2);

                deployMarker();

                robot.gyrodrive.vertical(0.7, distToCrater1, angleToCrater);
                robot.gyrodrive.





            }



        }






















    public void deployMarker() throws InterruptedException {
        robot.setMarkerDown();
        wait1(1000);
        }


    public void unlatch() throws InterruptedException {
        robot.liftMotor.setPower(-0.75);
        wait1(250);
        robot.pulleyHolder.setPosition(.655f); //latch is .168
        wait1(1500);
        robot.liftMotor.setPower(1);
        wait1(1500);
        robot.gyrodrive.vertical(0.7, Convert.tileToYeetGV(-.4), robot.getHeading());
        robot.gyrodrive.vertical(0.7, Convert.tileToYeetGV(.075), robot.getHeading());
        robot.gyrodrive.horizontal(0.7, Convert.tileToYeetGH(-0.207), robot.getHeading());
        robot.liftMotor.setPower(-0.5);
        robot.gyrodrive.vertical(0.7, Convert.tileToYeetGV(-.1), robot.getHeading());
        robot.gyrodrive.horizontal(0.7, Convert.tileToYeetGH(.1), robot.getHeading());
        }


    private void inputGameConfig() {
        telemetry.addData("Input which side", "Left (Square) or right (Crater) (use triggers)");
        telemetry.update();
        while (gamepad1.left_trigger < 0.5 && gamepad1.right_trigger < 0.5) {
        }

        if (gamepad1.left_trigger >= 0.5) {
            robot.startPosition = StartPosition.marker;
        } else {
            robot.startPosition = StartPosition.crater;
        }
        telemetry.addData("Chosen Start Position", robot.startPosition);

        telemetry.addData("Are you starting Hooked?", "Yes (Y) or No (X)");
        telemetry.update();
        while (!gamepad1.y && !gamepad1.x) {
        }

        if (gamepad1.y) {
            robot.isHooked = true;
        } else {
            robot.isHooked = false;
        }

        telemetry.addData("isHooked?", robot.isHooked);
        telemetry.addData("Start postion", robot.startPosition);
        telemetry.update();
    }

    public void wait1(int t) throws InterruptedException {
        TimeUnit.MILLISECONDS.sleep(t);
    }

    public android.hardware.Camera initVision() {
        android.hardware.Camera camera = android.hardware.Camera.open(0);

        return camera;
        //make sure to camera.release() after using
    }

    public void initDetector(){
        telemetry.addData("Status", "DogeCV 2018.0 - Sampling Order Example");

        detector = new GoldAlignDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();

        detector.downscale = 0.4; // How much to downscale the input frames

        // Optional Tuning
        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.001;

        detector.ratioScorer.weight = 15;
        detector.ratioScorer.perfectRatio = 1.0;

        detector.enable();
    }
}



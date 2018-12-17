package org.firstinspires.ftc.teamcode.teamcode2017;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.game.robot.StartPosition;
import org.firstinspires.ftc.teamcode.game.robot.TeamColor;

import java.util.concurrent.TimeUnit;

import static com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector.GoldLocation.CENTER;
import static com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector.GoldLocation.LEFT;
import static com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector.GoldLocation.RIGHT;

@TeleOp(name = "AltAutonomous", group = "Auto")
//originally had it as TeleOp b/c Autonomous wasn't working, but changed back over
public class AlternateAuto extends LinearOpMode {
    private Robot2017 robot;
    private ElapsedTime runtime = new ElapsedTime();
    private SamplingOrderDetector detector;

    public void runOpMode() throws InterruptedException {
        robot = new Robot2017(TeamColor.red, StartPosition.marker);
        robot.init(hardwareMap);
        robot.setTelemetry(telemetry);
        robot.setTime(runtime);
        initDetector();

        inputGameConfig();

        //Wait for the match to begin, presses start button
        waitForStart();
        while (opModeIsActive()) {
            // Get Down
            /*
            robot.lift.setPower(.5);
            wait1(500);
            robot.lift.setPower(0);
            robot.drive.vertical(.1);
            robot.lift.setPower(-.5);
            wait1(500);
            robot.lift.setPower(0);
            */
            SamplingOrderDetector.GoldLocation glyphPosition;
            if(detector.isFound()){
                glyphPosition = detector.getCurrentOrder();
            } else {
                // TODO: Move robot to find glyphs
                glyphPosition =  CENTER;
            }
            // If Pointed at Square ->
            if(robot.startPosition == StartPosition.marker && robot.teamColor == TeamColor.red){
                // Move Gold
                int angleToMineral;
                int angleToMarker;
                int angleToCrater;
                int distToMineral;
                int distToMarker;
                if (glyphPosition == LEFT) {
                    angleToMineral = -30;
                    angleToMarker = 52;
                    angleToCrater = 107;
                    distToMineral = 27;
                    distToMarker = 28;
                } else if (glyphPosition == CENTER){
                    angleToMineral = 0;
                    angleToMarker = 0;
                    angleToCrater = 135;
                    distToMineral = 22;
                    distToMarker = 24;
                } else if (glyphPosition == RIGHT){
                    angleToMineral = 30;
                    angleToMarker = -52;
                    angleToCrater = 158;
                    distToMineral = 27;
                    distToMarker = 18;
                } else {
                    angleToMineral = 0;
                    angleToMarker = 0;
                    angleToCrater = 135;
                    distToMineral = 14;
                    distToMarker = 15;
                }
                robot.drive.turn(angleToMineral);
                robot.drive.vertical(distToMineral);
                robot.drive.turn(angleToMarker);
                robot.drive.vertical(distToMarker);
                // Set Marker
                deployMarker();
                // Park in Crater
                robot.drive.turn(angleToCrater);
                robot.drive.vertical(57); // 24*3.5/1.574803 Moving Across 3.5ish tiles
            } else if (robot.startPosition == StartPosition.marker && robot.teamColor == TeamColor.blue){
                // Move Gold
                int angleToMineral;
                int angleToMarker;
                int angleToCrater;
                int distToMineral;
                int distToMarker;
                if (glyphPosition == LEFT) {
                    angleToMineral = -30;
                    angleToMarker = 52;
                    angleToCrater = 107;
                    distToMineral = 27;
                    distToMarker = 18;
                } else if (glyphPosition == CENTER){
                    angleToMineral = 0;
                    angleToMarker = 0;
                    angleToCrater = 135;
                    distToMineral = 14;
                    distToMarker = 15;
                } else if (glyphPosition == RIGHT){
                    angleToMineral = 30;
                    angleToMarker = -52;
                    angleToCrater = 158;
                    distToMineral = 27;
                    distToMarker = 18;
                } else {
                    angleToMineral = 0;
                    angleToMarker = 0;
                    angleToCrater = 135;
                    distToMineral = 14;
                    distToMarker = 15;
                }
                robot.drive.turn(angleToMineral);
                robot.drive.vertical(distToMineral);
                robot.drive.turn(angleToMarker);
                robot.drive.vertical(distToMarker);
                // Set Marker
                deployMarker();
                // Park in Crater
                robot.drive.turn(angleToCrater);
                robot.drive.vertical(57); // 24*3.5/1.574803 Moving Across 3.5ish tiles

                // If Pointed at Crater
            } else if (robot.startPosition == StartPosition.crater && robot.teamColor == TeamColor.red){
                // Set Marker
                robot.drive.turn(45);
                robot.drive.vertical(23);
                robot.drive.turn(-135);
                robot.drive.vertical(46);
                robot.drive.turn(135);
                robot.drive.vertical(45);
                deployMarker();
                // Park in Crater, While Moving Gold
                robot.drive.turn(90);
                robot.drive.vertical(30);
                robot.drive.turn(45);
                if(glyphPosition == LEFT){
                    robot.drive.vertical(20);
                } else if (glyphPosition == CENTER){
                    robot.drive.vertical(29);
                } else {
                    robot.drive.vertical(38);
                }
                robot.drive.horizontal(20);
            } else if (robot.startPosition == StartPosition.crater && robot.teamColor == TeamColor.blue){
                // Set Marker
                robot.drive.turn(45);
                robot.drive.vertical(23);
                robot.drive.turn(-135);
                robot.drive.vertical(46);
                robot.drive.turn(135);
                robot.drive.vertical(45);
                deployMarker();
                // Park in Crater, While Moving Gold
                robot.drive.turn(90);
                robot.drive.vertical(30);
                robot.drive.turn(45);
                if(glyphPosition == LEFT){
                    robot.drive.vertical(20);
                } else if (glyphPosition == CENTER){
                    robot.drive.vertical(29);
                } else {
                    robot.drive.vertical(38);
                }
                robot.drive.horizontal(20);
            }
        }
    }

    public void deployMarker() throws InterruptedException{
        //robot.markerServo.setPosition(.90f);
        wait1(1000);
        //robot.markerServo.setPosition(.20f);
        wait1(1000);
    }

    private void inputGameConfig() throws InterruptedException {
        telemetry.addData("Input team color", "Red (press b) or Blue (press x)");
        telemetry.update();
        while (!gamepad1.b && !gamepad1.x) {
        }

        if (gamepad1.b == true) {
            robot.teamColor = TeamColor.red;
        } else {
            robot.teamColor = TeamColor.blue;
        }
        telemetry.addData("Chosen team color", robot.teamColor);

        telemetry.addData("Input which side", "Left (Square) or right (Crater) (use triggers)");
        telemetry.update();
        while (gamepad1.left_trigger < 0.5 && gamepad1.right_trigger < 0.5) {
        }

        if (gamepad1.left_trigger >= 0.5) {
            robot.startPosition = StartPosition.marker;
        } else {
            robot.startPosition = StartPosition.crater;
        }
        telemetry.addData("Chosen start postion", robot.startPosition);
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

        detector = new SamplingOrderDetector();
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

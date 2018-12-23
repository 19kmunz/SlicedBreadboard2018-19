package org.firstinspires.ftc.teamcode.teamcode2017;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.game.robot.StartPosition;
import org.firstinspires.ftc.teamcode.game.robot.TeamColor;

import java.util.concurrent.TimeUnit;

import org.firstinspires.ftc.teamcode.game.robot.Convert;

import static com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector.GoldLocation.CENTER;
import static com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector.GoldLocation.LEFT;
import static com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector.GoldLocation.RIGHT;

@TeleOp(name = "Auto2", group = "Auto")
//originally had it as TeleOp b/c Autonomous wasn't working, but changed back over
public class Auto2 extends LinearOpMode {
    // 40 cm = 15.74803 in = 10
    private Robot2017 robot;
    private ElapsedTime runtime = new ElapsedTime();
    private GoldAlignDetector detector;

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
            robot.pulleyHolder.setPosition(.779f);
            robot.liftMotor.setPower(.5);
            robot.negLiftMotor.setPower(-.5);
            wait1(500);
            robot.liftMotor.setPower(0);
            robot.negLiftMotor.setPower(0);
            robot.drive.horizontal(1);
            robot.liftMotor.setPower(-.5);
            robot.negLiftMotor.setPower(.5);
            wait1(500);
            robot.liftMotor.setPower(0);
            robot.drive.vertical(-1);
            robot.drive.turn(180);
            */
            // SCAN GLYPHS
            SamplingOrderDetector.GoldLocation glyphPosition;
            if(detector.isFound()){
                if (detector.getXPosition() <= 160 || detector.getXPosition() > 0){
                    glyphPosition = LEFT;
                    telemetry.addData("Glyph Position:", glyphPosition);
                } else if (detector.getXPosition() > 160 || detector.getXPosition() <= 450){
                    glyphPosition = CENTER;
                    telemetry.addData("Glyph Position:", glyphPosition);
                } else if (detector.getXPosition() > 450 || detector.getXPosition() <= 640){
                    glyphPosition = RIGHT;
                    telemetry.addData("Glyph Position:", glyphPosition);
                } else {
                    glyphPosition = CENTER;
                    telemetry.addData("OUT OF BOUNDS, Default CENTER", glyphPosition);
                }
            } else {
                // TODO: Move robot to find glyphs
                glyphPosition =  CENTER;
                telemetry.addData("NO DETECTOR, Default CENTER", glyphPosition);
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
                    distToMineral = Convert.tileToYeet(1.155);
                    distToMarker = Convert.tileToYeet(1.118);
                } else if (glyphPosition == CENTER){
                    angleToMineral = 0;
                    angleToMarker = 0;
                    angleToCrater = 135;
                    distToMineral = Convert.tileToYeet(1);
                    distToMarker = Convert.tileToYeet(1);
                } else if (glyphPosition == RIGHT){
                    angleToMineral = 30;
                    angleToMarker = -52;
                    angleToCrater = 158;
                    distToMineral = Convert.tileToYeet(1.155);
                    distToMarker = Convert.tileToYeet(1.118);
                } else {
                    angleToMineral = 0;
                    angleToMarker = 0;
                    angleToCrater = 135;
                    distToMineral = Convert.tileToYeet(1);
                    distToMarker = Convert.tileToYeet(1);
                    telemetry.addData("GLYPH POSITION NOT DEFINED, Defualting", glyphPosition);
                }
                robot.drive.turn(angleToMineral);
                robot.drive.vertical(distToMineral);
                robot.drive.turn(angleToMarker);
                robot.drive.vertical(distToMarker);
                // Set Marker
                deployMarker();
                // Park in Crater
                robot.drive.turn(angleToCrater);
                robot.drive.vertical(Convert.tileToYeet(4.5)); // 24*3.5/1.574803 Moving Across 3.5ish tiles
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
                    distToMineral = Convert.tileToYeet(1.155);
                    distToMarker = Convert.tileToYeet(1.118);
                } else if (glyphPosition == CENTER){
                    angleToMineral = 0;
                    angleToMarker = 0;
                    angleToCrater = 135;
                    distToMineral = Convert.tileToYeet(1);
                    distToMarker = Convert.tileToYeet(1);
                } else if (glyphPosition == RIGHT){
                    angleToMineral = 30;
                    angleToMarker = -52;
                    angleToCrater = 158;
                    distToMineral = Convert.tileToYeet(1.155);
                    distToMarker = Convert.tileToYeet(1.118);
                } else {
                    angleToMineral = 0;
                    angleToMarker = 0;
                    angleToCrater = 135;
                    distToMineral = Convert.tileToYeet(1);
                    distToMarker = Convert.tileToYeet(1);
                    telemetry.addData("GLYPH POSITION NOT DEFINED, Defualting", glyphPosition);
                }
                robot.drive.turn(angleToMineral);
                robot.drive.vertical(distToMineral);
                robot.drive.turn(angleToMarker);
                robot.drive.vertical(distToMarker);
                // Set Marker
                deployMarker();
                // Park in Crater
                robot.drive.turn(angleToCrater);
                robot.drive.vertical(Convert.tileToYeet(4.5)); // 24*3.5/1.574803 Moving Across 3.5ish tiles

                // If Pointed at Crater
            } else if (robot.startPosition == StartPosition.crater && robot.teamColor == TeamColor.red){
                // Set Marker
                robot.drive.turn(45);
                robot.drive.vertical(Convert.tileToYeet(1.5));
                robot.drive.turn(-90);
                robot.drive.vertical(Convert.tileToYeet(3.5));
                deployMarker();
                // Park in Crater, While Moving Gold
                robot.drive.turn(180);
                robot.drive.vertical(Convert.tileToYeet(2.5));
                robot.drive.turn(45);
                if(glyphPosition == LEFT){
                    robot.drive.vertical(Convert.tileToYeet(1));
                } else if (glyphPosition == CENTER){
                    robot.drive.vertical(Convert.tileToYeet(1.5));
                } else {
                    robot.drive.vertical(Convert.tileToYeet(2));
                    telemetry.addData("Glyph position may or may not be defined", glyphPosition);
                }
                robot.drive.horizontal(Convert.tileToYeet(2));
            } else if (robot.startPosition == StartPosition.crater && robot.teamColor == TeamColor.blue){
                // Set Marker
                robot.drive.turn(45);
                robot.drive.vertical(Convert.tileToYeet(1.5));
                robot.drive.turn(-90);
                robot.drive.vertical(Convert.tileToYeet(3.5));
                deployMarker();
                // Park in Crater, While Moving Gold
                robot.drive.turn(180);
                robot.drive.vertical(Convert.tileToYeet(2.5));
                robot.drive.turn(45);
                if(glyphPosition == LEFT){
                    robot.drive.vertical(Convert.tileToYeet(1));
                } else if (glyphPosition == CENTER){
                    robot.drive.vertical(Convert.tileToYeet(1.5));
                } else {
                    robot.drive.vertical(Convert.tileToYeet(2));
                    telemetry.addData("Glyph position may or may not be defined", glyphPosition);
                }
                robot.drive.horizontal(Convert.tileToYeet(2));
            }
            idle();
        }
    }

    public void deployMarker() throws InterruptedException{
        //robot.markerServo.setPosition(.90f);
        wait1(1000);
        //robot.markerServo.setPosition(.20f);
        wait1(1000);
    }

    private void inputGameConfig() {
        telemetry.addData("Input team color", "Red (press b) or Blue (press x)");
        telemetry.update();
        while (!gamepad1.b && !gamepad1.x) {
        }

        if (gamepad1.b) {
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

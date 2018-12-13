package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.vuforia.VuMarkTarget;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaRoverRuckus;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;


@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name= "Crater Side Autonomous" , group="Autonomous")
public class Crater_Autonomous extends LinearOpMode {

    private static final String VUFORIA_KEY = "AapGrWL/////AAABmcifLBT1TUVhnZ+aTQGdBWIQHnHyHcTkFs/6OO3kzCjZLbWbrhfdOvpvqk9ZRVLxHD2QdGopI8FItrMYZq2WQG+Oy/6xBf4sOERU+LskJkrZfHowrbQ6t7UxwuQtl9lUiZeSNb4TFfJraieCeKxQOQP8ULkOXIdjn+57WtwqKEKKNB8WftNLc0Uq1+uHZnoZsIYt7Vjg/SB19frlKYKM5bbXKhj8gDje5ttns70vpz/Ry1gJl9v+B+WcLxjeUsy7w5Hmj+4373IbuSEOsPwjRwJC3HJiDBHKpCwH1QWnRo8jWzBfGqKRGKxsTBGMpm5yJsi3jxTfa/HmQtXvR8drycE1/e/nANiyH1Cy33iEohHq";

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch = 25.4f;
    private static final float mmFTCFieldWidth = (12 * 6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor


    private OpenGLMatrix lastLocation = null;
    private boolean targetVisible = false;

    private TFObjectDetector tfod;
    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;
    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;

    // Declare OpMode members.
    public DcMotor leftDrive = null;
    public DcMotor rightDrive = null;
    public DcMotor intakeSlideMotor = null;
    public DcMotor intakeFold = null;
    public DcMotor sweeperMotor = null;
    public DcMotor liftMotor = null;
    public DcMotor lynchpin = null;
    public Servo leftLiftServo = null;
    public Servo rightLiftServo = null;
    public DistanceSensor distanceSensorLeft = null;
    public DistanceSensor distanceSensorRight = null;
    public Servo craterArmServo = null;


    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        intakeSlideMotor = hardwareMap.get(DcMotor.class, "intakeSlideMotor");
        sweeperMotor = hardwareMap.get(DcMotor.class, "sweeperMotor");
        intakeFold = hardwareMap.get(DcMotor.class, "intakeFold");
        lynchpin = hardwareMap.get(DcMotor.class, "lynchpin");
        leftLiftServo = hardwareMap.get(Servo.class, "leftLiftServo");
        rightLiftServo = hardwareMap.get(Servo.class, "rightLiftServo");
        distanceSensorLeft = hardwareMap.get(DistanceSensor.class, "distSensorLeft");
        distanceSensorRight = hardwareMap.get(DistanceSensor.class, "distSensorRight");
        craterArmServo = hardwareMap.get(Servo.class, "craterArmServo");

        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        liftMotor.setDirection(DcMotor.Direction.REVERSE);

        //Set all motors to run to position and reset encoders
        SetModeRUN_TO_POSITION();

        SetZeroPowerBrake();

        // Save the calibration data to a file. You can choose whatever file
        // name you wish here, but you'll want to indicate the same file name
        // when you initialize the IMU in an opmode in which it is used. If you
        // have more than one IMU on your robot, you'll of course want to use
        // different configuration file names for each.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


        //Get calibration data
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        telemetry.addData("Status", "initialized");
        telemetry.update();

        waitForStart();

        ScoreMineral();
       /* LowerIntake();

        LowerFromLander();

        ResetLift();

        ResetIntake();

        DriveForward(300, 0.9 );

        RightTurn(750, 0.9);

        DriveBackwards(400,0.9);

        ScoreMineral();

        ResetIntake();

        LynchpinReset();
*/
        stop();
        telemetry.addData("Status", "Ready!!!");
        telemetry.update();

    }

    //METHOD SECTION

    //Drive Methods
    public void DriveForward(int distance , double speed){
        leftDrive.setTargetPosition(leftDrive.getCurrentPosition() - distance);
        rightDrive.setTargetPosition(rightDrive.getCurrentPosition() - distance);
        leftDrive.setPower(speed);
        rightDrive.setPower(speed);
        while (leftDrive.isBusy() & rightDrive.isBusy() & opModeIsActive()) {}
    }
    public void DriveBackwards(int distance, double speed){
        leftDrive.setTargetPosition(leftDrive.getCurrentPosition() + distance);
        rightDrive.setTargetPosition(rightDrive.getCurrentPosition() + distance);
        leftDrive.setPower(speed);
        rightDrive.setPower(speed);
        while (leftDrive.isBusy() & rightDrive.isBusy() & opModeIsActive()) {}
    }
    public void LeftTurn(int distance,double speed){

        leftDrive.setTargetPosition(leftDrive.getCurrentPosition() - distance);
        rightDrive.setTargetPosition(rightDrive.getCurrentPosition() + distance);
        leftDrive.setPower(speed);
        rightDrive.setPower(speed);
        while (leftDrive.isBusy() & rightDrive.isBusy() & opModeIsActive()) {}
    }
    public void RightTurn(int distance , double speed){

        leftDrive.setTargetPosition(leftDrive.getCurrentPosition() + distance);
        rightDrive.setTargetPosition(rightDrive.getCurrentPosition() - distance);
        leftDrive.setPower(speed);
        rightDrive.setPower(speed);
        while (leftDrive.isBusy() & rightDrive.isBusy() & opModeIsActive()) {}
    }

    //Standard Functions
    public void LowerFromLander(){
        lynchpin.setTargetPosition(lynchpin.getCurrentPosition() + 525);
        lynchpin.setPower(1);
        while (lynchpin.isBusy()&& opModeIsActive()){}


        //Let robot down
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setPower(.1);
        sleep (1600);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftMotor.setTargetPosition(liftMotor.getCurrentPosition() + 150);
        liftMotor.setPower(0.4);
        while (liftMotor.isBusy() && opModeIsActive()){}

        leftDrive.setTargetPosition(leftDrive.getCurrentPosition() + 400);
        rightDrive.setTargetPosition(rightDrive.getCurrentPosition() - 400);
        leftDrive.setPower(0.8);
        rightDrive.setPower(0.8);
        while (leftDrive.isBusy() & rightDrive.isBusy() & opModeIsActive()) {}
    }
    public void LowerIntake() {
        intakeFold.setTargetPosition(intakeFold.getCurrentPosition() + 400);
        intakeFold.setPower(0.5);
        while (intakeFold.isBusy() && opModeIsActive()) {
        }
    }
    public void ScoreMineral(){
        int mineralPosition = returnMineralPosition();
        telemetry.addData("Object is on the,", mineralPosition);
        telemetry.update();
    }
    public void ResetLift(){
        liftMotor.setTargetPosition(0);
        liftMotor.setPower(0.9);
        while (liftMotor.isBusy() && opModeIsActive()){}
    }
    public void DropMarker() {
        leftLiftServo.setPosition(0.7);
        rightLiftServo.setPosition(0.3);
        sleep(1900);
        leftLiftServo.setPosition(0);
        rightLiftServo.setPosition(1);
    }
    public void ResetIntake() {
        intakeFold.setTargetPosition(0);
        intakeFold.setPower(0.4);
        while (intakeFold.isBusy() && opModeIsActive()) {
        }
    }
    public void LynchpinReset(){
        lynchpin.setTargetPosition(0);
        lynchpin.setPower(1);
        while (lynchpin.isBusy()&& opModeIsActive()){}
    }


    //Gyro Methods
    public void RightGyroTurn(double degrees , double speed) {
        leftDrive.setTargetPosition(leftDrive.getCurrentPosition() + 8000);
        rightDrive.setTargetPosition(rightDrive.getCurrentPosition() - 8000);
        leftDrive.setPower(speed);
        rightDrive.setPower(speed);
        while (AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle) > -degrees+5 && leftDrive.isBusy() && rightDrive.isBusy() && opModeIsActive()) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


            telemetry.addData("heading", new Func<String>() {
                @Override
                public String value() {
                    return formatAngle(angles.angleUnit, angles.firstAngle);
                }
            });
            telemetry.update();

        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }
    public void NegativeRightGyroTurn(double degrees , double speed) {
        leftDrive.setTargetPosition(leftDrive.getCurrentPosition() + 8000);
        rightDrive.setTargetPosition(rightDrive.getCurrentPosition() - 8000);
        leftDrive.setPower(speed);
        rightDrive.setPower(speed);
        while (AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle) < -degrees+5 && leftDrive.isBusy() && rightDrive.isBusy() && opModeIsActive()) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


            telemetry.addData("heading", new Func<String>() {
                @Override
                public String value() {
                    return formatAngle(angles.angleUnit, angles.firstAngle);
                }
            });
            telemetry.update();

        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }
    public void LeftGyroTurn(double degrees , double speed) {
        leftDrive.setTargetPosition(leftDrive.getCurrentPosition() - 8000);
        rightDrive.setTargetPosition(rightDrive.getCurrentPosition() + 8000);
        leftDrive.setPower(speed);
        rightDrive.setPower(speed);
        while (AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle) < degrees-5 && leftDrive.isBusy() && rightDrive.isBusy() && opModeIsActive()) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


            telemetry.addData("heading", new Func<String>() {
                @Override
                public String value() {
                    return formatAngle(angles.angleUnit, angles.firstAngle);
                }
            });
            telemetry.update();

        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }
    public void NegativeLeftGyroTurn(double degrees , double speed) {
        leftDrive.setTargetPosition(leftDrive.getCurrentPosition() - 8000);
        rightDrive.setTargetPosition(rightDrive.getCurrentPosition() + 8000);
        leftDrive.setPower(speed);
        rightDrive.setPower(speed);
        while (AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle) > degrees-5 && leftDrive.isBusy() && rightDrive.isBusy() && opModeIsActive()) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


            telemetry.addData("heading", new Func<String>() {
                @Override
                public String value() {
                    return formatAngle(angles.angleUnit, angles.firstAngle);
                }
            });
            telemetry.update();

        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }
    public void ResetGyro() {
        imu.startAccelerationIntegration(new Position(), new Velocity(), 50);
        imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


    }
    public void CenterMineralAdjustment(){
        double Lcorrect = distanceSensorLeft.getDistance(DistanceUnit.MM) - 857;
        double Rcorrect = distanceSensorRight.getDistance(DistanceUnit.MM) - 857;
        double preHypotenuse = (Lcorrect * Lcorrect) + (Rcorrect * Rcorrect);
        double hypotenuse = Math.sqrt(preHypotenuse);
        double correctionAngle = Math.acos((hypotenuse * hypotenuse + Lcorrect * Lcorrect - Rcorrect * Rcorrect) / (2 * hypotenuse * Lcorrect));


        leftDrive.setTargetPosition(leftDrive.getCurrentPosition() + 900);
        rightDrive.setTargetPosition(rightDrive.getCurrentPosition() - 900);
        leftDrive.setPower(0.2);
        rightDrive.setPower(-0.2);
        while (AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle) > -correctionAngle + 44 & leftDrive.isBusy() & rightDrive.isBusy() & opModeIsActive()) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


            telemetry.addData("heading", new Func<String>() {
                @Override
                public String value() {
                    return formatAngle(angles.angleUnit, angles.firstAngle);
                }
            });
            telemetry.update();
        }}
    public void LeftMineralAdjustment(){
        double Lcorrect = distanceSensorLeft.getDistance(DistanceUnit.MM) - 594;
        double Rcorrect = distanceSensorRight.getDistance(DistanceUnit.MM) - 1120;
        double preHypotenuse = (Lcorrect * Lcorrect) + (Rcorrect * Rcorrect);
        double hypotenuse = Math.sqrt(preHypotenuse);
        double correctionAngle = Math.acos((hypotenuse * hypotenuse + Lcorrect * Lcorrect - Rcorrect * Rcorrect) / (2 * hypotenuse * Lcorrect));


        leftDrive.setTargetPosition(leftDrive.getCurrentPosition() + 900);
        rightDrive.setTargetPosition(rightDrive.getCurrentPosition() - 900);
        leftDrive.setPower(0.2);
        rightDrive.setPower(-0.2);
        while (AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle) > -correctionAngle + 44 & leftDrive.isBusy() & rightDrive.isBusy() & opModeIsActive()) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


            telemetry.addData("heading", new Func<String>() {
                @Override
                public String value() {
                    return formatAngle(angles.angleUnit, angles.firstAngle);
                }
            });
            telemetry.update();
        }}
    public void RightMineralAdjustment(){
        double Lcorrect = distanceSensorLeft.getDistance(DistanceUnit.MM) - 1120;
        double Rcorrect = distanceSensorRight.getDistance(DistanceUnit.MM) - 594;
        double preHypotenuse = (Lcorrect * Lcorrect) + (Rcorrect * Rcorrect);
        double hypotenuse = Math.sqrt(preHypotenuse);
        double correctionAngle = Math.acos((hypotenuse * hypotenuse + Lcorrect * Lcorrect - Rcorrect * Rcorrect) / (2 * hypotenuse * Lcorrect));


        leftDrive.setTargetPosition(leftDrive.getCurrentPosition() + 900);
        rightDrive.setTargetPosition(rightDrive.getCurrentPosition() - 900);
        leftDrive.setPower(0.2);
        rightDrive.setPower(-0.2);
        while (AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle) > -correctionAngle + 44 & leftDrive.isBusy() & rightDrive.isBusy() & opModeIsActive()) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


            telemetry.addData("heading", new Func<String>() {
                @Override
                public String value() {
                    return formatAngle(angles.angleUnit, angles.firstAngle);
                }
            });
            telemetry.update();
        }}
    public void craterArmDeploy(){


        craterArmServo.setPosition(0.5);
    }
    //Mode set Protocols
    public void SetModeRUN_TO_POSITION(){
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeFold.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lynchpin.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeFold.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sweeperMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lynchpin.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void SetZeroPowerBrake(){
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeFold.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    void composeTelemetry() {


        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = imu.getGravity();
        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
                    }
                });
    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }


    public int returnMineralPosition() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        int result = 0;

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }

            while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() == 2) {
                            int goldMineralX = -1;
                            int silverMineral1X = -1;
                            int silverMineral2X = -1;
                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    goldMineralX = (int) recognition.getLeft();
                                } else if (silverMineral1X == -1) {
                                    silverMineral1X = (int) recognition.getLeft();
                                } else {
                                    silverMineral2X = (int) recognition.getLeft();
                                }
                            }
                            if (goldMineralX == -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                                telemetry.addData("Gold Mineral Position", "Right");

                                DriveForward(100,0.5);

                                LeftGyroTurn(43, 0.2);

                                CenterMineralAdjustment();

                                RightGyroTurn(-10,0.2);

                                DriveForward(2900,0.8);

                                craterArmDeploy();

                                return result;

                            } else if (goldMineralX != -1 && silverMineral1X != -1){


                                if (goldMineralX < silverMineral1X) {
                                    telemetry.addData("Gold Mineral Position", "Center");
                                    DriveForward(100,0.5);

                                    LeftGyroTurn(43, 0.2);

                                    CenterMineralAdjustment();

                                    LeftGyroTurn(2,0.2);

                                    DriveForward(2300, 0.9);

                                    craterArmDeploy();

                                    return result;
                                } else
                                    telemetry.addData("Gold Mineral Position", "Left");
                                DriveForward(100,0.5);

                                LeftGyroTurn(43, 0.2);

                                CenterMineralAdjustment();

                                LeftGyroTurn(75,0.2);

                                DriveForward(2500,0.9);

                                craterArmDeploy();

                                return result;
                            }
                        }

                    }
                    telemetry.update();
                }
            }
        }


        if (tfod != null) {
            tfod.shutdown();
        }
        return result;
    }
    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }



}
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name = "SampleAuto", group = "Autonomous")
public class SampleAuto extends LinearOpMode {

    public void runOpMode() {
        DcMotor viperSlideOne = hardwareMap.get(DcMotor.class, "viperSlideOne");
        DcMotor viperSlideTwo = hardwareMap.dcMotor.get("viperSlideTwo");
        DcMotor intakeExtension = hardwareMap.dcMotor.get("intakeExtension");
        // geckoWheelCRServo = hardwareMap.get(CRServo.class, "geckoWheelServo");
        Servo intakeArmServo = hardwareMap.get(Servo.class, "intakeArmServo");
        Servo wristServo = hardwareMap.get(Servo.class, "wristServo");
        Servo gripperServo = hardwareMap.get(Servo.class, "gripperServo");
        DcMotor intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        Servo IntakeBlockServo = hardwareMap.get(Servo.class, "IntakeBlockServo");
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));

        viperSlideTwo.setDirection(DcMotorSimple.Direction.REVERSE);
        viperSlideOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viperSlideTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        // Delcare Trajectory as such
        Action moveToSample1 = drive.actionBuilder(new Pose2d(0, 0 ,0))
                .strafeToLinearHeading(new Vector2d(10,19), Math.toRadians(-45))
                .build();
        Action dropSample1 = drive.actionBuilder(new Pose2d(10, 19 ,-45))
                .strafeToLinearHeading(new Vector2d(8,21),Math.toRadians(-45))
                .build();

        Action safety = drive.actionBuilder(new Pose2d(8, 21 ,-45))
                .strafeToLinearHeading(new Vector2d(10,19),Math.toRadians(-45))
                .build();

        Action intakeSample2 = drive.actionBuilder(new Pose2d(10, 19 ,-45))
                .strafeToLinearHeading(new Vector2d(13,13), Math.toRadians(0))
                .build();

        Action moveToSample2 = drive.actionBuilder(new Pose2d(13, 13 ,0))
                .strafeToLinearHeading(new Vector2d(10,19), Math.toRadians(-45))
                .build();

        Action dropSample2 = drive.actionBuilder(new Pose2d(10, 19 ,-45))
                .strafeToLinearHeading(new Vector2d(8,21),Math.toRadians(-45))
                .build();

        Action safety2 = drive.actionBuilder(new Pose2d(8, 21 ,-45))
                .strafeToLinearHeading(new Vector2d(12,18),Math.toRadians(-45))
                .build();

        Action intakeSample3 = drive.actionBuilder(new Pose2d(12, 18,-45))
                .strafeToLinearHeading(new Vector2d(13,27), Math.toRadians(0))
                .turn(Math.toRadians(-3))
                .turn(Math.toRadians(3))
                .build();

        Action moveToSample3 = drive.actionBuilder(new Pose2d(11, 27 ,0))
                .strafeToLinearHeading(new Vector2d(10,19), Math.toRadians(-30))
                .build();

        Action dropSample3 = drive.actionBuilder(new Pose2d(10, 19 ,-45))
                .strafeToLinearHeading(new Vector2d(8,21),Math.toRadians(-45))
                .build();

        Action safety3 = drive.actionBuilder(new Pose2d(8, 21 ,-45))
                .strafeToLinearHeading(new Vector2d(10,19),Math.toRadians(-45))
                .build();

        Action intakeSample4 = drive.actionBuilder(new Pose2d(10, 19 ,-45))
                .strafeToLinearHeading(new Vector2d(36,10), Math.toRadians(90))
                .build();

        Action moveToSample4 = drive.actionBuilder(new Pose2d(36, 7 ,90))
                .strafeToLinearHeading(new Vector2d(10,19), Math.toRadians(-45))
                .build();

        Action dropSample4 = drive.actionBuilder(new Pose2d(10, 19 ,-45))
                .strafeToLinearHeading(new Vector2d(9,27),Math.toRadians(-45))
                .build();


        Action safety4 = drive.actionBuilder(new Pose2d(8, 21 ,-45))
                .strafeToLinearHeading(new Vector2d(9,18),Math.toRadians(-45))
                .build();

        Action park = drive.actionBuilder(new Pose2d(9, 18 ,-45))
                .strafeToLinearHeading(new Vector2d(57,10),Math.toRadians(-90))
                .build();



        while(!isStopRequested() && !opModeIsActive()) {

        }

        waitForStart();

        if (isStopRequested()) return;
        //INITIALIZATION CODE
        IntakeBlockServo.setPosition(0);
        viperSlideOne.setTargetPosition(4350);
        viperSlideTwo.setTargetPosition(4350);
        viperSlideOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlideTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlideOne.setPower(1);
        viperSlideTwo.setPower(1);
        viperSlideOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        viperSlideTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeArmServo.setPosition(0.4);
        wristServo.setPosition(0.75);
        gripperServo.setPosition(0);

        Actions.runBlocking(moveToSample1);
        sleep(750);
        Actions.runBlocking(dropSample1);
        gripperServo.setPosition(0.47);
        sleep(200);
        //SAMPLE ONE DONE
        //EXTEND STUFF FOR TAKE
        Actions.runBlocking(safety);
        viperSlideOne.setTargetPosition(0);
        viperSlideTwo.setTargetPosition(0);
        viperSlideOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlideTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlideOne.setPower(-1);
        viperSlideTwo.setPower(-1);
        viperSlideOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        viperSlideTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wristServo.setPosition(0.07);

        intakeExtension.setTargetPosition(-330);
        intakeExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeExtension.setPower(-1);
        intakeExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sleep(500);
        intakeArmServo.setPosition(0.12);
        intakeMotor.setPower(1);
        Actions.runBlocking(intakeSample2);
        sleep(500);

        //SECOND SAMPLE ATE
        intakeExtension.setTargetPosition(0);
        intakeExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeExtension.setPower(1);
        intakeExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeArmServo.setPosition(1);
        sleep(500);
        gripperServo.setPosition(0);
        sleep(750);
        intakeMotor.setPower(0);
        //DROPPING SAMPLE 2
        viperSlideOne.setTargetPosition(4350);
        viperSlideTwo.setTargetPosition(4350);
        viperSlideOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlideTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlideOne.setPower(1);
        viperSlideTwo.setPower(1);
        viperSlideOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        viperSlideTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Actions.runBlocking(moveToSample2);
        sleep(1000);
        Actions.runBlocking(dropSample2);
        wristServo.setPosition(0.75);
        sleep(500);
        gripperServo.setPosition(0.47);
        sleep(200);
        //SECOND SAMPLE DROPPED


        //EXTEND STUFF FOR TAKE
        intakeExtension.setTargetPosition(-330);
        intakeExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeExtension.setPower(-1);
        intakeExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeArmServo.setPosition(0.12);
        intakeMotor.setPower(1);
        Actions.runBlocking(safety2);
        viperSlideOne.setTargetPosition(0);
        viperSlideTwo.setTargetPosition(0);
        viperSlideOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlideTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlideOne.setPower(-1);
        viperSlideTwo.setPower(-1);
        viperSlideOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        viperSlideTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wristServo.setPosition(0.07);
        Actions.runBlocking(intakeSample3);
        sleep(500);

        //THIRD SAMPLE ATE
        intakeExtension.setTargetPosition(0);
        intakeExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeExtension.setPower(1);
        intakeExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeArmServo.setPosition(1);
        sleep(750);
        gripperServo.setPosition(0);
        sleep(500);
        intakeMotor.setPower(0);
        //DROPPING SAMPLE 3
        viperSlideOne.setTargetPosition(4350);
        viperSlideTwo.setTargetPosition(4350);
        viperSlideOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlideTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlideOne.setPower(1);
        viperSlideTwo.setPower(1);
        viperSlideOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        viperSlideTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wristServo.setPosition(0.75);
        Actions.runBlocking(moveToSample3);
        sleep(1000);
        Actions.runBlocking(dropSample3);
        gripperServo.setPosition(0.47);
        sleep(200);
        //THIRD SAMPLE DROPPED

        //EXTEND STUFF FOR TAKE
        Actions.runBlocking(safety3);
        sleep(200);
        viperSlideOne.setTargetPosition(1900);
        viperSlideTwo.setTargetPosition(1900);
        viperSlideOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlideTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlideOne.setPower(-1);
        viperSlideTwo.setPower(-1);
        viperSlideOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        viperSlideTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wristServo.setPosition(0.07);

        Actions.runBlocking(intakeSample4);
        viperSlideOne.setTargetPosition(0);
        viperSlideTwo.setTargetPosition(0);
        viperSlideOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlideTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlideOne.setPower(-1);
        viperSlideTwo.setPower(-1);
        viperSlideOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        viperSlideTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeExtension.setTargetPosition(-330);
        intakeExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeExtension.setPower(-1);
        intakeExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeArmServo.setPosition(0.12);
        intakeMotor.setPower(1);
        sleep(1000);

        //FOURTH SAMPLE ATE
        intakeExtension.setTargetPosition(0);
        intakeExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeExtension.setPower(1);
        intakeExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeArmServo.setPosition(1);
        sleep(750);
        gripperServo.setPosition(0);
        intakeMotor.setPower(0);
        sleep(300);
        //DROPPING SAMPLE 4
        viperSlideOne.setTargetPosition(4350);
        viperSlideTwo.setTargetPosition(4350);
        viperSlideOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlideTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlideOne.setPower(1);
        viperSlideTwo.setPower(1);
        viperSlideOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        viperSlideTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wristServo.setPosition(0.75);
        Actions.runBlocking(moveToSample4);
        sleep(600);
        Actions.runBlocking(dropSample4);
        gripperServo.setPosition(0.47);
        sleep(200);
        wristServo.setPosition(0.07);
        Actions.runBlocking(safety4);
        viperSlideOne.setTargetPosition(1100);
        viperSlideTwo.setTargetPosition(1100);
        viperSlideOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlideTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlideOne.setPower(-1);
        viperSlideTwo.setPower(-1);
        viperSlideOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        viperSlideTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Actions.runBlocking(park);

        //FOURTH SAMPLE DROPPED
    }
}

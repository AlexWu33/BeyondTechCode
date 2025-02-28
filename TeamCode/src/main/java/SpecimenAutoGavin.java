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
@Autonomous(name = "SpecimenAutoGavin", group = "Autonomous")
public class SpecimenAutoGavin extends LinearOpMode {

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
        Action hangSpec1 = drive.actionBuilder(new Pose2d(0, 0 ,0))
                .strafeToConstantHeading(new Vector2d(-26, 0))
                .build();

        Action intakeSpec2 = drive.actionBuilder(new Pose2d(-26, 0 ,0))
                .strafeToLinearHeading(new Vector2d(-20,18), Math.toRadians(125))
                .build();

        Action dropSpec2 = drive.actionBuilder(new Pose2d(-20, 18,125))
                .strafeToLinearHeading(new Vector2d(-20,17), Math.toRadians(40))
                .build();

        Action intakeSpec3 = drive.actionBuilder(new Pose2d(-20, 17,40))
                .strafeToLinearHeading(new Vector2d(-25,28), Math.toRadians(110))
                .build();

        Action dropSpec3 = drive.actionBuilder(new Pose2d(-25, 28,110))
                .strafeToLinearHeading(new Vector2d(-21,30),Math.toRadians(40))
                .build();

        Action grabSpec2 = drive.actionBuilder(new Pose2d(-21, 30,40))
                .strafeToLinearHeading(new Vector2d(-2.5,45), Math.toRadians(180))
                .build();

        Action hangSpec2 = drive.actionBuilder(new Pose2d(-2.5,45,180))
                .strafeToLinearHeading(new Vector2d(-21,-8),Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(-24.5,-8),Math.toRadians(0))
                .build();

        Action grabSpec3 = drive.actionBuilder(new Pose2d(-24.5, -9 ,0))
                .strafeToLinearHeading(new Vector2d(-2.5,40), Math.toRadians(180))
                .build();

        Action hangSpec3 = drive.actionBuilder(new Pose2d(-2.5, 40 ,180))
                .strafeToLinearHeading(new Vector2d(-21,-10),Math.toRadians(3))
                .strafeToLinearHeading(new Vector2d(-22,-10),Math.toRadians(3))
                .build();

        Action grabSpec4 = drive.actionBuilder(new Pose2d(-23, -10,3))
                .strafeToLinearHeading(new Vector2d(-7,45),Math.toRadians(186))
                .strafeToLinearHeading(new Vector2d(-5,45),Math.toRadians(186))
                .build();

        Action hangSpec4 = drive.actionBuilder(new Pose2d(-5, 45,186))
                .strafeToLinearHeading(new Vector2d(-21, -9),Math.toRadians(20))
                .build();

        while(!isStopRequested() && !opModeIsActive()) {

        }

        waitForStart();

        if (isStopRequested()) return;
        //INITIALIZATION CODE
        IntakeBlockServo.setPosition(0);
        viperSlideOne.setTargetPosition(1900);
        viperSlideTwo.setTargetPosition(1900);
        viperSlideOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlideTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlideOne.setPower(1);
        viperSlideTwo.setPower(1);
        viperSlideOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        viperSlideTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeArmServo.setPosition(1);
        wristServo.setPosition(0.75);
        gripperServo.setPosition(0);
        //HANGING FIRST SPECIMEN
        Actions.runBlocking(hangSpec1);
        viperSlideOne.setTargetPosition(1200);
        viperSlideTwo.setTargetPosition(1200);
        viperSlideOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlideTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlideOne.setPower(-1);
        viperSlideTwo.setPower(-1);
        viperSlideOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        viperSlideTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sleep(500);
        gripperServo.setPosition(0.45);
        //FIRST SPECIMEN HANGED
        viperSlideOne.setTargetPosition(0);
        viperSlideTwo.setTargetPosition(0);
        viperSlideOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlideTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlideOne.setPower(-1);
        viperSlideTwo.setPower(-1);
        viperSlideOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        viperSlideTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wristServo.setPosition(0.07);
        //EATING SECOND SPEC
        Actions.runBlocking(intakeSpec2);
        intakeExtension.setTargetPosition(-330);
        intakeExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeExtension.setPower(1);
        intakeExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sleep(250);
        intakeArmServo.setPosition(0.12);
        intakeMotor.setPower(1);
        sleep(850);
        Actions.runBlocking(dropSpec2);
        intakeMotor.setPower(-1);
        sleep(500);
        intakeMotor.setPower(1);
        //SECOND SPEC ATE
        Actions.runBlocking(intakeSpec3);

        intakeMotor.setPower(1);
        sleep(500);
        Actions.runBlocking(dropSpec3);
        intakeMotor.setPower(-1);
        sleep(250);

        wristServo.setPosition(0.75);
        intakeArmServo.setPosition(0.4);
        sleep(500);
        gripperServo.setPosition(0.47);
        intakeExtension.setTargetPosition(0);
        intakeExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeExtension.setPower(1);
        intakeExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeArmServo.setPosition(1);
        intakeMotor.setPower(0);
        //THIRD TRANSFERED

        //HANGING SPEC 2
        Actions.runBlocking(grabSpec2);
        gripperServo.setPosition(0);
        sleep(150);
        viperSlideOne.setTargetPosition(1925);
        viperSlideTwo.setTargetPosition(1925);
        viperSlideOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlideTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlideOne.setPower(1);
        viperSlideTwo.setPower(1);
        viperSlideOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        viperSlideTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Actions.runBlocking(hangSpec2);

        viperSlideOne.setTargetPosition(1200);
        viperSlideTwo.setTargetPosition(1200);
        viperSlideOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlideTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlideOne.setPower(-1);
        viperSlideTwo.setPower(-1);
        viperSlideOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        viperSlideTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sleep(500);
        gripperServo.setPosition(0.47);
        //SECOND SPEC HUNG

        //GETTING THIRD ONE
        viperSlideOne.setTargetPosition(0);
        viperSlideTwo.setTargetPosition(0);
        viperSlideOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlideTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlideOne.setPower(-1);
        viperSlideTwo.setPower(-1);
        viperSlideOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        viperSlideTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //THIRD ONE HUNG

        //HANGING SPEC 3
        Actions.runBlocking(grabSpec3);
        gripperServo.setPosition(0);
        sleep(150);
        viperSlideOne.setTargetPosition(1925);
        viperSlideTwo.setTargetPosition(1925);
        viperSlideOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlideTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlideOne.setPower(1);
        viperSlideTwo.setPower(1);
        viperSlideOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        viperSlideTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Actions.runBlocking(hangSpec3);

        viperSlideOne.setTargetPosition(1200);
        viperSlideTwo.setTargetPosition(1200);
        viperSlideOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlideTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlideOne.setPower(-1);
        viperSlideTwo.setPower(-1);
        viperSlideOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        viperSlideTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sleep(500);
        gripperServo.setPosition(0.47);

        //THIRD SPEC HUNG

        //HANGING SPEC 4
        viperSlideOne.setTargetPosition(0);
        viperSlideTwo.setTargetPosition(0);
        viperSlideOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlideTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlideOne.setPower(-1);
        viperSlideTwo.setPower(-1);
        viperSlideOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        viperSlideTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Actions.runBlocking(grabSpec4);
        gripperServo.setPosition(0);
        sleep(150);
        viperSlideOne.setTargetPosition(1925);
        viperSlideTwo.setTargetPosition(1925);
        viperSlideOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlideTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlideOne.setPower(1);
        viperSlideTwo.setPower(1);
        viperSlideOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        viperSlideTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Actions.runBlocking(hangSpec4);

        viperSlideOne.setTargetPosition(1200);
        viperSlideTwo.setTargetPosition(1200);
        viperSlideOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlideTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlideOne.setPower(-1);
        viperSlideTwo.setPower(-1);
        viperSlideOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        viperSlideTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sleep(500);
        gripperServo.setPosition(0.47);
        wristServo.setPosition(0.06);
        sleep(500);

        //FOURTH SPEC HUNG
    }
}


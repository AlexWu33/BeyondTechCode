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
@Autonomous(name = "SampleAutoRefined", group = "Autonomous")
public class SampleAutoRefined extends LinearOpMode {

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
                .strafeToLinearHeading(new Vector2d(8,20), Math.toRadians(-27))
                .build();

        Action intakeSample2 = drive.actionBuilder(new Pose2d(8, 20 ,-27))
                .strafeToLinearHeading(new Vector2d(13,19), Math.toRadians(0))
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
        gripperServo.setPosition(0);

        Actions.runBlocking(moveToSample1);
        sleep(500);
        wristServo.setPosition(0.75);
        sleep(750);
        gripperServo.setPosition(0.5);


        //FOURTH SAMPLE DROPPED
    }
}


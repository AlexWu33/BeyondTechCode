import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name = "ColorSensorAuto", group = "Autonomous")
public class ColorSensorAuto extends LinearOpMode {

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
        ColorSensor CS = hardwareMap.get(ColorSensor.class, "CS");
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));

        viperSlideTwo.setDirection(DcMotorSimple.Direction.REVERSE);
        viperSlideOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viperSlideTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        boolean RightColor = false;
        // Delcare Trajectory as such

        while(!isStopRequested() && !opModeIsActive()) {

        }

        waitForStart();

        if (isStopRequested()) return;
        //INITIALIZATION CODE

        while (RightColor == false && opModeIsActive()) {
            intakeExtension.setTargetPosition(-330);
            intakeExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            intakeExtension.setPower(0.1);
            intakeExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intakeArmServo.setPosition(0.12);
            intakeMotor.setPower(1);
            if (CS.red() > 750 && CS.green() < 700) {
                IntakeBlockServo.setPosition(0.6);
                intakeArmServo.setPosition(1);
                intakeExtension.setPower(0);
                telemetry.addLine("wrong color");
                sleep(1000);
            }
            intakeArmServo.setPosition(0.12);
            IntakeBlockServo.setPosition(0);
            if ((CS.red() > 1200 && CS.green() > 1500) || (CS.blue() > 600)) {
                telemetry.addLine("right color");
                intakeArmServo.setPosition(1);
                sleep(1000);
                RightColor = true;
            }

        }
        intakeMotor.setPower(-1);
        sleep(10000);



        //FOURTH SPEC HUNG
    }
}



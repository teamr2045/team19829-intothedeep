package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;


@TeleOp
public class DrivebaseTest extends OpMode {
    private DcMotorEx FrontLeft_Motor;
    private DcMotorEx FrontRight_Motor;
    private DcMotorEx RearLeft_Motor;
    private DcMotorEx RearRight_Motor;

    private OverflowEncoder xAxisEncoder;
    private OverflowEncoder yAxisEncoder;

//    private

    @Override
    public void init() {
        FrontLeft_Motor  = hardwareMap.get(DcMotorEx.class, "frontLeft");
        FrontRight_Motor = hardwareMap.get(DcMotorEx.class, "frontRight");
        RearRight_Motor  = hardwareMap.get(DcMotorEx.class, "rearRight");
        RearLeft_Motor = hardwareMap.get(DcMotorEx.class, "rearLeft");

        FrontLeft_Motor.setDirection(DcMotorEx.Direction.FORWARD);
        FrontRight_Motor.setDirection(DcMotorEx.Direction.REVERSE);
        RearLeft_Motor.setDirection(DcMotorEx.Direction.FORWARD);
        RearRight_Motor.setDirection(DcMotorEx.Direction.REVERSE);

        FrontLeft_Motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight_Motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        RearLeft_Motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        RearRight_Motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        FrontLeft_Motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        FrontRight_Motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        RearLeft_Motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        RearRight_Motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);



        xAxisEncoder = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "rearLeft")));
        yAxisEncoder = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "frontRight")));

        yAxisEncoder.setDirection(DcMotor.Direction.REVERSE);
        xAxisEncoder.setDirection(DcMotor.Direction.REVERSE);

    }

    @Override
    public void start(){

    }

    @Override
    public void loop() {
        double slide = gamepad1.left_stick_x;
        double drive = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;

        double defspeed = gamepad1.left_trigger > 0 ? 1.0 : 0.6;

        FrontLeft_Motor.setPower(Range.clip(turn + drive + slide, -1.0, 1.0) * defspeed);
        FrontRight_Motor.setPower(Range.clip(-turn + drive - slide, -1.0, 1.0) * defspeed);
        RearRight_Motor.setPower(Range.clip(-turn + drive + slide, -1.0, 1.0) * defspeed);
        RearLeft_Motor.setPower(Range.clip(turn + drive - slide, -1.0, 1.0) * defspeed);
        PositionVelocityPair xAxisPosVel = xAxisEncoder.getPositionAndVelocity();
        PositionVelocityPair yAxisPosVel = yAxisEncoder.getPositionAndVelocity();


        telemetry.addData("X Pos", xAxisPosVel.position * 0.0009286488159727595);
        telemetry.addData("X Raw Pos", xAxisPosVel.position);
        telemetry.addData("Y Pos", yAxisPosVel.position * 0.0009286488159727595);
        telemetry.addData("Y Raw Pos", yAxisPosVel.position);

        telemetry.addLine("==========");
        telemetry.addData("X Vel", xAxisPosVel.velocity);
        telemetry.addData("Y Vel", yAxisPosVel.velocity);
        telemetry.addLine("==========");
        telemetry.addData("v0", 0.00010318320177475107 * yAxisPosVel.velocity);
        telemetry.addLine("==========");
        telemetry.addData("rear Right Pos", RearRight_Motor.getCurrentPosition());
        telemetry.addData("front Right Raw Pos", FrontRight_Motor.getCurrentPosition());
        telemetry.addData("rear Left Pos", RearLeft_Motor.getCurrentPosition());
        telemetry.addData("front Left Raw Pos", FrontLeft_Motor.getCurrentPosition());

        telemetry.addData("rear left vel", RearRight_Motor.getVelocity());
        telemetry.addData("front right vel", FrontRight_Motor.getVelocity());
        telemetry.addData("rear left vel", RearLeft_Motor.getVelocity());
        telemetry.addData("front left vel", FrontLeft_Motor.getVelocity());

        telemetry.update();
    }
}

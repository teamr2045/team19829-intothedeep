package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.controllers.PIDCoefficients;
import org.firstinspires.ftc.teamcode.controllers.PIDFController;

import java.lang.reflect.Array;

@TeleOp(name = "Testing_Claw")
public class Testing_Claw extends OpMode {

    private DcMotor perpEncoder = null;
    private DcMotor parEncoder = null;

    private DcMotorEx FrontLeft_Motor;
    private DcMotorEx FrontRight_Motor;
    private DcMotorEx RearLeft_Motor;
    private DcMotorEx RearRight_Motor;

    private DcMotorEx Extendo;
    private DcMotorEx Lifter;

    private Servo ArmPivotLeft;
    private Servo ArmPivotRight;

    private Servo ClawRotator;
    private Servo Claw;
    private Servo Speciment;
    private Servo Bucket;
    private Servo BucketRotator;

    private boolean isDpadUp_pad1_Pressed = false;
    private boolean isDpadDown_pad1_Pressed = false;
    private boolean isY_pad1_Pressed = false;
    private boolean isA_pad1_Pressed = false;
    private boolean isB_pad1_Pressed = false;
    private boolean isB_pad2_Pressed = false;
    private boolean isA_pad2_Pressed = false;
    private boolean isX_pad2_Pressed = false;
    private boolean isY_pad2_Pressed = false;
    private boolean isYB_pad2_Pressed = false;


    private boolean isLeftBumper_pad1_Pressed = false;

    private double IntakePos = 0.05;

    private boolean isOuttakeInIdle = true;
    private boolean isSpecimenArmIdle = true;
    private boolean isClawIdle = true;
    private boolean isBucketIdle = true;
    private boolean isBucketRotatorIdle = true;
    private boolean isSpecimenClawIdle = false;
    private boolean isClawDirClockwise = true;
    private boolean isArmPivotDirClockwise = true;
    private boolean isLifterLifting = true;
    private boolean isInitialisingHang = false;
    private boolean isIntake_LowChamberActive = false;


    private double clawRotatorPlacementPos = 0.325;
    private double clawClosedPos = 0.705;
    private double clawOpenPos = 0.3;
    private double specimentClosedPos = 0.345;
    private double specimentOpenPos  = 1;
    private double bucketIdlePos  = 0.13;

    private double maxPivot = 0;
    private double minPivot = 0;

    private final double[] armPivot_setpoints = { 0.335, 0.8425, 0.935};
    private int armPivot_setpointIndex = 0;
    private double armIdlePos = 0.335;
    private double armRetractPos_LowChamber = 0.75;
    private double armRetractPos_HighChamber = armIdlePos;


    private double bucketDoorIdle = 0.35;
    private double bucketDoorOpen = 0;

    private double maxArmPivotPos  = 0.4;
    private double armPivotIdle  = 0.32;

    private double maxGripSpecimentPos  = 0.345;
    private double clawRotatorPos = 0;
    private double armPos = armIdlePos;
    private double bucketPos = 0;
    private double bucketDoorPos = 0;

    private PIDCoefficients ExtendoCoeffs = new PIDCoefficients(0.01, 0, 0.005);
    PIDFController ExtendoController = new PIDFController(ExtendoCoeffs);

    private PIDCoefficients LifterCoeffs = new PIDCoefficients(0.01, 0, 0.0001);
    PIDFController LifterController = new PIDFController(LifterCoeffs);

    private int extendo_currentSetpointIndex = 0;
    private int lifter_currentSetpointIndex = 0;

    private final int[] extendo_Setpoints = {0, 1000,  2200};
    private int lifterHighRungSetpoints = 1026;
    private final int[] lifter_Setpoints = {5, lifterHighRungSetpoints, 2220};

    private boolean extendoManualControl = false;
    private boolean lifterManualControl = false;
    private boolean isLifterAutomationFinished = false;

    private boolean initFinished = false;
    private double runtime = 0;
    private double lastSavedRuntime = 0;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        FrontLeft_Motor  = hardwareMap.get(DcMotorEx.class, "frontLeft");
        FrontRight_Motor = hardwareMap.get(DcMotorEx.class, "frontRight");
        RearRight_Motor  = hardwareMap.get(DcMotorEx.class, "rearRight");
        RearLeft_Motor = hardwareMap.get(DcMotorEx.class, "rearLeft");

        parEncoder  = hardwareMap.get(DcMotor.class, "parEncoder");
        perpEncoder = hardwareMap.get(DcMotor.class, "perpEncoder");

        ArmPivotLeft = hardwareMap.get(Servo.class, "armPivotLeft");
        ArmPivotRight= hardwareMap.get(Servo.class, "armPivotRight");

        ClawRotator = hardwareMap.get(Servo.class, "clawRotator");
        Claw = hardwareMap.get(Servo.class, "claw");
        Speciment = hardwareMap.get(Servo.class, "speciment");
        Bucket = hardwareMap.get(Servo.class, "bucket");
        BucketRotator = hardwareMap.get(Servo.class, "bucketRotator");

        Extendo = hardwareMap.get(DcMotorEx.class, "extendo");
        Lifter = hardwareMap.get(DcMotorEx.class, "lifter");

        Extendo.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        Extendo.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        Lifter.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        Lifter.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        ArmPivotRight.setDirection(Servo.Direction.REVERSE);
        Extendo.setDirection(DcMotorEx.Direction.REVERSE);
        Lifter.setDirection(DcMotorEx.Direction.FORWARD);

        FrontLeft_Motor.setDirection(DcMotorEx.Direction.FORWARD);
        FrontRight_Motor.setDirection(DcMotorEx.Direction.REVERSE);
        RearLeft_Motor.setDirection(DcMotorEx.Direction.FORWARD);
        RearRight_Motor.setDirection(DcMotorEx.Direction.REVERSE);
        ExtendoController.targetPosition = 0;
        LifterController.targetPosition = 0;
        telemetry.update();

        ArmPivotRight.setPosition(armIdlePos);
        ArmPivotLeft.setPosition(armIdlePos);
    }
    private double position = 0;
    private double lastPosition = 2;


    @Override
    public void start() {

        if (position != lastPosition) {
            Claw.setPosition(position);
        }

        ArmPivotRight.setPosition(0.5);
        ArmPivotLeft.setPosition(0.5);

        Claw.setPosition(clawOpenPos);
        ClawRotator.setPosition(clawRotatorPlacementPos);
        runtime = getRuntime();
        bucketPos = bucketIdlePos;
        Bucket.setPosition(bucketIdlePos);
        Speciment.setPosition(specimentOpenPos);
        bucketDoorPos = bucketDoorOpen;
        BucketRotator.setPosition(bucketDoorOpen);
    }

    @Override
    public void loop() {
        runtime += getRuntime() - runtime;

        if (gamepad1.right_bumper) {
            if (!isLeftBumper_pad1_Pressed) {
                if (isArmPivotDirClockwise && armPivot_setpointIndex == armPivot_setpoints.length - 1) {
                    isArmPivotDirClockwise = false;
                }else if ((isClawIdle && !isArmPivotDirClockwise && armPivot_setpointIndex == 1) || (!isArmPivotDirClockwise && armPivot_setpointIndex == 0)) {
                    isArmPivotDirClockwise = true;
                }

                if (isArmPivotDirClockwise && armPivot_setpointIndex != armPivot_setpoints.length ) {
                    if (armPivot_setpointIndex + 1 == armPivot_setpoints.length -1) {
                        Claw.setPosition(clawClosedPos);
                        isClawIdle =  false;
                    }
                    armPivot_setpointIndex += 1;

                }else if (!isArmPivotDirClockwise && armPivot_setpointIndex != 0) {
                    if ((!isClawIdle ) || (isClawIdle && armPivot_setpointIndex - 1 != 0)) {
                        armPivot_setpointIndex -= 1;
                    }

                    if (!isClawIdle) {
                        clawRotatorPos = clawRotatorPlacementPos;
                        ClawRotator.setPosition(clawRotatorPos);
                    }
                }
                double desiredSetpoint = (double) Array.get(armPivot_setpoints, armPivot_setpointIndex);
                armPos = desiredSetpoint;
                ArmPivotLeft.setPosition(armPos);
                ArmPivotRight.setPosition(armPos);
                isLeftBumper_pad1_Pressed = true;
            }
        }else {
            isLeftBumper_pad1_Pressed = false;
        }

        telemetry.addData("armPos", armPos);

        if (gamepad1.x && !isInitialisingHang) {
            clawRotatorPos = clawRotatorPlacementPos;
            ClawRotator.setPosition(clawRotatorPos);
            armPivot_setpointIndex = 0;
            armPos = armIdlePos;
            ArmPivotRight.setPosition(armPos);
            ArmPivotLeft.setPosition(armPos);

        }

        if (gamepad1.b || gamepad2.b ) {
            if (!isB_pad1_Pressed) {
                isClawIdle = !isClawIdle;
                if (isClawIdle) {
                    Claw.setPosition(clawOpenPos);
                }else {
                    Claw.setPosition(clawClosedPos);
                }
                isB_pad1_Pressed = true;
            }
        }else {
            isB_pad1_Pressed = false;
        }

        if (gamepad1.left_bumper ) {
            if (isClawDirClockwise && clawRotatorPos <= 0.75) {
                clawRotatorPos += 0.025;
            }else if (!isClawDirClockwise && clawRotatorPos > 0) {
                clawRotatorPos -= 0.025;
            }else {
                if (clawRotatorPos >= 0.685) {
                    isClawDirClockwise = false;
                }else if (clawRotatorPos <= 0) {
                    isClawDirClockwise = true;
                }
            }
            ClawRotator.setPosition(clawRotatorPos);
        }

        if (gamepad2.a  && !isInitialisingHang) {
            if (!isA_pad2_Pressed) {
                isBucketIdle = !isBucketIdle;
                if (armPos < 0.55) {
                    armPos = 0.55;
                    ArmPivotLeft.setPosition(armPos);
                    ArmPivotRight.setPosition(armPos);
                }

                if (isBucketIdle) {
                    isBucketRotatorIdle = true;
                    bucketDoorPos = bucketDoorIdle;
                    BucketRotator.setPosition(bucketDoorIdle);
                }else {
                    isBucketRotatorIdle = false;
                    bucketDoorPos = bucketDoorOpen;
                    BucketRotator.setPosition(bucketDoorOpen);

                    if (Lifter.getCurrentPosition() > 500) {
                        bucketPos = 0.7;
                        Bucket.setPosition(0.7);
                    }

                }
                isA_pad2_Pressed = true;
            }
        }else {
            isA_pad2_Pressed = false;
        }

        if (gamepad2.y) {
            if (!isY_pad2_Pressed) {
                isSpecimenClawIdle = !isSpecimenClawIdle;
                if (isSpecimenClawIdle) {
                    Speciment.setPosition(specimentOpenPos);
                }else {
                    Speciment.setPosition(specimentClosedPos);
                }
                isY_pad2_Pressed = true;
            }
        }else {
            isY_pad2_Pressed = false;
        }

        double slide = gamepad1.left_stick_x;
        double drive = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;

        double defspeed = gamepad1.left_trigger > 0 ? 1.0 : 0.6;

        FrontLeft_Motor.setPower(Range.clip(turn + drive + slide, -1.0, 1.0) * defspeed);
        FrontRight_Motor.setPower(Range.clip(-turn + drive - slide, -1.0, 1.0) * defspeed);
        RearRight_Motor.setPower(Range.clip(-turn + drive + slide, -1.0, 1.0) * defspeed);
        RearLeft_Motor.setPower(Range.clip(turn + drive - slide, -1.0, 1.0) * defspeed);

        if (gamepad1.y || gamepad1.a) {
            extendoManualControl = false;
            if (gamepad1.y && !isY_pad1_Pressed && extendo_currentSetpointIndex != extendo_Setpoints.length-1) {
                extendo_currentSetpointIndex += 1;
                isY_pad1_Pressed = true;
            }else if (gamepad1.a && !isA_pad1_Pressed && extendo_currentSetpointIndex != 0) {
                extendo_currentSetpointIndex -= 1;
                isA_pad1_Pressed = true;
            }
            int desiredSetpoint = (int) Array.get(extendo_Setpoints, extendo_currentSetpointIndex);
            ExtendoController.targetPosition = desiredSetpoint;

        }else {
            if (!gamepad1.a && isY_pad1_Pressed) {
                isY_pad1_Pressed = false;
            }else if (!gamepad1.a && isA_pad1_Pressed) {
                isA_pad1_Pressed = false;
            }
        }

        if (gamepad1.dpad_up && Extendo.getCurrentPosition() < 1995) {
            Extendo.setPower( 0.75);
            extendoManualControl = true;
        }else if (gamepad1.dpad_down) {
            Extendo.setPower(-0.75);
            extendoManualControl = true;
        }else if (extendoManualControl) {
            Extendo.setPower(0);
        }

        if (Math.abs(gamepad2.right_stick_y) > 0.1) {
            extendoManualControl = true;
            if (gamepad2.right_stick_y < 0 && Extendo.getCurrentPosition() < 1995) {
                Extendo.setPower(0.75);
            }else if (gamepad2.right_stick_y > 0) {
                Extendo.setPower(-0.75);
            }
        }

        if (!extendoManualControl) {
            double extendoDesiredPower = ExtendoController.update(Extendo.getCurrentPosition());
            Extendo.setPower(extendoDesiredPower);
        }

        if ((Extendo.getCurrentPosition() < 10 )  && isIntake_LowChamberActive) {
            armPos = 0.3775;
            ArmPivotLeft.setPosition(armPos);
            ArmPivotRight.setPosition(armPos);
//            0.3775
        }else {
            double nextArmPos = armPos;
            if (Extendo.getVelocity() < -2000 && !isInitialisingHang) {
                armPivot_setpointIndex = 0;

                if (isIntake_LowChamberActive) {
                    nextArmPos = armRetractPos_LowChamber;
                    ArmPivotLeft.setPosition(armPos);
                    ArmPivotRight.setPosition(armPos);
                }else {
                    nextArmPos = armRetractPos_HighChamber;
                    ArmPivotLeft.setPosition(armPos);
                    ArmPivotRight.setPosition(armPos);
                }

                clawRotatorPos = 0.32;
                ClawRotator.setPosition(clawRotatorPos);
                armPivot_setpointIndex = 0;

            }

            if (nextArmPos != armPos) {
                armPos = nextArmPos;
                ArmPivotLeft.setPosition(nextArmPos);
                ArmPivotRight.setPosition(nextArmPos);
            }
        }


        if (gamepad2.right_bumper && gamepad2.left_bumper) {
            if (!isYB_pad2_Pressed) {
                if (!isInitialisingHang) {
                    armPos = 0.7;
                    ArmPivotLeft.setPosition(armPos);
                    ArmPivotRight.setPosition(armPos);

                    isInitialisingHang = true;
                    lifterManualControl = false;
                    LifterController.targetPosition = 400;

                    bucketPos = 0.75;
                    Bucket.setPosition(bucketPos);

                    extendoManualControl = false;
                    ExtendoController.targetPosition = 1700;

                    clawRotatorPos = clawRotatorPlacementPos;
                    ClawRotator.setPosition(clawRotatorPos);
                    lastSavedRuntime = runtime;
                }else {
                    isInitialisingHang = false;
                    extendoManualControl = false;
                    ExtendoController.targetPosition = 1700;

                    bucketPos = bucketIdlePos;
                    Bucket.setPosition(bucketIdlePos);
                    clawRotatorPos = clawRotatorPlacementPos;
                    ClawRotator.setPosition(clawRotatorPos);
                }
                isYB_pad2_Pressed = true;
            }
        }else {
            isYB_pad2_Pressed = false;
        }

        if (Math.abs(gamepad2.left_trigger) > 0.1){
            parEncoder.setPower(-1.0);
            perpEncoder.setPower(1.0);

        }else if (Math.abs(gamepad2.right_trigger) > 0.1 ){
            parEncoder.setPower(1.0);
            perpEncoder.setPower(-1.0);
        }else{
            parEncoder.setPower(0.0);
            perpEncoder.setPower(0.0);
        }

        if (Math.abs(ExtendoController.lastError) < 10 && ExtendoController.targetPosition == 1700) {
            if (isInitialisingHang) {
                if (armPos != 0 && LifterController.targetPosition == 400 && LifterController.lastError < 20 && ((runtime - lastSavedRuntime) > 0.7)) {
                    armPos = 0;
                    ArmPivotLeft.setPosition(armPos);
                    ArmPivotRight.setPosition(armPos);
                }else {
                    extendoManualControl = false;
                    ExtendoController.targetPosition = 0;
                }

            }else {
                armPos = 0.66;
                ArmPivotLeft.setPosition(armPos);
                ArmPivotRight.setPosition(armPos);

                lifterManualControl = false;
                LifterController.targetPosition = 0;

                extendoManualControl = false;
                ExtendoController.targetPosition = 0;
            }


        }


        // Lifter
        if ((gamepad2.dpad_up || gamepad2.dpad_down) && !isInitialisingHang) {
            lifterManualControl = false;
            if (gamepad2.dpad_up && !isDpadUp_pad1_Pressed && lifter_currentSetpointIndex != lifter_Setpoints.length-1) {
                isLifterLifting = true;
                if (lifter_currentSetpointIndex == 0) {
                    bucketPos = 0.1;
                    Bucket.setPosition(0.1);
                    if (armPos < 0.55) {
                        armPos = 0.55;
                        ArmPivotLeft.setPosition(armPos);
                        ArmPivotRight.setPosition(armPos);
                    }
                    bucketDoorPos = bucketDoorIdle;
                    BucketRotator.setPosition(bucketDoorIdle);

                }
                lifter_currentSetpointIndex += 1;
                isDpadUp_pad1_Pressed = true;
            }else if (gamepad2.dpad_down && !isDpadDown_pad1_Pressed && lifter_currentSetpointIndex != 0) {
                isLifterLifting = false;
                bucketPos = bucketIdlePos;
                Bucket.setPosition(bucketPos);
                if (armPos < 0.55) {
                    armPos = 0.55;
                    ArmPivotLeft.setPosition(armPos);
                    ArmPivotRight.setPosition(armPos);
                }
                if (lifter_currentSetpointIndex - 1 == 0) {
                    bucketDoorPos = bucketDoorOpen;
                    BucketRotator.setPosition(bucketDoorOpen);
                }

                lifter_currentSetpointIndex -= 1;
                isDpadDown_pad1_Pressed = true;
            }

            int desiredSetpoint = (int) Array.get(lifter_Setpoints, lifter_currentSetpointIndex);
            LifterController.targetPosition = desiredSetpoint;
        }else {
            if (!gamepad2.dpad_up && isDpadUp_pad1_Pressed) {
                isDpadUp_pad1_Pressed = false;
            }else if (!gamepad2.dpad_down && isDpadDown_pad1_Pressed) {
                isDpadDown_pad1_Pressed = false;
            }
        }

        if (gamepad2.x) {
            isIntake_LowChamberActive = true;
        }else if (isIntake_LowChamberActive) {
            isIntake_LowChamberActive = false;
        }

        if ((gamepad2.left_stick_y != 0 || lifterManualControl) && !isInitialisingHang) {
            lifterManualControl = true;
            Lifter.setPower(-gamepad2.left_stick_y);

            if (Lifter.getPower() > 0 && !isLifterLifting) {
                isLifterLifting = true;
            }else if (Lifter.getPower() < 0 && isLifterLifting) {
                isLifterLifting = false;
            }
        }

        if (!lifterManualControl) {
            double lifterDesiredPower = LifterController.update(Lifter.getCurrentPosition());
            Lifter.setPower(lifterDesiredPower);
        }

        if (Lifter.getVelocity() > 1500 && isLifterLifting) {
            if (Lifter.getCurrentPosition() > 500) {
//                bucketPos = 0.785;
//                Bucket.setPosition(bucketPos);
//
//                bucketDoorPos = bucketDoorOpen;
//                BucketRotator.setPosition(bucketDoorOpen);

            }
        }else if (Lifter.getVelocity() < -1500 && !isLifterLifting) {
            bucketPos = bucketIdlePos;
            Bucket.setPosition(bucketPos);
            isBucketRotatorIdle = false;
            bucketDoorPos = bucketDoorOpen;
            BucketRotator.setPosition(bucketDoorOpen);

            if (armPos < 0.55) {
                armPos = 0.55;
                ArmPivotLeft.setPosition(armPos);
                ArmPivotRight.setPosition(armPos);
            }
//            BucketRotator.setPosition();
        }

        telemetry.addData("Lifter Manual Control ", lifterManualControl);

        telemetry.addData("runtime ", runtime);
        telemetry.addData("lastSavedRuntime ", lastSavedRuntime);
        telemetry.addData("difference ", runtime - lastSavedRuntime);


        telemetry.addData("Claw Rotator Position ", clawRotatorPos);
        telemetry.addData("Arm Position ", armPos);

        telemetry.addData("Gamepad Right Stick Y ", gamepad2.right_stick_y);
        telemetry.addData("Extendo Velocity", Extendo.getVelocity());
        telemetry.addData("Lifter Index", lifter_currentSetpointIndex);
        telemetry.addData("Lifter Setpoints", lifter_Setpoints);
        telemetry.addData("Lifter Pos", Lifter.getCurrentPosition());
        telemetry.addData("Lifter Vel", Lifter.getVelocity());

        telemetry.addData("Dpad Left", gamepad1.dpad_left);

        telemetry.addData("Extendo Encoder ", Extendo.getCurrentPosition());
        telemetry.addData("gamepad2.x ", gamepad2.x);

        telemetry.update();
    }
}

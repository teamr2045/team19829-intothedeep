package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.controllers.PIDCoefficients;
import org.firstinspires.ftc.teamcode.controllers.PIDFController;

public class Lifter {
    private PIDCoefficients LifterCoeffs = new PIDCoefficients(0.01, 0, 0.0001);
    PIDFController LifterController = new PIDFController(LifterCoeffs);
    private double currentSetpoint = 0;

    private  int[] lifter_Setpoints = {5, 1000, 2180};


    public LifterController CurrentLifterControllerAction = null;

    private DcMotorEx LifterMotor;
    private Servo SpecimentServo;

    private Servo Bucket;
    private Servo BucketRotator;


    private String lifterPinName = "lifter";
    private String spcimentServoPinName = "speciment";
    private DcMotorSimple.Direction MotorDir = DcMotorSimple.Direction.FORWARD;

    private double specimentMaxLevelSetpoint = 1532; // use as a reference to put the speciment
    private double spcimentMinLevelSetpoint = 858; // use as a reference to take off claw.
    private double specimentClosedPos = 0.345;
    private double specimentOpenPos  = 1;

    private double bucketIdlePos  = 0.13;
    private double bucketDoorIdle = 0.35;
    private double bucketDoorOpen = 0;
    private double bucketFlipPos = 0.715;



    private double bucketPos = 0;


    public Lifter(HardwareMap hardwareMap) {
        LifterMotor = hardwareMap.get(DcMotorEx.class, lifterPinName);
        SpecimentServo = hardwareMap.get(Servo.class, spcimentServoPinName);

        LifterMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        LifterMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        LifterMotor.setDirection(DcMotorEx.Direction.FORWARD);

        Bucket = hardwareMap.get(Servo.class, "bucket");
        BucketRotator = hardwareMap.get(Servo.class, "bucketRotator");


//        ArmPivotLeft = hardwareMap.get(Servo.class, "armPivotLeft");
//        ArmPivotRight= hardwareMap.get(Servo.class, "armPivotRight");
//        ArmPivotRight.setDirection(Servo.Direction.REVERSE);
    }

    public void runAuto() {
        double lifterDesiredPower = LifterController.update(LifterMotor.getCurrentPosition());
        LifterMotor.setPower(lifterDesiredPower);
    }
    public void sendTelemetryAuto(TelemetryPacket telemetry) {
        telemetry.addLine("Running Lifter Auto");
        telemetry.put("Lifter Setpoint", LifterController.targetPosition);
        telemetry.put("Lifter Error", LifterController.lastError);
        telemetry.put("Lifter Pos", LifterMotor.getCurrentPosition());

    }

    public Action flipBucket() {
        return new Action() {
            private boolean isInitialised = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!isInitialised) {
                    BucketRotator.setPosition(bucketDoorOpen);

                    bucketPos = bucketFlipPos;
                    Bucket.setPosition(bucketPos);

                    isInitialised = true;
                    return true;
                }
                return false;
            }
        };
    }

    public Action openBucket() {
        return new Action() {
            private boolean isInitialised = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!isInitialised) {

                    telemetryPacket.addLine("OPEN BUCKET!!");
                    BucketRotator.setPosition(bucketDoorOpen);

                    isInitialised = true;
                    return true;
                }
                return false;
            }
        };
    }
    public Action setBucketToIdle() {
        return new Action() {
            private boolean isInitialised = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!isInitialised) {
                    bucketPos = bucketIdlePos;
                    Bucket.setPosition(bucketIdlePos);

                    isInitialised = true;
                    return true;
                }
                return false;
            }
        };
    }
    public Action resetBucket_toClose() {
        return new Action() {
            private boolean isInitialised = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!isInitialised) {
                    bucketPos = bucketIdlePos;
                    Bucket.setPosition(bucketPos);
                    BucketRotator.setPosition(bucketDoorIdle);

                    isInitialised = true;
                    return true;
                }
                return false;
            }
        };
    }
    public Action resetBucket_toOpen() {
        return new Action() {
            private boolean isInitialised = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!isInitialised) {
                    bucketPos = bucketIdlePos;
                    Bucket.setPosition(bucketPos);
                    BucketRotator.setPosition(bucketDoorOpen);

                    isInitialised = true;
                    return true;
                }
                return false;
            }
        };
    }

    public Action setLifterPosition(double setpoint) {
        CurrentLifterControllerAction = new LifterController(setpoint);
        return CurrentLifterControllerAction;
    }
    public Action setLifterPosition_v2(double setpoint) {
        return new LifterController_v2(setpoint);
    }
    public Action setLifterPosition_v2(double setpoint, Action callback) {
        return new LifterController_v2(setpoint, callback);
    }

    public  class CancelLifter implements Action {
        private boolean initialised = false;
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            telemetryPacket.addLine("CANCEL NOW 11!!");
            if (!initialised) {
                telemetryPacket.addLine("CANCEL NOW!!");
                CurrentLifterControllerAction.CancelAction();
                initialised = true;
                return true;
            }
            telemetryPacket.addLine("FINISHED CANCELING NOW 11!!");

            return false;
        }
    }
    public Action cancelLift() {
        return new CancelLifter();
    }
    public void cancelLifter() {
        CurrentLifterControllerAction.CancelAction();
    }
    private void CancelLifterController() {
        if (CurrentLifterControllerAction != null) {
            CurrentLifterControllerAction.CancelAction();
        }
    }

    public class LifterController implements Action {
        PIDFController LifterController = new PIDFController(LifterCoeffs);

        private double setpoint ;
        public boolean cancel = false;

        private double breakingPoint = -1;
        private Action CallbackAction = null;

        public LifterController(double setpoint) {
            currentSetpoint = setpoint;
            this.setpoint = setpoint;
            LifterController.targetPosition = this.setpoint;
        }

        public LifterController(double setpoint,  Action callbackAction, double breakingPoint) {
            currentSetpoint = setpoint;
            this.setpoint = setpoint;
            LifterController.targetPosition = this.setpoint;
            this.CallbackAction = callbackAction;
            this.breakingPoint = breakingPoint;
        }

        public LifterController(double setpoint,  Action callbackAction) {
            currentSetpoint = setpoint;
            this.setpoint = setpoint;
            LifterController.targetPosition = this.setpoint;
            this.CallbackAction = callbackAction;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            telemetryPacket.addLine("Run Lifter");

            telemetryPacket.put("Cancel", cancel);

            telemetryPacket.put("Lifter Error", LifterController.lastError);
            telemetryPacket.put("Lifter Setpoint", this.setpoint);

            if (cancel) {
                return false;
            }else {
                double lifterDesiredPower = LifterController.update(LifterMotor.getCurrentPosition());
                LifterMotor.setPower(lifterDesiredPower);

                if ((this.breakingPoint != -1 && ( Math.abs(LifterController.lastError) < breakingPoint)) || (Math.abs(LifterController.lastError) < 50)) {
                    if (this.CallbackAction != null) {
                        return this.CallbackAction.run(telemetryPacket);
                    }
                }

                if (this.setpoint == 0 && Math.abs(LifterController.lastError) < 20) {
                    return false;
                }

            }
            return true;
        }

        public void CancelAction() {
            cancel = true;
        }
    }

    public Action releaseSpeciment() {
        return new LifterController(850, servoReleaseSpeciment() );
    }
    public Action releaseSpeciment_v2() {
        return new LifterController_v2(830, servoReleaseSpeciment() );
    }
    public Action servoReleaseSpeciment() {
        return new ServoReleaseSpeciment();
    }

    public Action grabSpeciment() {
        return new GrabSpeciment();
    }

    public class ServoReleaseSpeciment implements Action {
        @Override
        public boolean run (@NonNull TelemetryPacket telemetryPacket) {
            SpecimentServo.setPosition(specimentOpenPos);
            return false;
        }
    }

    public class GrabSpeciment implements Action {
        @Override
        public boolean run (@NonNull TelemetryPacket telemetryPacket) {
            SpecimentServo.setPosition(specimentClosedPos);
            return false;
        }
    }



    public class LifterController_v2 implements Action {
        private boolean initialised = false;
        private Action CallbackAction = null;
        private double breakingPoint = -1;
        private double setpoint = 0;

        public LifterController_v2(double setpoint) {
            this.setpoint = setpoint;
        }

        public LifterController_v2(double setpoint,  Action callbackAction) {
            this.setpoint = setpoint;
            this.CallbackAction = callbackAction;
        }
        public LifterController_v2(double setpoint,  Action callbackAction, double breakingPoint) {
            this.setpoint = setpoint;

            this.CallbackAction = callbackAction;
            this.breakingPoint = breakingPoint;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!this.initialised) {
                currentSetpoint = this.setpoint;
                LifterController.targetPosition = this.setpoint;
                this.initialised = true;
                return true;
            }else {
                telemetryPacket.put("CALLBACK", this.CallbackAction);

                if (this.CallbackAction != null) {
                    telemetryPacket.addLine("CALLBACK!");

                    if ((this.breakingPoint != -1 && ( Math.abs(LifterController.lastError) < breakingPoint)) || (Math.abs(LifterController.lastError) < 50)) {
                        return this.CallbackAction.run(telemetryPacket);
                    }

                    return  true;
                }
                return false;
            }
        }
    }


}

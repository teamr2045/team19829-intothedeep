package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.controllers.PIDCoefficients;
import org.firstinspires.ftc.teamcode.controllers.PIDFController;

public class Extendo {
    private DcMotorEx Extendo;
    private String pinName = "extendo";
    private DcMotorSimple.Direction MotorDir = DcMotorSimple.Direction.FORWARD;

    private Servo Claw;
    private Servo ClawRotator;


    private Servo ArmPivotLeft;
    private Servo ArmPivotRight;

    private double clawClosedPos = 0.705;
    private double clawOpenPos = 0.3;

    private double armIdlePos = 0.335;
    private double clawRotatorPos_1_2 = 0.325;
    private double clawRotatorPos_1_3 = 0.65;


    private PIDCoefficients ExtendoCoeffs = new PIDCoefficients(0.01, 0, 0.005);
    PIDFController ExtendoController = new PIDFController(ExtendoCoeffs);
    private double currentSetpoint = 0;


    public Extendo(HardwareMap hardwareMap) {
        Extendo = hardwareMap.get(DcMotorEx.class, pinName);
        ExtendoController.targetPosition = 0;

        Extendo.setDirection(DcMotorEx.Direction.REVERSE);
        Extendo.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        Extendo.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        ArmPivotLeft = hardwareMap.get(Servo.class, "armPivotLeft");
        ArmPivotRight= hardwareMap.get(Servo.class, "armPivotRight");
        ArmPivotRight.setDirection(Servo.Direction.REVERSE);

        ClawRotator = hardwareMap.get(Servo.class, "clawRotator");
        Claw = hardwareMap.get(Servo.class, "claw");

        ClawRotator.setPosition(clawRotatorPos_1_2);


    }

    public void runAuto() {
        double lifterDesiredPower = ExtendoController.update(Extendo.getCurrentPosition());
        Extendo.setPower(lifterDesiredPower);
    }

    public void sendTelemetryAuto(TelemetryPacket telemetry) {
        telemetry.addLine("Running Extendo Auto");
        telemetry.put("Extendo Setpoint", currentSetpoint);
        telemetry.put("Extendo Error", ExtendoController.lastError);
        telemetry.put("Extendo Pos", Extendo.getCurrentPosition());

    }

    public Action setClawRotatorPos (double position) {
        return new Action() {
            private boolean isInitialised = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!isInitialised) {
                    ClawRotator.setPosition(position);
                    return true;
                }

                return false;
            }
        };
    }

    public Action takeSample() {
        return new Action() {
            private boolean isInitialised = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!isInitialised) {
                    Claw.setPosition(clawClosedPos);
                    ArmPivotLeft.setPosition(0.935);
                    ArmPivotRight.setPosition(0.935);
                    isInitialised = true;
                    return true;
                }

                return false;
            }
        };
    }

    public Action resetArmToIdle() {
        return new Action() {
            private boolean isInitialised = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!isInitialised) {
                    ArmPivotLeft.setPosition(armIdlePos);
                    ArmPivotRight.setPosition(armIdlePos);
                    isInitialised = true;
                    return true;
                }

                return false;
            }
        };
    }

    public Action retract_and_putSampleToBucket() {
        return new ExtendoController(0, new Action() {
            private boolean isInitialised = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!isInitialised) {
                    Claw.setPosition(clawOpenPos);
                    isInitialised = true;
                    return true;
                }
                return false;
            }
        } );
    }

    public Action openClaw() {
        return new Action() {
            private boolean isInitialised = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!isInitialised) {
                    Claw.setPosition(clawOpenPos);
                    isInitialised = true;
                    return true;
                }
                return false;
            }
        };
    }

    public Action setArmPivot(double pos) {
        return new SetArmPivot(pos);
    }
    public Action setExtendoPos(double pos) {
        return new ExtendoController(pos);
    }
    public Action setExtendoPos(double pos, Action callback) {
        return new ExtendoController(pos, callback);
    }

    public class ExtendoController implements Action {
        private boolean initialised = false;
        private double setpoint = 0;
        private Action CallbackAction;

        public ExtendoController(double setpoint) {
            this.setpoint = setpoint;
        }
        public ExtendoController(double setpoint, Action callback) {
            this.setpoint = setpoint;
            this.CallbackAction = callback;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialised) {
                if (currentSetpoint != this.setpoint) {
                    currentSetpoint = this.setpoint;
                    ExtendoController.targetPosition = this.setpoint;

                }
                this.initialised = true;
                return true;
            }else {
                if (this.CallbackAction != null) {
                    if (Math.abs(ExtendoController.lastError) < 20) {
                        return this.CallbackAction.run(telemetryPacket);
                    }
                    return true;
                }
                return false;
            }
        }

    }

    public class SetArmPivot implements Action {

        private double pos = 0;
        public SetArmPivot(double pos) {
            this.pos = pos;
        }
        @Override
        public boolean run (@NonNull TelemetryPacket telemetryPacket) {
            ArmPivotRight.setPosition(this.pos);
            ArmPivotLeft.setPosition(this.pos);

            return false;
        }
    }
}

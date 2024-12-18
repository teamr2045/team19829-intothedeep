package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.Lifter;

@Autonomous
public class Auto_BlueAlliance extends LinearOpMode {
    private Servo ArmPivotLeft;
    private Servo ArmPivotRight;

    @Override
    public void runOpMode() {
        Pose2d initialPose =  new Pose2d(-24, 62, Math.toRadians(270));
        telemetry.addLine("Init Subsystem");
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Lifter lifter = new Lifter(hardwareMap);

        ArmPivotLeft = hardwareMap.get(Servo.class, "armPivotLeft");
        ArmPivotRight= hardwareMap.get(Servo.class, "armPivotRight");
        ArmPivotRight.setDirection(Servo.Direction.REVERSE);


        telemetry.addLine("Initialising Trajectories");
//        TrajectoryActionBuilder spcimentCycle =
        TrajectoryActionBuilder initialCycle = drive.actionBuilder(initialPose)
                .splineToLinearHeading(new Pose2d(-2, 29.5, Math.toRadians(90)), Math.toRadians(270)
                );
        TrajectoryActionBuilder goToOberservationZone_1 = initialCycle.endTrajectory().fresh()
                .splineTo(new Vector2d(-21.80, 40.90), Math.toRadians(180))
                .splineTo(new Vector2d(-32, 14.20), Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(-54.42, 7.94, Math.toRadians(270)), Math.toRadians(90)) // , 0.0
                .lineToY(61);
        TrajectoryActionBuilder goToSubmersible_fromOZone1 = goToOberservationZone_1.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(-5, 31, Math.toRadians(90)), Math.toRadians(270)
                );
        TrajectoryActionBuilder goToOberservationZone_2 = goToSubmersible_fromOZone1.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(-40, 55, Math.toRadians(270)), Math.toRadians(90))
                .lineToY(61);
        TrajectoryActionBuilder goToSubmersible_fromOZone2 = goToOberservationZone_2.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(-5, 31, Math.toRadians(90)), Math.toRadians(270)
                );

        telemetry.addLine("Build Trajectory : initialCycle_build");
        Action initialCycle_build = initialCycle.build();
        telemetry.addLine("Build Trajectory : goToOberservationZone_1_build");
        Action goToOberservationZone_1_build = goToOberservationZone_1.build();
        telemetry.addLine("Build Trajectory : goToSubmersible_fromOZone1_build");
        Action goToSubmersible_fromOZone1_build = goToSubmersible_fromOZone1.build();
        telemetry.addLine("Build Trajectory : goToOberservationZone_2_build");
        Action goToOberservationZone_2_build = goToOberservationZone_2.build();
        telemetry.addLine("Build Trajectory : goToSubmersible_fromOZone2_build");
        Action goToSubmersible_fromOZone2_build = goToSubmersible_fromOZone2.build();

        telemetry.addLine("All Trajectories Are Set Up!");


        ArmPivotRight.setPosition(0.4);
        ArmPivotLeft.setPosition(0.4);
        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(new SequentialAction(
                new ParallelAction(
                        lifter.setLifterPosition(1532), // go to the top of max truss
                        initialCycle_build
                ),
                lifter.releaseSpeciment()
//                new ParallelAction(
//                        lifter.setLifterPosition(0),
//                        goToOberservationZone_1_build
//                ),
//                lifter.grabSpeciment(),
//                new ParallelAction(
//                        lifter.setLifterPosition(1532), // go to the top of max truss
//                        goToSubmersible_fromOZone1_build
//                ),
//                lifter.releaseSpeciment(),
//
//                new ParallelAction(
//                        lifter.setLifterPosition(0),
//                        goToOberservationZone_2_build
//                ),
//                lifter.grabSpeciment(),
//                new ParallelAction(
//                        lifter.setLifterPosition(1532), // go to the top of max truss
//                        goToSubmersible_fromOZone2_build
//                ),
//                lifter.releaseSpeciment()

        ));

    }
}

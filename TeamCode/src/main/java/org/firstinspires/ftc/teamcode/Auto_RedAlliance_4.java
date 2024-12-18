package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Extendo;
import org.firstinspires.ftc.teamcode.subsystems.Lifter;

@Autonomous(name = "Auto_RedAlliance_4")
public class Auto_RedAlliance_4 extends LinearOpMode {

    private double lifterBucketPosition = 0;

    private ElapsedTime opModeTime = new ElapsedTime();

    private Lifter lifter;
    private Extendo extendo;

    private double grabSpecimentXPos = 45;
    private Pose2d bucketParkPos = new Pose2d(-60,-60,Math.toRadians(45));
    private double bucketTargetPos = 145;
    private double highBasketPosition = 2330;
    private double bucketIdlePos  = 0.1275;
    private double extendo__takeSample_pos = 150;

    private double clawRotatorPos_1_3 = 0.65;


    @Override
    public void runOpMode() {
        Pose2d initialPose =  new Pose2d(-24, -64, Math.toRadians(90));
        Pose2d grabSpecimentPos =  new Pose2d(grabSpecimentXPos, -64, Math.toRadians(270));

        telemetry.addLine("Init Subsystem");
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        lifter = new Lifter(hardwareMap);
        extendo = new Extendo(hardwareMap);

        telemetry.addLine("Initialising Trajectories");
        TrajectoryActionBuilder initialCycle = drive.actionBuilder(initialPose)
                .lineToYLinearHeading(-60, Math.toRadians(90))
                .splineTo(new Vector2d(-45.93, -46.27), Math.toRadians(50))
                .strafeTo(new Vector2d(-60, -60)) // go to basket

                .endTrajectory();
        TrajectoryActionBuilder takeOneSample_1 = initialCycle.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(-48.5, -42.01, Math.toRadians(90)), Math.toRadians(90.00))

                // extendo claw Action
                .afterTime(0, new ParallelAction(extendo.takeSample()))
                .waitSeconds(0.5)
                .afterTime(0, new ParallelAction(extendo.resetArmToIdle()))
                .afterTime(0.4, new ParallelAction(extendo.setExtendoPos(0, extendo.openClaw())))
                .afterTime(1.4, new ParallelAction(extendo.setArmPivot(0.7)))
                .afterTime(1.4, new ParallelAction(lifter.setLifterPosition_v2(highBasketPosition)))

                .afterTime(1.4, new ParallelAction(lifter.resetBucket_toClose()))

                // return to basket

                .splineToLinearHeading(bucketParkPos, Math.toRadians(225))
                ;

        TrajectoryActionBuilder takeOneSample_2 = takeOneSample_1.endTrajectory().fresh()
                // get one sample
                .afterTime(0, new ParallelAction(extendo.setArmPivot(0.85)))
                .splineToLinearHeading(new Pose2d(-62.25, -35.5, Math.toRadians(90)), Math.toRadians(90.00))
                // extendo claw Action (insert code here)
                .afterTime(0, new ParallelAction(extendo.takeSample()))
                .waitSeconds(0.5)
                .afterTime(0, new ParallelAction(extendo.resetArmToIdle()))
                .afterTime(0.4, new ParallelAction(extendo.setExtendoPos(0, extendo.openClaw())))
                .afterTime(1.2, new ParallelAction(extendo.setArmPivot(0.7)))
                .afterTime(1.2, new ParallelAction(lifter.setLifterPosition_v2(highBasketPosition)))

                // return to basket
                .afterTime(1.2, new ParallelAction(lifter.resetBucket_toClose()))

                .splineToLinearHeading(new Pose2d(-61, -61, Math.toRadians(45)), Math.toRadians(225))
                ;
        TrajectoryActionBuilder takeOneSample_3 = takeOneSample_2.endTrajectory().fresh()
                // get one sample
//                .afterTime(0, new ParallelAction(extendo.setArmPivot(0.85)))
                .afterTime(0, new ParallelAction(extendo.setArmPivot(0.85)))

                .strafeToConstantHeading(new Vector2d(-43.54, -42.86))
                .splineToLinearHeading(new Pose2d(-55.5, -30, Math.toRadians(180)), Math.toRadians(180))

                // extendo claw Action (insert code here)
                .afterTime(0, new ParallelAction(extendo.takeSample()))
                .waitSeconds(0.65)
                .afterTime(0, new ParallelAction(extendo.resetArmToIdle()))
                .afterTime(0.4, new ParallelAction(extendo.setExtendoPos(0, extendo.openClaw())))
                .afterTime(0.7, new ParallelAction(extendo.setArmPivot(0.7)))
                .afterTime(0.7, new ParallelAction(lifter.setLifterPosition_v2(highBasketPosition)))

                .afterTime(0.7, new ParallelAction(lifter.resetBucket_toClose()))

                .strafeToConstantHeading(new Vector2d(-50.5,  -27.6) )

//                .splineTo(new Vector2d(-46.48, -38.87), Math.toRadians(270-45))
//                .splineTo(new Vector2d(-59, -59), Math.toRadians(270-45))

                // return to basket
//                .afterTime(1, new ParallelAction(lifter.setLifterPosition_v2(highBasketPosition)))
//                .afterTime(1.2, new ParallelAction(lifter.flipBucket()))

                .splineToLinearHeading(new Pose2d(-60.6, -60.6, Math.toRadians(45)), Math.toRadians(225.00))
                .waitSeconds(0.8)

                .afterTime(0, new ParallelAction(lifter.flipBucket()))
                .waitSeconds(0.8)
                .afterTime(0, new ParallelAction(lifter.resetBucket_toOpen()))
                .strafeToConstantHeading(new Vector2d(-58, -58))

                .afterTime(0, new ParallelAction(lifter.setLifterPosition_v2(0, extendo.setArmPivot(0.32))))
                .afterTime(0, new ParallelAction(extendo.setExtendoPos(0)))

                ;
        TrajectoryActionBuilder park_lol = takeOneSample_3.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(-58, -58));


        telemetry.addLine("Build Trajectory : initialCycle_build");
        Action initialCycle_build = initialCycle.build();
        telemetry.addLine("Build Trajectory : goToOberservationZone_1_build");
        Action takeOneSample_1_build = takeOneSample_1.build();
        telemetry.addLine("Build Trajectory : goToSubmersible_fromOZone1_build");
        Action takeOneSample_2_build = takeOneSample_2.build();
        telemetry.addLine("Build Trajectory : goToOberservationZone_2_build");
        Action takeOneSample_3_build = takeOneSample_3.build();
        Action parklol_build = park_lol.build();

//        telemetry.addLine("Build Trajectory : goToSubmersible_fromOZone2_build");
//        Action goToSubmersible_fromOZone2_build = goToSubmersible_fromOZone2.build();

        telemetry.addLine("All Trajectories Are Set Up!");
        telemetry.update();

        Actions.runBlocking(new ParallelAction(
                extendo.setArmPivot(0.32),
                lifter.grabSpeciment()
        ));

        waitForStart();

        if (isStopRequested()) return;

        Action trajectoryAction = new SequentialAction(
                new ParallelAction(
                        extendo.setArmPivot(0.7),
                // extend extendo (insert code here)
                        initialCycle_build,
                        lifter.setLifterPosition_v2(highBasketPosition),
                        lifter.resetBucket_toClose(),
                        extendo.setExtendoPos(22.5),
                        extendo.openClaw()

//                        new SequentialAction(
//                                new SleepAction(1),
//                                lifter.flipBucket()
//                        )
                ),
                // open bucket door (insert code here) (at basket)
                new SleepAction(0.5),
                lifter.flipBucket(),
                new SleepAction(1),
                lifter.resetBucket_toOpen(),

                new ParallelAction(
                        // extend extendo
                        extendo.setArmPivot(0.85),
                        new SequentialAction(
                                new SleepAction(0.5),
                                lifter.setLifterPosition_v2(0)
                        ),
                        takeOneSample_1_build
                ),
                new SleepAction(0.9),
                lifter.flipBucket(), // (at basket)
                new SleepAction(1.2),
                extendo.setExtendoPos(0),

                new ParallelAction(
                        lifter.resetBucket_toOpen(),
                        // extend extendo
                        new SequentialAction(
                                new SleepAction(0.5),
                                lifter.setLifterPosition_v2(0)
                        ),
                        takeOneSample_2_build
                ),
                new SleepAction(0.6),
                lifter.flipBucket(),
                new SleepAction(1.1),
                extendo.setExtendoPos(135.5),

                new ParallelAction(
                        extendo.setClawRotatorPos(clawRotatorPos_1_3),
                        extendo.setArmPivot(0.7),
                        lifter.resetBucket_toOpen(),
                        new SequentialAction(
                                new SleepAction(0.5),
                                lifter.setLifterPosition_v2(0)
                        ),
                        takeOneSample_3_build
                )

//                new SleepAction(0.8),
//                lifter.flipBucket(),
//                new SleepAction(1),
//                new ParallelAction(
//                        new SequentialAction(
//                                new SleepAction(0.5),
//                                lifter.setLifterPosition(0)
//                        ),
//                        extendo.setExtendoPos(0),
//                        parklol_build
//                )




//                new ParallelAction(
//                        lifter.grabSpeciment()
//                ),
//                new ParallelAction(
//                        initialCycle_build,// go to the top of max truss,
//                        lifter.setLifterPosition_v2(highBasketPosition)
//
////                        lifter.setArmPivot(0.5)
//                        // extend extendo (insert code here)
//                ),
//                lifter.openBucket(),
//
//                // open bucket door (insert code here)
//                new SleepAction(0.4)

//                new ParallelAction(
//                        // extend extendo
//                        lifter.setLifterPosition_v2(0),
//                        takeOneSample_1_build
//                ),
//                // flip bucket (insert code)
//                lifter.openBucket(),
//
//                // open bucket door (insert code here)
//                new SleepAction(0.4),
//                // open bucket door (insert code here)
//
//                // lift down (insert code)
//
//                new ParallelAction(
//                        // extend extendo (for sample 2) (insert code here)
//                        lifter.setLifterPosition_v2(0),
//                        takeOneSample_2_build
//                ),
//                // flip bucket (insert code)
//                lifter.openBucket(),
//
//                // open bucket door (insert code here)
//                new SleepAction(0.4),
//
//                // sleepAction here
//
//                new ParallelAction(
//                        // extend extendo (for sample 2) (insert code here)
//
//                        takeOneSample_3_build
//                )


//

//                goToSubmersible_fromOZone2_build
//                lifter.releaseSpeciment_v2()
//                goToOberservationZone_2_build,
//                lifter.grabSpeciment()
        );

        while (opModeIsActive()) {
            opModeTime.reset();
            runBlocking(trajectoryAction);
        }
    }

    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    }


    public void runBlocking(Action action) {
        FtcDashboard dash = FtcDashboard.getInstance();
        Canvas previewCanvas = new Canvas();
        action.preview(previewCanvas);

        boolean running = true;
        while (running && !Thread.currentThread().isInterrupted()) {
            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().getOperations().addAll(previewCanvas.getOperations());
            packet.put("time", opModeTime);

            running = action.run(packet);

            extendo.runAuto();
            lifter.runAuto();
            extendo.sendTelemetryAuto(packet);
            lifter.sendTelemetryAuto(packet);

            dash.sendTelemetryPacket(packet);
        }
    }


}
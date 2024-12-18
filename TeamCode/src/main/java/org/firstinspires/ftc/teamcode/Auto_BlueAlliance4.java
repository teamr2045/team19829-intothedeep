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

@Autonomous(name = "Auto_BlueAlliance_4")

public class Auto_BlueAlliance4 extends LinearOpMode {

    private double lifterBucketPosition = 0;

    private ElapsedTime opModeTime = new ElapsedTime();

    private Lifter lifter;
    private Extendo extendo;

    private double grabSpecimentXPos = 45;
    private Pose2d bucketParkPos = new Pose2d(59.66,60,Math.toRadians(270 - 50.5));
    private double bucketTargetPos = 145;
    private double highBasketPosition = 1925;

    @Override
    public void runOpMode() {
        Pose2d initialPose =  new Pose2d(24, 64, Math.toRadians(270));
        Pose2d grabSpecimentPos =  new Pose2d(grabSpecimentXPos, -64, Math.toRadians(270));

        telemetry.addLine("Init Subsystem");
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        lifter = new Lifter(hardwareMap);
        extendo = new Extendo(hardwareMap);

        double redAllianceDegree = 41.79;

        telemetry.addLine("Initialising Trajectories");
        TrajectoryActionBuilder initialCycle = drive.actionBuilder(initialPose)
                .lineToYLinearHeading(60, Math.toRadians(90))
                .splineTo(new Vector2d(45.93, 46.27), Math.toRadians(270 - redAllianceDegree))
                .strafeTo(new Vector2d(59.39, 60)) // go to basket
                .endTrajectory();
        TrajectoryActionBuilder takeOneSample_1 = initialCycle.endTrajectory().fresh()
                // get one sample
                .splineToLinearHeading(new Pose2d(49.51, 42.01, Math.toRadians(270)), Math.toRadians(270))

                // extendo claw Action
                .afterTime(0, new ParallelAction(extendo.takeSample()))
                .waitSeconds(0.65)
                .afterTime(0, new ParallelAction(extendo.resetArmToIdle()))
                .afterTime(0.6, new ParallelAction(extendo.setExtendoPos(0, extendo.openClaw())))
                .afterTime(1.2, new ParallelAction(extendo.setArmPivot(0.7)))
                .afterTime(1.6, new ParallelAction(lifter.setLifterPosition_v2(highBasketPosition)))

                .afterTime(1.6, new ParallelAction(lifter.resetBucket_toClose()))
                .afterTime(1.65, new ParallelAction(lifter.flipBucket()))

                // return to basket
                .splineToLinearHeading(new Pose2d(59.39, 60, Math.toRadians(270 - redAllianceDegree)), Math.toRadians(270-225))
                ;

        TrajectoryActionBuilder takeOneSample_2 = takeOneSample_1.endTrajectory().fresh()
                // get one sample
                .splineToLinearHeading(new Pose2d(57.51, 42.01, Math.toRadians(270)), Math.toRadians(270))
                // extendo claw Action (insert code here)
                .afterTime(0, new ParallelAction(extendo.takeSample()))
                .waitSeconds(0.65)
                .afterTime(0, new ParallelAction(extendo.resetArmToIdle()))
                .afterTime(0.6, new ParallelAction(extendo.setExtendoPos(0, extendo.openClaw())))
                .afterTime(1.2, new ParallelAction(extendo.setArmPivot(0.7)))
                .afterTime(1.5, new ParallelAction(lifter.resetBucket_toClose()))

                // return to basket
                .afterTime(1.0, new ParallelAction(lifter.setLifterPosition_v2(highBasketPosition)))
                .afterTime(1.45, new ParallelAction(lifter.flipBucket()))

                .splineToLinearHeading(new Pose2d(59.39, 60, Math.toRadians(270 - redAllianceDegree)), Math.toRadians(270-225))
                ;
        TrajectoryActionBuilder takeOneSample_3 = takeOneSample_2.endTrajectory().fresh()
                // get one sample
                .splineTo(new Vector2d(-49, -25.31), Math.toRadians(180))
                // extendo claw Action (insert code here)

                // return to basket
                .afterTime(0, new ParallelAction(lifter.setLifterPosition(lifterBucketPosition)))
                .splineToLinearHeading(bucketParkPos, Math.toRadians(225.00))
                ;


        telemetry.addLine("Build Trajectory : initialCycle_build");
        Action initialCycle_build = initialCycle.build();
        telemetry.addLine("Build Trajectory : goToOberservationZone_1_build");
        Action takeOneSample_1_build = takeOneSample_1.build();
        telemetry.addLine("Build Trajectory : goToSubmersible_fromOZone1_build");
        Action takeOneSample_2_build = takeOneSample_2.build();
        telemetry.addLine("Build Trajectory : goToOberservationZone_2_build");
        Action takeOneSample_3_build = takeOneSample_3.build();
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
                        // extend extendo (insert code here)
                        initialCycle_build,
                        lifter.setLifterPosition_v2(highBasketPosition),
                        lifter.resetBucket_toClose(),
                        extendo.setExtendoPos(140),
                        extendo.setArmPivot(0.85),
                        new SequentialAction(
                                new SleepAction(1),
                                lifter.flipBucket()
                        )
                ),
                // open bucket door (insert code here) (at basket)
                lifter.openBucket(),
                new SleepAction(2.2),
                lifter.resetBucket_toOpen(),

                new ParallelAction(
                        // extend extendo
                        lifter.setLifterPosition_v2(0),
                        takeOneSample_1_build
                ),
                lifter.openBucket(), // (at basket)
                new SleepAction(2.2),
                lifter.resetBucket_toOpen(),

                new ParallelAction(
                        // extend extendo
                        lifter.setLifterPosition_v2(0),
                        takeOneSample_2_build
                ),
                lifter.openBucket(),
                new SleepAction(2.2),
                lifter.resetBucket_toOpen()

                // flip bucket (insert code)

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
            lifter.sendTelemetryAuto(packet);
            dash.sendTelemetryPacket(packet);
        }
    }


}
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

@Autonomous(name = "Auto_RedAlliance_4.4")
public class Auto_RedAlliance_4_4 extends LinearOpMode {

    private double lifterBucketPosition = 0;

    private ElapsedTime opModeTime = new ElapsedTime();

    private Lifter lifter;
    private Extendo extendo;

    private double grabSpecimentXPos = 45;


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
                .splineTo(new Vector2d(-45.93, -46.27), Math.toRadians(40.91))
                .strafeTo(new Vector2d(-60, -60)) // go to basket
                .endTrajectory();
        TrajectoryActionBuilder takeOneSample_1 = initialCycle.endTrajectory().fresh()
                // get one sample
                .splineToLinearHeading(new Pose2d(-49.51, -42.01, Math.toRadians(90)), Math.toRadians(90.00))

                // extendo claw Action

                // return to basket
                .afterTime(0, new ParallelAction(lifter.setLifterPosition(lifterBucketPosition)))
                .splineToLinearHeading(new Pose2d(-60, -60, Math.toRadians(40.91)), Math.toRadians(225))
                ;
        TrajectoryActionBuilder takeOneSample_2 = takeOneSample_1.endTrajectory().fresh()
                // get one sample
                .splineToLinearHeading(new Pose2d(-57.51, -42.01, Math.toRadians(90)), Math.toRadians(90.00))
                // extendo claw Action (insert code here)

                // return to basket
                .afterTime(0, new ParallelAction(lifter.setLifterPosition(lifterBucketPosition)))
                .splineToLinearHeading(new Pose2d(-50, -60, Math.toRadians(40.91)), Math.toRadians(225))
                ;
        TrajectoryActionBuilder takeOneSample_3 = takeOneSample_2.endTrajectory().fresh()
                // get one sample
                .splineTo(new Vector2d(-49, -25.31), Math.toRadians(180))
                // extendo claw Action (insert code here)

                // return to basket
                .afterTime(0, new ParallelAction(lifter.setLifterPosition(lifterBucketPosition)))
                .splineToLinearHeading(new Pose2d(-60, -60, Math.toRadians(40.91)), Math.toRadians(225.00))
                ;
        TrajectoryActionBuilder goToNetZone = takeOneSample_3.endTrajectory().fresh()
                .splineTo(new Vector2d(-25.82, -36.89), Math.toRadians(1.35))
                .splineTo(new Vector2d(45, -37.07), Math.toRadians(-0.14))

                // park at netzone
                .splineToLinearHeading(new Pose2d(55, -58, Math.toRadians(90.00)), Math.toRadians(270.00))
                .strafeTo(new Vector2d(55, -64))
                ;

        telemetry.addLine("Build Trajectory : initialCycle_build");
        Action initialCycle_build = initialCycle.build();
        telemetry.addLine("Build Trajectory : goToOberservationZone_1_build");
        Action takeOneSample_1_build = takeOneSample_1.build();
        telemetry.addLine("Build Trajectory : goToSubmersible_fromOZone1_build");
        Action takeOneSample_2_build = takeOneSample_2.build();
        telemetry.addLine("Build Trajectory : goToOberservationZone_2_build");
        Action takeOneSample_3_build = takeOneSample_3.build();
        Action goToNetZone_build = goToNetZone.build();

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
                        lifter.grabSpeciment()
                ),
                new ParallelAction(
                        initialCycle_build// go to the top of max truss,
//                        lifter.setLifterPosition_v2(2180),
//                        lifter.setArmPivot(0.5)
                        // extend extendo (insert code here)

                ),
                // open bucket door (insert code here)
                new SleepAction(0.5),

                new ParallelAction(
                        // extend extendo
                        takeOneSample_1_build
                ),
                new SleepAction(0.5),
                // flip bucket (insert code)
                // open bucket door (insert code here)

                // lift down (insert code)

                new ParallelAction(
                        // extend extendo (for sample 2) (insert code here)
                        takeOneSample_2_build
                ),
                new SleepAction(0.5),

                // flip bucket (insert code)

                // sleepAction here

                new ParallelAction(
                        // extend extendo (for sample 2) (insert code here)

                        takeOneSample_3_build
                ),
                new SleepAction(0.5),
                goToNetZone_build



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
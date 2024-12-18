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

@Autonomous(name = "Auto_RedAlliance_4.1")
public class Auto_RedAlliance_4_1 extends LinearOpMode {

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
                .splineToLinearHeading(new Pose2d(-48.9, -42.01, Math.toRadians(90)), Math.toRadians(90.00))

                // extendo claw Action
                .afterTime(0, new ParallelAction(extendo.takeSample()))
                .waitSeconds(0.5)
                .afterTime(0, new ParallelAction(extendo.resetArmToIdle()))
                .afterTime(0.6, new ParallelAction(extendo.setExtendoPos(0, extendo.openClaw())))
                .afterTime(1.2, new ParallelAction(extendo.setArmPivot(0.7)))
                .afterTime(1.2, new ParallelAction(lifter.resetBucket_toClose()))


//                // return to basket
////                .afterTime(0, new ParallelAction(lifter.setLifterPosition(lifterBucketPosition)))
//                .splineToLinearHeading(new Pose2d(-59.39, -60, Math.toRadians(40.91)), Math.toRadians(225))
                ;

        telemetry.addLine("Build Trajectory : initialCycle_build");
        Action initialCycle_build = initialCycle.build();
        telemetry.addLine("Build Trajectory : goToOberservationZone_1_build");
        Action takeOneSample_1_build = takeOneSample_1.build();

        telemetry.addLine("All Trajectories Are Set Up!");
        telemetry.update();

        Actions.runBlocking(new ParallelAction(
                extendo.setArmPivot(0.5),
                lifter.grabSpeciment(),
                lifter.resetBucket_toOpen()
        ));

        waitForStart();

        if (isStopRequested()) return;

        Action trajectoryAction = new SequentialAction(

                new ParallelAction(
                        // extend extendo (insert code here)
                        initialCycle_build,
                        extendo.setExtendoPos(155),
                        extendo.setArmPivot(0.85),
                        extendo.openClaw()
                ),
                // open bucket door (insert code here)
                new SleepAction(0.5),

                new ParallelAction(
                        // extend extendo
                        takeOneSample_1_build
//                        lifter.setLifterPosition_v2(0)
                )

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
            extendo.sendTelemetryAuto(packet);
            lifter.runAuto();
//            lifter.sendTelemetryAuto(packet);
            dash.sendTelemetryPacket(packet);
        }
    }


}

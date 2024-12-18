package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Extendo;
import org.firstinspires.ftc.teamcode.subsystems.Lifter;

@Autonomous
public class Auto_BlueAlliance2 extends LinearOpMode {

    private ElapsedTime opModeTime = new ElapsedTime();

    private Lifter lifter;
    private Extendo extendo;

    @Override
    public void runOpMode() {
        Pose2d initialPose =  new Pose2d(-23.5, 64, Math.toRadians(90 ));
        telemetry.addLine("Init Subsystem");
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        lifter = new Lifter(hardwareMap);
        extendo = new Extendo(hardwareMap);


        telemetry.addLine("Initialising Trajectories");
        TrajectoryActionBuilder initialCycle = drive.actionBuilder(initialPose)
                .lineToYLinearHeading(60, Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-3, 29.5 , Math.toRadians(90)), Math.toRadians(270))
                .endTrajectory();
        TrajectoryActionBuilder goToOberservationZone_1 = initialCycle.endTrajectory().fresh()
                .lineToY(37)
                .splineTo(new Vector2d(-31.44, 36.51), Math.toRadians(178.36))
                .splineTo(new Vector2d(-38.70, 27.72), Math.toRadians(-87.40))
                .splineTo(new Vector2d(-45, 5), Math.toRadians(270.00))

                .strafeToLinearHeading(new Vector2d(-45,  5.58), Math.toRadians(270))

                .afterTime(0, new ParallelAction(lifter.setLifterPosition_v2(0)))

                .strafeToConstantHeading(new Vector2d(-47,  -55) )
                .strafeToConstantHeading(new Vector2d(-47,  -40))
                .strafeToConstantHeading(new Vector2d(-47,  -65));

        TrajectoryActionBuilder goToSubmersible_fromOZone1 = goToOberservationZone_1.endTrajectory().fresh()
                .waitSeconds(0.5)
                .strafeToLinearHeading(new Vector2d(-40,  58), Math.toRadians(90))

                .afterTime(0, new ParallelAction(lifter.setLifterPosition_v2(1300)))
                .splineToLinearHeading(new Pose2d(-1.25, 29, Math.toRadians(90)), Math.toRadians(270)
                );
        TrajectoryActionBuilder goToOberservationZone_2 = goToSubmersible_fromOZone1.endTrajectory().fresh()
                .lineToY(38)
                .afterTime(0, new ParallelAction(lifter.setLifterPosition_v2(0)))
                .splineToLinearHeading(new Pose2d(-55, 55, Math.toRadians(270)), Math.toRadians(90))
                .lineToYLinearHeading(58, Math.toRadians(270))
                .lineToYLinearHeading(65, Math.toRadians(270));

        TrajectoryActionBuilder goToSubmersible_fromOZone2 = goToOberservationZone_2.endTrajectory().fresh()
                .waitSeconds(0.5)
                .strafeToLinearHeading(new Vector2d(-50,  58), Math.toRadians(90))
                .afterTime(0, new ParallelAction(lifter.setLifterPosition_v2(1300   )))
                .splineToLinearHeading(new Pose2d(-1, 29, Math.toRadians(270)), Math.toRadians(90)
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
                        initialCycle_build,// go to the top of max truss,
                        lifter.setLifterPosition_v2(1300),
                        extendo.setArmPivot(0.5)
                ),
                lifter.releaseSpeciment_v2(),
                goToOberservationZone_1_build,

                lifter.grabSpeciment(),
                goToSubmersible_fromOZone1_build,
                lifter.releaseSpeciment_v2(),
                goToOberservationZone_2_build,
                lifter.grabSpeciment()
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

            lifter.runAuto();
            lifter.sendTelemetryAuto(packet);
            dash.sendTelemetryPacket(packet);
        }
    }



}

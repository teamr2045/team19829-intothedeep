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
public class Auto_RedAlliance3 extends LinearOpMode {

    private ElapsedTime opModeTime = new ElapsedTime();

    private Lifter lifter;
    private Extendo extendo;

    @Override
    public void runOpMode() {
        Pose2d initialPose =  new Pose2d(23.5, -64, Math.toRadians(270));
        Pose2d grabSpecimentPos =  new Pose2d(50, -64, Math.toRadians(270));

        telemetry.addLine("Init Subsystem");
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        lifter = new Lifter(hardwareMap);
        extendo = new Extendo(hardwareMap);

        telemetry.addLine("Initialising Trajectories");
        TrajectoryActionBuilder initialCycle = drive.actionBuilder(initialPose)
                .lineToYLinearHeading(-60, Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(3, -29.5, Math.toRadians(270.00)), Math.toRadians(90))
                .endTrajectory();
        TrajectoryActionBuilder goToOberservationZone_1 = initialCycle.endTrajectory().fresh()
                .lineToY(-31)
                .afterTime(0, new ParallelAction(lifter.setLifterPosition_v2(400)))
                .splineToLinearHeading(new Pose2d(37.24, -32, Math.toRadians(270.00)), Math.toRadians(90))
                .strafeTo(new Vector2d(37.24, -5))
                .splineToConstantHeading(new Vector2d(45, -15.25), Math.toRadians(270.00))
                .strafeToLinearHeading(new Vector2d(45, -55), Math.toRadians(270))

                .strafeToLinearHeading(new Vector2d(45, -50), Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(45, -10, Math.toRadians(90)), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(45, -5), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(55, -15.25), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(55, -55), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(45, -55), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(45, -64), Math.toRadians(90));
        TrajectoryActionBuilder goToSubmersible_fromOZone1 = goToOberservationZone_1.endTrajectory().fresh()
                .waitSeconds(0.5)
                .afterTime(0, new ParallelAction(lifter.setLifterPosition_v2(1750)))
                .strafeToLinearHeading(new Vector2d(40,  -58), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(2, -29.5, Math.toRadians(270.00)), Math.toRadians(90));

        TrajectoryActionBuilder goToOberservationZone_2 = goToSubmersible_fromOZone1.endTrajectory().fresh()
                .lineToY(-32)
                .afterTime(0, new ParallelAction(lifter.setLifterPosition_v2(0)))
                .splineToLinearHeading(new Pose2d(50, -55, Math.toRadians(90.00)), Math.toRadians(270))
                .lineToYLinearHeading(-58, Math.toRadians(90))
                .lineToYLinearHeading(-64, Math.toRadians(90));

        TrajectoryActionBuilder goToSubmersible_fromOZone2 = drive.actionBuilder(grabSpecimentPos)
                .waitSeconds(0.5)
                .strafeToLinearHeading(new Vector2d(50,  -58), Math.toRadians(90))
                .afterTime(0, new ParallelAction(lifter.setLifterPosition_v2(1780)))
                .splineToLinearHeading(new Pose2d(2, -29.5, Math.toRadians(90)), Math.toRadians(270));

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
                        lifter.setLifterPosition_v2(1800),
                        extendo.setArmPivot(0.5)
                ),
//                new Ti
                lifter.releaseSpeciment_v2(),
                goToOberservationZone_1_build,
                lifter.grabSpeciment(),

                goToSubmersible_fromOZone1_build,
                lifter.releaseSpeciment_v2(),

                goToOberservationZone_2_build,
                lifter.grabSpeciment(),

                goToSubmersible_fromOZone2_build,
                lifter.releaseSpeciment_v2(),
                goToOberservationZone_2_build,
                lifter.grabSpeciment()

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

            lifter.runAuto();
            extendo.runAuto();
            lifter.sendTelemetryAuto(packet);
            dash.sendTelemetryPacket(packet);
        }
    }
}

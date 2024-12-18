package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.core.colorscheme.ColorScheme;
import org.rowlandhall.meepmeep.core.entity.BotEntity;
import org.rowlandhall.meepmeep.roadrunner.Constraints;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.DriveTrainType;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import org.rowlandhall.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;
import org.rowlandhall.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

import java.awt.Color;

class BotBuilder {
    private MeepMeep meepMeep;

    /** The constraints for the bot's trajectory. */
    private Constraints constraints = new Constraints(30.0, 30.0, Math.toRadians(60), Math.toRadians(60), 15);
    /** The width of the bot. */
    private double width = 18.0;

    /** The height of the bot. */
    private double height = 18.0;

    /** The starting pose of the bot. */
    private Pose2d startPose = new Pose2d();

    /** The color scheme of the bot. */
    private ColorScheme colorScheme = null;

    /** The opacity of the bot. */
    private double opacity = 0.8;


    /** The drive train type of the bot. */
    private DriveTrainType driveTrainType = DriveTrainType.MECANUM;


    public BotBuilder(MeepMeep meepMeep, double width, double height, Pose2d startPose) {
        this.meepMeep = meepMeep;
    }


}
public class MeepMeepTesting {
    private double redAllianceDegree = 40.91;

    public static void main(String[] args) {


        double redAllianceDegree = 40.91;

        MeepMeep meepMeep = new MeepMeep(800);

//        RoadRunnerBotEntity redBot2 = new DefaultBotBuilder(meepMeep)
//                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
//                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-24, -61.52, Math.toRadians(270)))
//                        .back(2)
////                        .splineTo(new Vector2d(-1.52, -32), 0)
//                        .splineToLinearHeading(new Pose2d(-1.52, -30.5, Math.toRadians(270.00)), Math.toRadians(90))
//                        .splineTo(new Vector2d(21.80, -40.90), Math.toRadians(0))
//                        .splineTo(new Vector2d(32, -14.20), Math.toRadians(90.00))
//                        .splineToLinearHeading(new Pose2d(54.42, -7.94, Math.toRadians(90.00)), Math.toRadians(270)) // , 0.0
//                        .back(53)
//                        .splineToLinearHeading(new Pose2d(5, -31, Math.toRadians(270.00)), Math.toRadians(90.00))
//                        .splineToLinearHeading(new Pose2d(40, -55, Math.toRadians(90.00)), Math.toRadians(270.00))
//                        .back(6)
////                        .splineToLinearHeading(new Pose2d(6.08, -30, Math.toRadians(270.00)), Math.toRadians(90.00))
////                        .splineToLinearHeading(new Pose2d(54.42, -55, Math.toRadians(90.00)), Math.toRadians(270.00))
////                        .back(6)
////                        .splineToLinearHeading(new Pose2d(6.08, -30, Math.toRadians(270.00)), Math.toRadians(90.00))
////                        .splineToLinearHeading(new Pose2d(54.42, -55, Math.toRadians(90.00)), Math.toRadians(270.00))
////                        .back(6)
//
//                        .build());

        Constraints constraints = new Constraints(30.0, 30.0, Math.toRadians(60), Math.toRadians(60), 15);
//        TrajectoryVelocityConstraint TrajectoryVelocityConstraint;
//        Pose2d robotPos = new Pose2d(24, -61.52, Math.toRadians(270));
//        TrajectorySequence trajectory;

//        DefaultBotBuilder
        RoadRunnerBotEntity redBot3 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(23.5, -64, Math.toRadians(270)))
                .back(2)
//               .splineTo(new Vector2d(-1.52, -32), 0)
                .splineToLinearHeading(new Pose2d(3, -30.5, Math.toRadians(270.00)), Math.toRadians(90))
                .strafeTo(new Vector2d(3, -32))
                .splineToLinearHeading(new Pose2d(37.24, -32, Math.toRadians(270.00)), Math.toRadians(90))
                .strafeTo(new Vector2d(37.24, -5))
                .splineToConstantHeading(new Vector2d(45, -15.25), Math.toRadians(270.00))
                .strafeTo(new Vector2d(45, -55))
                .strafeTo(new Vector2d(45, -50))
                .splineToLinearHeading(new Pose2d(45, -10, Math.toRadians(90)), Math.toRadians(90))
                .strafeTo(new Vector2d(45, -5))
                .splineToConstantHeading(new Vector2d(55, -15.25), Math.toRadians(90))
                .strafeTo(new Vector2d(55, -55))
                .strafeTo(new Vector2d(45, -55))
                .strafeTo(new Vector2d(45, -62.5))
                .strafeTo(new Vector2d(45, -58))
                .splineToLinearHeading(new Pose2d(3, -30.5, Math.toRadians(270.00)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(54.42, -55, Math.toRadians(90.00)), Math.toRadians(270.00))
                .strafeTo(new Vector2d(54.42, -64))
                .splineToLinearHeading(new Pose2d(3, -30, Math.toRadians(270.00)), Math.toRadians(90.00))
                        .splineToLinearHeading(new Pose2d(54.42, -55, Math.toRadians(90.00)), Math.toRadians(270.00))
                        .strafeTo(new Vector2d(54.42, -64))
                .build());

        RoadRunnerBotEntity redBot4 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-24, -62, Math.toRadians(90)))

                        .splineTo(new Vector2d(-45.93, -46.27), Math.toRadians(40.91))
                        .strafeTo(new Vector2d(-59.39, -60)) // go to basket


                        // get one sample
                        .splineToLinearHeading(new Pose2d(-49.51, -42.01, Math.toRadians(90)), Math.toRadians(90.00))

                        // return to basket
                        .splineToLinearHeading(new Pose2d(-59.39, -60, Math.toRadians(40.91)), Math.toRadians(225))

                        // get one sample
                        .splineToLinearHeading(new Pose2d(-57.51, -42.01, Math.toRadians(90)), Math.toRadians(90.00))

                        // return to basket
                        .splineToLinearHeading(new Pose2d(-59.39, -60, Math.toRadians(40.91)), Math.toRadians(225))

                        // get one sample
                        .splineTo(new Vector2d(-49, -25.31), Math.toRadians(180))

                        // return to basket
                        .splineToLinearHeading(new Pose2d(-60.58, -59.90, Math.toRadians(40.91)), Math.toRadians(225.00))

                        // go to netzone
                        .splineTo(new Vector2d(-25.82, -36.89), Math.toRadians(1.35))
                        .splineTo(new Vector2d(45, -37.07), Math.toRadians(-0.14))

                        // park at netzone
                        .splineToLinearHeading(new Pose2d(55, -58, Math.toRadians(90.00)), Math.toRadians(270.00))
                        .strafeTo(new Vector2d(55, -62))

                        // grab speciment
                        // <code here>

                        .splineToLinearHeading(new Pose2d(45.24, -38.6, Math.toRadians(0)), Math.toRadians(90))
                        .strafeTo(new Vector2d(44, -38.6))
                        .splineTo(new Vector2d(4.86, -29), Math.toRadians(93.24))


                        .build());

        RoadRunnerBotEntity bluebot4 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(24, 62, Math.toRadians(270)))

                        .splineTo(new Vector2d(45.93, 46.27), Math.toRadians(270 - redAllianceDegree))
                        .strafeTo(new Vector2d(59.39, 60)) // go to basket


                        // get one sample
                        .splineToLinearHeading(new Pose2d(49.51, 42.01, Math.toRadians(270)), Math.toRadians(270))

                        // return to basket
                        .splineToLinearHeading(new Pose2d(59.39, 60, Math.toRadians(270 - redAllianceDegree)), Math.toRadians(270-225))

                        // get one sample
                        .splineToLinearHeading(new Pose2d(57.51, 42.01, Math.toRadians(270)), Math.toRadians(270))

                        // return to basket
                        .splineToLinearHeading(new Pose2d(59.39, 60, Math.toRadians(270 - redAllianceDegree)), Math.toRadians(270-225))
//
                        // get one sample
                        .splineTo(new Vector2d(49, 25.31), Math.toRadians(0))

                        // return to basket
                        .splineToLinearHeading(new Pose2d(60.58, 59.90, Math.toRadians(270 - redAllianceDegree)), Math.toRadians(270-225))

                        // go to netzone
                        .splineTo(new Vector2d(-25.82, 36.89), Math.toRadians(180))
                        .splineTo(new Vector2d(-45, 37.07), Math.toRadians(180))
//
//                        // park at netzone
//                        .splineToLinearHeading(new Pose2d(-55, 58, Math.toRadians(270)), Math.toRadians(90))
//                        .strafeTo(new Vector2d(-55, 62))

                        // grab speciment
                        // <code here>

//                        .splineToLinearHeading(new Pose2d(45.24, 38.6, Math.toRadians(90)), Math.toRadians(0))
//                        .strafeTo(new Vector2d(44, 38.6))
//                        .splineTo(new Vector2d(4.86, 29), Math.toRadians(93.24))


                        .build());


        RoadRunnerBotEntity blueBot2 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-24, 61.52, Math.toRadians(90)))
                        .back(2)
//                        .splineTo(new Vector2d(-1.52, -32), 0)
                        .splineToLinearHeading(new Pose2d(-3, 30.5, Math.toRadians(90)), Math.toRadians(270))
                        .strafeTo(new Vector2d(-3, 36))
                        .splineToLinearHeading(new Pose2d(-37.24, 32, Math.toRadians(90)), Math.toRadians(270))
                        .strafeTo(new Vector2d(-37.24, 5))
                        .splineToConstantHeading(new Vector2d(-45, 15.25), Math.toRadians(90))
                        .strafeTo(new Vector2d(-45, 55))
                        .strafeTo(new Vector2d(-45, 50))
                        .splineToLinearHeading(new Pose2d(-45, 10, Math.toRadians(270)), Math.toRadians(270))
                        .strafeTo(new Vector2d(-45, 5))
                        .splineToConstantHeading(new Vector2d(-55, 15.25), Math.toRadians(270))
                        .strafeTo(new Vector2d(-55, 55))
                        .strafeTo(new Vector2d(-45, 55))
                        .strafeTo(new Vector2d(-45, 62.5))
                        .strafeTo(new Vector2d(-45, 58))
                        .splineToLinearHeading(new Pose2d(-3, 30.5, Math.toRadians(90)), Math.toRadians(270))
                        .strafeTo(new Vector2d(-3, 32))
                        .splineToLinearHeading(new Pose2d(-50, 50, Math.toRadians(270)), Math.toRadians(90))
                        .build());

//        RoadRunnerBotEntity blueRobot = new DefaultBotBuilder(meepMeep)
//                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
//                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-24, 61.52, Math.toRadians(270)))
//                        .splineToLinearHeading(new Pose2d(-1.52, 30.5, Math.toRadians(90)), Math.toRadians(270))
//                        .splineTo(new Vector2d(-21.80, 40.90), Math.toRadians(180))
//                        .splineTo(new Vector2d(-32, 14.20), Math.toRadians(270))
//                        .splineToLinearHeading(new Pose2d(-54.42, 7.94, Math.toRadians(270)), Math.toRadians(90))
//                        .back(53)
//                        .splineToLinearHeading(new Pose2d(-5, 31, Math.toRadians(90)), Math.toRadians(270))
//                        .splineToLinearHeading(new Pose2d(-40, 55, Math.toRadians(270)), Math.toRadians(90))
//                        .back(6)
//                        .splineToLinearHeading(new Pose2d(-5, 31, Math.toRadians(90)), Math.toRadians(270))
//                        .splineToLinearHeading(new Pose2d(-40, 55, Math.toRadians(270)), Math.toRadians(90))
//                        .back(6)
//                        .splineToLinearHeading(new Pose2d(-5, 31, Math.toRadians(90)), Math.toRadians(270))
//                        .splineToLinearHeading(new Pose2d(-40, 55, Math.toRadians(270)), Math.toRadians(90))
//                        .back(6)
//                        .splineToLinearHeading(new Pose2d(-5, 31, Math.toRadians(90)), Math.toRadians(270))
//                        .splineToLinearHeading(new Pose2d(-40, 55, Math.toRadians(270)), Math.toRadians(90))
//                        .back(6)
//
//
//                        .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(redBot4)
                .addEntity(redBot3)

//                .addEntity(blueBot2)
                .addEntity(bluebot4)
                .start();
    }

}
package com.flocksafety.android.meepmeeptest;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTest {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        //drive.setPoseEstimate(new Pose2d(-35.4, -59, Math.toRadians(0)));
                        drive.trajectorySequenceBuilder(new Pose2d(-35, -59.0, Math.toRadians(90)))
                                .lineToLinearHeading( new Pose2d(-35 + 2.42, -59+39, Math.toRadians(90 - 8.14)))
                                .lineToLinearHeading( new Pose2d(-35 + 2.42 - 4, -59+39 + 8, Math.toRadians(95.83)))
                                .lineToLinearHeading( new Pose2d(-35 + 2.42 - 4 -21.1, -59+39 + 8+1, Math.toRadians(180)))
                                .lineToLinearHeading( new Pose2d(-35 + 2.42 - 4 -21.1 +32, -59+39 + 8+1, Math.toRadians(180)))
                                .lineToLinearHeading( new Pose2d(-35 + 2.42 - 4 -21.1 +12, -59+39 + 8+1, Math.toRadians(180)))
                                .lineToLinearHeading( new Pose2d(-35 + 2.42 - 4 -21.1, -59+39 + 8+1, Math.toRadians(180)))
                                .lineToLinearHeading( new Pose2d(-35 + 2.42 - 4 -21.1 +32, -59+39 + 8+1, Math.toRadians(180)))
                                .lineToLinearHeading( new Pose2d(-35 + 2.42 - 4 -21.1 +12, -59+39 + 8+1, Math.toRadians(180)))
                                .lineToLinearHeading( new Pose2d(-35 + 2.42 - 4 -21.1, -59+39 + 8+1, Math.toRadians(180)))

                              //  .lineToLinearHeading( new Pose2d(48.33, 22.74, Math.toRadians(90)))
                                .build()
                );


        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
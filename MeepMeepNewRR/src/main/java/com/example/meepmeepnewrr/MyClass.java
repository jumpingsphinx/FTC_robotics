package com.example.meepmeepnewrr;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MyClass {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-56, -11, Math.toRadians(180)))
                .lineToX(45)
                .setTangent(Math.toRadians(90))
                .lineToY(-33)
                .setTangent(Math.toRadians(0))
                .lineToX(50)
//                //UNIVERSAL
//                .strafeTo(new Vector2d(-53.9, 11.5))
//                //PICKUP
//                .waitSeconds(2)
//                .strafeTo(new Vector2d(48, 11))
//                .strafeToConstantHeading(new Vector2d(49, 28.9))
//                //PLACE
//                .waitSeconds(2)
//                .strafeTo(new Vector2d(48, 11))
//                .strafeTo(new Vector2d(-53.9, 11.5))
//                //PICKUP
//                .waitSeconds(2)
//                .strafeTo(new Vector2d(48, 11))
//                .strafeToConstantHeading(new Vector2d(49, 28.9))
//                //PLACE
//                .waitSeconds(2)
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
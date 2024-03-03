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

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(14, -61.7, Math.toRadians(90)))
                .lineToY(-37)
                .turnTo(Math.toRadians(183))
                .setTangent(Math.toRadians(90))
                .lineToY(-33)
                .setTangent(0)
                .lineToX(14)
                //STOP
                .setTangent(Math.toRadians(0))
                .lineToX(44.5)
                .setTangent(Math.toRadians(90))
                .lineToY(-27)
                .setTangent(0)
                .lineToX(50)
                //CLOSE
                .lineToX(45)
                .setTangent(Math.toRadians(90))
                .lineToY(-12)
                .setTangent(0)
                .lineToX(54)
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
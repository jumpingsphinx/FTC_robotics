package com.example.meepmeepnewrr;
import com.acmerobotics.roadrunner.Action;
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

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(11.8, -61.7, Math.toRadians(90)))
                .lineToY(-37)
                .turnTo(Math.toRadians(183))
                .setTangent(Math.toRadians(90))
                .lineToY(-33)
                .setTangent(0)
                .lineToX(14)
                .setTangent(Math.toRadians(0))
                .lineToX(44.5)
                .setTangent(Math.toRadians(90))
                .lineToY(-27)
                .setTangent(0)
                .lineToX(50)
                .lineToX(45)
                .setTangent(Math.toRadians(90))
                .lineToY(-11)
                .setTangent(0)
                .lineToX(-56)
                .lineToX(45)
                .setTangent(Math.toRadians(90))
                .lineToY(-33)
                .setTangent(Math.toRadians(0))
                .lineToX(50)
                .lineToX(45)
                .setTangent(Math.toRadians(90))
                .lineToY(-11)
                .setTangent(0)
                .lineToX(-56)
                .lineToX(45)
                .setTangent(Math.toRadians(90))
                .lineToY(-33)
                .setTangent(Math.toRadians(0))
                .lineToX(50)
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
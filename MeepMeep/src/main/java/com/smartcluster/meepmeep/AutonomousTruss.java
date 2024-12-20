package com.smartcluster.meepmeep;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class AutonomousTruss {
    public static void main(String[] args)
    {
        MeepMeep meepMeep = new MeepMeep(800);
        AutonomousUtils.AllianceColor color = AutonomousUtils.AllianceColor.Red;
        AutonomousUtils.StartPosition startPosition = AutonomousUtils.StartPosition.Basket;
        AutonomousUtils.Park park = AutonomousUtils.Park.Side;
        AutonomousUtils.AutoCase autoCase= AutonomousUtils.AutoCase.Side;
        RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 75, Math.toRadians(180), Math.toRadians(180),10.754825427067597363146047734721)
                .setDimensions(352/25.4, 382/25.4)
                .build();


        bot.runAction(
                new SequentialAction(
                        caseAndStack(bot.getDrive(),color,autoCase),
                        stackToBackdrop(bot.getDrive(),color, startPosition, autoCase)
//                        backdropToStack(bot.getDrive(), color, startPosition,autoCase),
//                        stackToBackdropExterior(bot.getDrive(),color, startPosition),
//                        backdropToStack2(bot.getDrive(), color, startPosition),
//
//                        stack2ToBackdropExterior(bot.getDrive(),color,startPosition)
                    )
            );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(bot)
                .start();
    }
    public static Action caseAndStack(DriveShim driveShim, AutonomousUtils.AllianceColor color, AutonomousUtils.AutoCase autoCase)
    {
        final Pose2d startPose = AutonomousBlueTrussPoses.getStartPose(color, AutonomousUtils.StartPosition.Basket);
        final Pose2d casePose = AutonomousBlueTrussPoses.getCasePose(color, AutonomousUtils.StartPosition.Basket,autoCase);
        final Pose2d stackPose = AutonomousBlueTrussPoses.getStackPose(color, AutonomousUtils.StartPosition.Basket);
        final Twist2d backAway = new Twist2d(new Vector2d(-9,0),0);

        switch (autoCase)
        {
            case Truss:
                return driveShim.actionBuilder(startPose)
                        .setTangent(startPose.heading)
                        .splineToLinearHeading(casePose, casePose.heading)
                        .setTangent(casePose.heading.log()+Math.PI/2)
                        .setReversed(true)
                        .splineToLinearHeading(stackPose, AutonomousUtils.mirrorColor(Math.toRadians(90), color))
                        .build();
            case Side:
                return driveShim.actionBuilder(startPose)
                        .setTangent(startPose.heading)
                        .splineToLinearHeading(casePose, casePose.heading)
                        .setTangent(casePose.heading.log()-Math.PI)
                        .splineToLinearHeading(casePose.plus(backAway).plus(new Twist2d(new Vector2d(-7,0),0)), casePose.heading.log()-Math.PI)
                        .setTangent(AutonomousUtils.mirrorColor(Math.toRadians(45), color))
                        .splineToLinearHeading(stackPose, Math.toRadians(180))
                        .build();
            case Middle:
                return driveShim.actionBuilder(startPose)
                        .setTangent(startPose.heading)
                        .splineToLinearHeading(casePose, casePose.heading)
                        .setTangent(casePose.heading.log()-Math.PI)
                        .splineToLinearHeading(stackPose, AutonomousUtils.mirrorColor(Math.toRadians(90),color))
                        .build();
        }
        return null;
    }

    public static Action stackToBackdrop(DriveShim drive, AutonomousUtils.AllianceColor color, AutonomousUtils.StartPosition startPosition, AutonomousUtils.AutoCase autoCase)
    {
        final Pose2d cotPose = AutonomousUtils.mirrorColor(new Pose2d(AutonomousUtils.TILE_WIDTH, -AutonomousUtils.TILE_WIDTH_HALF+2, Math.toRadians(0)),color);
        final Pose2d cot2Pose = AutonomousUtils.mirrorColor(new Pose2d(-AutonomousUtils.TILE_WIDTH, -AutonomousUtils.TILE_WIDTH_HALF+2, Math.toRadians(0)),color);

        return drive.actionBuilder(AutonomousBlueTrussPoses.getStackPose(color, startPosition))
                .setTangent(AutonomousUtils.mirrorColor(Math.toRadians(0),color))
                .setReversed(false)
                .splineToLinearHeading(cot2Pose, Math.toRadians(0))
                .splineToLinearHeading(cotPose, Math.toRadians(0))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(AutonomousBlueTrussPoses.getBackdropPose(color,startPosition,autoCase), Math.toRadians(0))
                .build();
    }
    public static Action push(DriveShim drive, AutonomousUtils.AllianceColor color, AutonomousUtils.StartPosition startPosition, AutonomousUtils.AutoCase autoCase)
    {
        final Pose2d cotPose = AutonomousUtils.mirrorColor(new Pose2d(AutonomousUtils.TILE_WIDTH, -AutonomousUtils.TILE_WIDTH_HALF+2, Math.toRadians(0)),color);
        final Pose2d cot2Pose = AutonomousUtils.mirrorColor(new Pose2d(-AutonomousUtils.TILE_WIDTH, -AutonomousUtils.TILE_WIDTH_HALF+2, Math.toRadians(0)),color);

        return drive.actionBuilder(AutonomousBlueTrussPoses.getStackPose(color, startPosition))
//                .setTangent(AutonomousUtils.mirrorColor(Math.toRadians(0),color))
//                .setReversed(false)
//                .splineToLinearHeading(cot2Pose, Math.toRadians(0))
//                .splineToLinearHeading(cotPose, Math.toRadians(0))
//                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(AutonomousBlueTrussPoses.getBackdropPose(color,startPosition,autoCase), Math.toRadians(0))
                .build();
    }
//    public static Action stackToBackdropExterior(DriveShim drive, AutonomousUtils.AllianceColor color, AutonomousUtils.StartPosition startPosition)
//    {
//        final Pose2d cotPose = AutonomousUtils.mirrorColor(new Pose2d(AutonomousUtils.TILE_WIDTH, -AutonomousUtils.TILE_WIDTH_HALF+2, Math.toRadians(0)),color);
//        final Pose2d cot2Pose = AutonomousUtils.mirrorColor(new Pose2d(-AutonomousUtils.TILE_WIDTH, -AutonomousUtils.TILE_WIDTH_HALF+2, Math.toRadians(0)),color);
//        return drive.actionBuilder(AutonomousBlueTrussPoses.getStackPose(color, startPosition))
//                .setTangent(Math.toRadians(0))
//                .setReversed(false)
//                .splineToLinearHeading(cot2Pose, Math.toRadians(0))
//                .splineToLinearHeading(cotPose, Math.toRadians(0))
//                .splineToLinearHeading(AutonomousBlueTrussPoses.getBackdropExterior(color,startPosition), Math.toRadians(0))
//                .build();
//    }
//    public static Action backdropToStack(DriveShim drive, AutonomousUtils.AllianceColor color, AutonomousUtils.StartPosition startPosition, AutonomousUtils.AutoCase autoCase)
//    {
//        final Pose2d cotPose = AutonomousUtils.mirrorColor(new Pose2d(AutonomousUtils.TILE_WIDTH, -AutonomousUtils.TILE_WIDTH_HALF+2, Math.toRadians(0)),color);
//        final Pose2d cot2Pose = AutonomousUtils.mirrorColor(new Pose2d(-AutonomousUtils.TILE_WIDTH, -AutonomousUtils.TILE_WIDTH_HALF+2, Math.toRadians(0)),color);
//
//        return drive.actionBuilder(AutonomousBlueTrussPoses.getBackdropPose(color,startPosition,autoCase))
//                .setTangent(Math.toRadians(180))
//
//                .setReversed(true)
//                .splineToLinearHeading(cotPose, Math.toRadians(180))
//                .splineToLinearHeading(cot2Pose, Math.toRadians(180))
//                .setTangent(Math.toRadians(180))
//                .splineToLinearHeading(AutonomousBlueTrussPoses.getStackPose(color, startPosition), Math.toRadians(180))
//                .build();
//    }
//    public static Action backdropToStack2(DriveShim drive, AutonomousUtils.AllianceColor color, AutonomousUtils.StartPosition startPosition)
//    {
//        final Pose2d cotPose = AutonomousUtils.mirrorColor(new Pose2d(AutonomousUtils.TILE_WIDTH, -AutonomousUtils.TILE_WIDTH_HALF, Math.toRadians(0)),color);
//        final Pose2d cot2Pose = AutonomousUtils.mirrorColor(new Pose2d(-AutonomousUtils.TILE_WIDTH, -AutonomousUtils.TILE_WIDTH_HALF, Math.toRadians(0)),color);
//
//        return drive.actionBuilder(AutonomousBlueTrussPoses.getBackdropExterior(color,startPosition))
//                .setTangent(Math.toRadians(180))
//                .setReversed(true)
//                .splineToLinearHeading(cotPose, Math.toRadians(180))
//                .setTangent(Math.toRadians(180))
//                .splineToSplineHeading(cot2Pose, Math.toRadians(180))
//                .setTangent(Math.toRadians(180))
//                .splineToLinearHeading(AutonomousBlueTrussPoses.getStack2Pose(color, startPosition), AutonomousBlueTrussPoses.getStack2Pose(color, startPosition).heading.log()-Math.PI)
//                .build();
//    }
//    public static Action stack2ToBackdropExterior(DriveShim drive, AutonomousUtils.AllianceColor color, AutonomousUtils.StartPosition startPosition)
//    {
//        final Pose2d cotPose = AutonomousUtils.mirrorColor(new Pose2d(AutonomousUtils.TILE_WIDTH, -AutonomousUtils.TILE_WIDTH_HALF+2, Math.toRadians(0)),color);
//        final Pose2d cot2Pose = AutonomousUtils.mirrorColor(new Pose2d(-AutonomousUtils.TILE_WIDTH, -AutonomousUtils.TILE_WIDTH_HALF, Math.toRadians(0)),color);
//        return drive.actionBuilder(AutonomousBlueTrussPoses.getStack2Pose(color, startPosition))
//                .setTangent(Math.toRadians(0))
//                .setReversed(false)
//                .splineToSplineHeading(cot2Pose, Math.toRadians(0))
//                .setTangent(Math.toRadians(0))
//                .splineToSplineHeading(cotPose, Math.toRadians(0))
//                .splineToLinearHeading(AutonomousBlueTrussPoses.getBackdropExterior(color,startPosition), Math.toRadians(0))
//                .build();
//    }
}

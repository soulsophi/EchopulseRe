//package com.smartcluster.meepmeep;
//
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.SequentialAction;
//import com.acmerobotics.roadrunner.Twist2d;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.noahbres.meepmeep.MeepMeep;
//import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
//import com.noahbres.meepmeep.roadrunner.DriveShim;
//import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
//
//public class AutonomousBackdrop {
//    public static void main(String[] args)
//    {
//        MeepMeep meepMeep = new MeepMeep(800);
//        AutonomousUtils.AllianceColor color = AutonomousUtils.AllianceColor.Blue;
//        AutonomousUtils.StartPosition startPosition = AutonomousUtils.StartPosition.Corner;
//        AutonomousUtils.Park park = AutonomousUtils.Park.Side;
//        AutonomousUtils.AutoCase autoCase= AutonomousUtils.AutoCase.Side;
//        RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep)
//                .setConstraints(60, 75, Math.toRadians(180), Math.toRadians(180),10.754825427067597363146047734721)
//                .setDimensions(352/25.4, 382/25.4)
//                .build();
//
//
//        bot.runAction(
//                new SequentialAction(
//                        caseAndBackdrop(bot.getDrive(),color,autoCase),
//                        backdropToStack(bot.getDrive(), color,autoCase),
//                        stackToBackdrop(bot.getDrive(),color)
//                )
//        );
//
//        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
//                .setDarkMode(true)
//                .setBackgroundAlpha(0.95f)
//                .addEntity(bot)
//                .start();
//
//    }
//    public static Action caseAndBackdrop(DriveShim drive, AutonomousUtils.AllianceColor color, AutonomousUtils.AutoCase autoCase)
//    {
//        final Pose2d startPose = AutonomousUtils.getStartPose(color, AutonomousUtils.StartPosition.Backdrop);
//        final Pose2d casePose = AutonomousUtils.getCasePose(color, AutonomousUtils.StartPosition.Backdrop,autoCase);
//        final Pose2d backdropPose = AutonomousUtils.getBackdropPose(color, AutonomousUtils.StartPosition.Backdrop, autoCase);
//        final Twist2d backAway = new Twist2d(new Vector2d(-12,0), AutonomousUtils.mirrorColor(Math.toRadians(50),color));
//        return drive.actionBuilder(startPose)
//                .setTangent(startPose.heading)
//                .splineToLinearHeading(casePose, casePose.heading)
//                .setTangent(casePose.heading.log()-Math.PI)
//                .splineToLinearHeading(casePose.plus(backAway), casePose.plus(backAway).heading.log()-Math.PI)
//                .setTangent(Math.toRadians(0))
//                .splineToSplineHeading(backdropPose, Math.toRadians(0))
//                .build();
//
//    }
//    public static Action backdropToStack(DriveShim drive, AutonomousUtils.AllianceColor color, AutonomousUtils.AutoCase autoCase) {
//        final Pose2d backdropPose = AutonomousUtils.getBackdropPose(color, AutonomousUtils.StartPosition.Backdrop, autoCase);
//        final Pose2d cotPose = AutonomousUtils.mirrorColor(new Pose2d(AutonomousUtils.TILE_WIDTH, -2* AutonomousUtils.TILE_WIDTH- AutonomousUtils.TILE_WIDTH_HALF, Math.toRadians(0)),color);
//        final Pose2d cot2Pose = AutonomousUtils.mirrorColor(new Pose2d(-AutonomousUtils.TILE_WIDTH, -2* AutonomousUtils.TILE_WIDTH- AutonomousUtils.TILE_WIDTH_HALF, Math.toRadians(0)),color);
//        final Pose2d stackPose = AutonomousUtils.getStackPose(color, AutonomousUtils.StartPosition.Backdrop);
//        return drive.actionBuilder(backdropPose)
//                .setTangent(Math.toRadians(180))
//                .setReversed(true)
//                .splineToSplineHeading(cotPose, Math.toRadians(180))
//                .splineToSplineHeading(cot2Pose, Math.toRadians(180))
//                .splineToLinearHeading(stackPose, stackPose.heading.log()-Math.PI)
//                .build();
//    }
//    public static Action stackToBackdrop(DriveShim drive, AutonomousUtils.AllianceColor color) {
//        final Pose2d backdropPose = AutonomousUtils.getBackdropExterior(color, AutonomousUtils.StartPosition.Backdrop);
//        final Pose2d cotPose = AutonomousUtils.mirrorColor(new Pose2d(AutonomousUtils.TILE_WIDTH, -2* AutonomousUtils.TILE_WIDTH- AutonomousUtils.TILE_WIDTH_HALF, Math.toRadians(0)),color);
//        final Pose2d cot2Pose = AutonomousUtils.mirrorColor(new Pose2d(-AutonomousUtils.TILE_WIDTH, -2* AutonomousUtils.TILE_WIDTH- AutonomousUtils.TILE_WIDTH_HALF, Math.toRadians(0)),color);
//        final Pose2d stackPose = AutonomousUtils.getStackPose(color, AutonomousUtils.StartPosition.Backdrop);
//        return drive.actionBuilder(stackPose)
//                .setTangent(Math.toRadians(0))
//                .setReversed(false)
//                .splineToSplineHeading(cot2Pose, Math.toRadians(0))
//                .splineToSplineHeading(cotPose, Math.toRadians(0))
//                .splineToLinearHeading(backdropPose, Math.toRadians(0))
//                .build();
//    }
//}

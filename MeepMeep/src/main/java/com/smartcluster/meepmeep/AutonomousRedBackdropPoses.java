package com.smartcluster.meepmeep;

import static com.smartcluster.meepmeep.AutonomousUtils.mirrorColor;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Vector2d;

public class AutonomousRedBackdropPoses {
    public static Pose2d getStartPose(AutonomousUtils.AllianceColor color, AutonomousUtils.StartPosition startPosition)
    {
        if(color== AutonomousUtils.AllianceColor.Red)
            if(startPosition== AutonomousUtils.StartPosition.Corner)
                return new Pose2d(AutonomousUtils.TILE_WIDTH_HALF,-(3*AutonomousUtils.TILE_WIDTH-(382/25.4)/2),Math.toRadians(90));
            else
                return new Pose2d(-(AutonomousUtils.TILE_WIDTH+AutonomousUtils.TILE_WIDTH_HALF),-(3*AutonomousUtils.TILE_WIDTH-AutonomousUtils.ROBOT_LENGTH/2),Math.toRadians(90));
        else
        if(startPosition== AutonomousUtils.StartPosition.Corner)
            return new Pose2d(AutonomousUtils.TILE_WIDTH_HALF,(3*AutonomousUtils.TILE_WIDTH-(382/25.4)/2),Math.toRadians(-90));
        else
            return new Pose2d(-(2*AutonomousUtils.TILE_WIDTH-AutonomousUtils.TILE_WIDTH_HALF),(3*AutonomousUtils.TILE_WIDTH-(382/25.4)/2),Math.toRadians(-90));
    }

    public static Pose2d getCasePose(AutonomousUtils.AllianceColor color, AutonomousUtils.StartPosition startPosition, AutonomousUtils.AutoCase autoCase)
    {
        final Twist2d robotFromPurplePixel = new Twist2d(new Vector2d(-8, 0),0);
        final Pose2d trussCase = new Pose2d(0.5, -(AutonomousUtils.TILE_WIDTH+AutonomousUtils.TILE_WIDTH_HALF), Math.toRadians(150)).plus(robotFromPurplePixel);
        final Pose2d middleCase = new Pose2d(AutonomousUtils.TILE_WIDTH_QUARTER,-AutonomousUtils.TILE_WIDTH-0.5, Math.toRadians(110)).plus(robotFromPurplePixel);
        final Pose2d sideCase = new Pose2d(AutonomousUtils.TILE_WIDTH-0.5, -(AutonomousUtils.TILE_WIDTH+AutonomousUtils.TILE_WIDTH_HALF), Math.toRadians(60)).plus(robotFromPurplePixel);


        Pose2d targetPose = new Pose2d(0,0,0);
        switch (autoCase)
        {
            case Side:
                targetPose=sideCase;
                break;
            case Truss:
                targetPose=trussCase;
                break;
            case Middle:
                targetPose=middleCase;
                break;
        }
        final Vector2d mirrorCenter = new Vector2d(-AutonomousUtils.TILE_WIDTH_HALF,0);
        double mirroredX=-(targetPose.position.x-mirrorCenter.x)+mirrorCenter.x;
        if(color== AutonomousUtils.AllianceColor.Red)
            if(startPosition== AutonomousUtils.StartPosition.Corner)
                return targetPose;
            else {
                return new Pose2d(mirroredX, targetPose.position.y, -(targetPose.heading.log()-Math.PI/2)+Math.PI/2);
            }
        else if(startPosition== AutonomousUtils.StartPosition.Corner)
        {
            return new Pose2d(targetPose.position.x, -targetPose.position.y, -targetPose.heading.log());
        }else {
            return new Pose2d(mirroredX, -targetPose.position.y, targetPose.heading.log()-Math.PI);
        }
    }
    public static Pose2d getStackPose(AutonomousUtils.AllianceColor color, AutonomousUtils.StartPosition startPosition)
    {
        if(startPosition== AutonomousUtils.StartPosition.Corner)
            return mirrorColor(new Pose2d(-3*AutonomousUtils.TILE_WIDTH, -AutonomousUtils.TILE_WIDTH-AutonomousUtils.TILE_WIDTH_QUARTER-11, Math.toRadians(0)), color).plus(new Twist2d(new Vector2d(15+6.5,0),mirrorColor(Math.toRadians(-30),color)));
        else
            return mirrorColor(new Pose2d(-2*AutonomousUtils.TILE_WIDTH, -AutonomousUtils.TILE_WIDTH_HALF-2 , Math.toRadians(0)), color);
    }
    public static Pose2d getStack2Pose(AutonomousUtils.AllianceColor color, AutonomousUtils.StartPosition startPosition)
    {
        if(startPosition== AutonomousUtils.StartPosition.Corner)
            return new Pose2d(0,0,0);
        else
            return mirrorColor(new Pose2d(-2*AutonomousUtils.TILE_WIDTH-AutonomousUtils.TILE_WIDTH_HALF+4, -AutonomousUtils.TILE_WIDTH_HALF, Math.toRadians(30)), color);
    }
    public static Pose2d getBackdropPose(AutonomousUtils.AllianceColor color, AutonomousUtils.StartPosition startPosition, AutonomousUtils.AutoCase autoCase)
    {
        final Pose2d baseBackdropPose = new Pose2d(2*AutonomousUtils.TILE_WIDTH+2,-AutonomousUtils.TILE_WIDTH-AutonomousUtils.TILE_WIDTH_HALF,Math.toRadians(0));
        final double[] offset = new double[] {-5.5,0.5,5};

        if((color== AutonomousUtils.AllianceColor.Red&&startPosition== AutonomousUtils.StartPosition.Corner)||(color== AutonomousUtils.AllianceColor.Blue&&startPosition== AutonomousUtils.StartPosition.Corner))
            return mirrorColor(new Pose2d(baseBackdropPose.position.x, baseBackdropPose.position.y+offset[2-autoCase.ordinal()], Math.toRadians(0)),color);
        else
            return mirrorColor(new Pose2d(baseBackdropPose.position.x, baseBackdropPose.position.y+offset[autoCase.ordinal()], Math.toRadians(0)),color);
    }
    public static Pose2d getBackdropExterior(AutonomousUtils.AllianceColor color, AutonomousUtils.StartPosition startPosition)
    {

        return mirrorColor(new Pose2d(2*AutonomousUtils.TILE_WIDTH+2,-AutonomousUtils.TILE_WIDTH-AutonomousUtils.TILE_WIDTH_HALF+5,Math.toRadians(0)),color);

    }
    public static Pose2d getParkPose(AutonomousUtils.AllianceColor color, AutonomousUtils.Park park)
    {
        if(park== AutonomousUtils.Park.Center)
        {
            return mirrorColor(new Pose2d(2*AutonomousUtils.TILE_WIDTH, -AutonomousUtils.TILE_WIDTH_HALF, Math.toRadians(0)), color);
        }else return mirrorColor(new Pose2d(2*AutonomousUtils.TILE_WIDTH, -AutonomousUtils.TILE_WIDTH_HALF-2*AutonomousUtils.TILE_WIDTH, Math.toRadians(0)), color);
    }
    public static double getParkTangent(AutonomousUtils.AllianceColor color, AutonomousUtils.Park park)
    {
        if(park== AutonomousUtils.Park.Center) return  mirrorColor(Math.toRadians(90), color);
        else return mirrorColor(Math.toRadians(-90),color);
    }



}

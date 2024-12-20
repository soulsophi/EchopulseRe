package com.smartcluster.meepmeep;

import com.acmerobotics.roadrunner.Pose2d;

public class AutonomousUtils {
    public enum AllianceColor{
        Red,
        Blue
    };

    public enum StartPosition {
        Corner,//backdrop
        Basket//truss
    }
    public enum AutoCase {
        Truss,
        Middle,
        Side;

    }

    public enum Park {
        Center,
        Side
    }

    public static double TILE_WIDTH =23.6;
    public static double TILE_WIDTH_HALF = TILE_WIDTH/2;
    public static double TILE_WIDTH_QUARTER = TILE_WIDTH/4;
    public static double ROBOT_WIDTH = (348/25.4);
    public static double ROBOT_LENGTH = (382/25.4);

    public static double mirrorColor(double heading, AutonomousUtils.AllianceColor color)
    {
        if(color== AutonomousUtils.AllianceColor.Blue) return heading;
        else return -heading;
    }
    public static Pose2d mirrorColor(Pose2d pose, AutonomousUtils.AllianceColor color)
    {
        if(color== AutonomousUtils.AllianceColor.Blue) return new Pose2d(pose.position.x, -pose.position.y, mirrorColor(pose.heading.log(), color));
        else return pose;
    }



}

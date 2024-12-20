package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Kinematics {
    public static double sliderHeight=37.58873   ; // mm
    public static double sliderWidth=270.60940; // mm
    public static double liftPoint=166.00000; // mm
    public static double liftRadius=8; // mm
    public static double linkageLength=96; // mm
    public static double boxOffset=3.8901155; // deg
    public static double boxLength = 87.84304; // mm

    public static double pivotY= - 7.9694; // mm
    public static double pivotX = 84.130420; // mm
    public static double pivot2pivotLength= Math.sqrt(pivotX*pivotX+pivotY*pivotY); // mm
    public static double pivotAngle = Math.abs(Math.atan2(pivotY, pivotX)); // deg
    public static double[] inverseKinematics(double x, double y)
    {
        double actualX = x+sliderWidth;
        double actualY = y+sliderHeight;
        double diagonal = Math.sqrt(actualX*actualX+actualY*actualY);
        double alpha = Math.asin(sliderHeight/diagonal);

        double angle = Math.atan2(actualY, actualX) - alpha;

        double targetHeight = Math.tan(angle)*liftPoint - (1/Math.sin(Math.PI/2.0-angle))*liftRadius+liftRadius;
        double targetDistance = Math.sqrt(diagonal*diagonal-sliderHeight*sliderHeight)-sliderWidth;

        double linkageAngle = Math.toRadians(60)-angle-Math.toRadians(boxOffset);

        double boxX = pivotX+Math.cos(linkageAngle)*boxLength;
        double boxY = pivotY+ Math.sin(linkageAngle)*boxLength;

        double linkageDiagonal = Math.sqrt(boxX*boxX + boxY*boxY);

        double overAngle = Math.acos(linkageDiagonal/(2*linkageLength));
        double underAngle = Math.acos((boxLength*boxLength-linkageDiagonal*linkageDiagonal-pivot2pivotLength*pivot2pivotLength)/(-2*pivot2pivotLength*linkageDiagonal));

        double targetAngle = Math.PI-(overAngle+underAngle-Math.toRadians(pivotAngle));

        return new double[] {targetDistance, targetHeight, Math.toDegrees(targetAngle), Math.toDegrees(angle)};

    }
    public static double[] forwardKinematics(double distance, double pitch)
    {
        double startPointX = Math.cos(Math.toRadians(pitch+90))*sliderHeight;
        double startPointY = Math.sin(Math.toRadians(pitch+90))*sliderHeight;
        double endPointX = startPointX + Math.cos(Math.toRadians(pitch))*(distance+sliderWidth);
        double endPointY = startPointY + Math.sin(Math.toRadians(pitch))*(distance+sliderWidth);
        return new double[] {endPointX-sliderWidth, endPointY-sliderHeight};
    }
}

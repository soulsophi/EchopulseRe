//package org.firstinspires.ftc.teamcode.autonomous.vision;
//
//import android.graphics.Bitmap;
//import android.graphics.Canvas;
//import android.graphics.Color;
//import android.graphics.Paint;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.smartcluster.meepmeep.AutonomousUtils;
//
//import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
//import org.firstinspires.ftc.vision.VisionProcessor;
//import org.opencv.android.Utils;
//import org.opencv.core.Core;
//import org.opencv.core.Mat;
//import org.opencv.core.MatOfPoint;
//import org.opencv.core.Rect;
//import org.opencv.core.Scalar;
//import org.opencv.imgproc.Imgproc;
//import org.opencv.imgproc.Moments;
//
//import java.util.ArrayList;
//import java.util.Collections;
//
//@Config
//public
//class CaseDetectionVisionProcessor implements VisionProcessor {
//    private static final String TAG="CaseDetectionVisionProcessor";
//    public enum DisplayMode {
//        NO_DISPLAY,
//        MASK,
//        CONTOURS,
//        TOP_CONTOUR
//    }
//
//    public static DisplayMode mode = DisplayMode.NO_DISPLAY;
//    private final AutonomousUtils.AllianceColor color;
//    private boolean reversed=false;
//    public CaseDetectionVisionProcessor(AutonomousUtils.AllianceColor color, AutonomousUtils.StartPosition startPosition)
//    {
//        this.color=color;
//        if((color== AutonomousUtils.AllianceColor.Red&&startPosition== AutonomousUtils.StartPosition.Truss)
//            || (color== AutonomousUtils.AllianceColor.Blue&&startPosition== AutonomousUtils.StartPosition.Backdrop))
//            reversed=true;
//    }
//
//    private final Paint linePaint = new Paint();
//    private int width, height;
//    @Override
//    public void init(int width, int height, CameraCalibration calibration) {
//        this.width=width;
//        this.height=height;
//        linePaint.setColor(Color.GREEN); // you may want to change this
//        linePaint.setAntiAlias(true);
//        linePaint.setStrokeWidth(5); // or this
//        linePaint.setStrokeCap(Paint.Cap.ROUND);
//        linePaint.setStrokeJoin(Paint.Join.ROUND);
//    }
//
//    public int detectedContours;
//
//
//    public AutonomousUtils.AutoCase autoCase;
//
//    private final Mat labFrame =new Mat();
//    private final Mat mask = new Mat();
//    private final Mat hierarchy = new Mat();
//    private final Mat drawFrame = new Mat();
//    public static class ContourAreaPair
//    {
//        public MatOfPoint contour;
//        public double area;
//        public ContourAreaPair(MatOfPoint contour)
//        {
//            this.contour=contour;
//            this.area=Imgproc.contourArea(contour);
//        }
//    }
//    private final ArrayList<Mat> labChannels = new ArrayList<>();
//   @Override
//    public Object processFrame(Mat frame, long captureTimeNanos) {
//        Mat newFrame=frame.submat(new Rect(0, height/2, width, height/2));
//        Imgproc.cvtColor(newFrame, labFrame, Imgproc.COLOR_RGB2Lab);
//        Core.split(labFrame, labChannels);
//        ArrayList<MatOfPoint> contours = new ArrayList<>();
//        if (color == AutonomousUtils.AllianceColor.Red) {
//            Imgproc.threshold(labChannels.get(1), mask, 127, 255, Imgproc.THRESH_BINARY + Imgproc.THRESH_OTSU);
//        } else if (color == AutonomousUtils.AllianceColor.Blue) {
//            Imgproc.threshold(labChannels.get(2), mask, 127, 255, Imgproc.THRESH_BINARY_INV + Imgproc.THRESH_OTSU);
//        }
//        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
//        detectedContours = contours.size();
//
//        MatOfPoint maxContour=null;
//        double maxArea=0;
//        for(MatOfPoint contour:contours)
//        {
//            double area = Imgproc.contourArea(contour);
//            if(area>maxArea)
//            {
//                maxArea=area;
//                maxContour=contour;
//            }
//        }
//        if(maxContour!=null)
//        {
//            Moments p =Imgproc.moments(maxContour, false);
//            int x = (int) (p.get_m10() / p.get_m00());
//            if(x<width/3)
//            {
//                autoCase= AutonomousUtils.AutoCase.Truss;
//            }
//            else if(x>width*2/3.0)
//            {
//                autoCase= AutonomousUtils.AutoCase.Side;
//            }else autoCase= AutonomousUtils.AutoCase.Middle ;
//        }
//        if(autoCase!=null&&reversed)
//        {
//            switch (autoCase)
//            {
//                case Truss:
//                    autoCase= AutonomousUtils.AutoCase.Side;
//                    break;
//                case Side:
//                    autoCase= AutonomousUtils.AutoCase.Truss;
//                    break;
//            }
//        }
//
//        synchronized (drawFrame)
//        {
//            switch (mode)
//            {
//                case MASK:
//                    mask.copyTo(drawFrame);
//                    break;
//                case CONTOURS:
//                    newFrame.copyTo(drawFrame);
//                    Imgproc.drawContours(drawFrame, contours, -1, new Scalar(0,255,0));
//                    break;
//                case TOP_CONTOUR:
//                    newFrame.copyTo(drawFrame);
//                    if(maxContour!=null)
//                        Imgproc.drawContours(drawFrame, Collections.singletonList(maxContour), 0, new Scalar(0,255,0));
//                    break;
//            }
//        }
//
//
//
//        return new Object();
//    }
//
//
//    @Override
//    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
//        canvas.save();
//        canvas.scale(scaleBmpPxToCanvasPx,scaleBmpPxToCanvasPx);
//
//        synchronized (drawFrame)
//        {
//            if (mode != DisplayMode.NO_DISPLAY) {
//                if(drawFrame.height()>0&&drawFrame.width()>0)
//                {
//                    Bitmap bitmap = Bitmap.createBitmap(drawFrame.width(), drawFrame.height(), Bitmap.Config.ARGB_8888);
//                    Utils.matToBitmap(drawFrame, bitmap);
//                    canvas.drawBitmap(bitmap, new android.graphics.Rect(0, 0,drawFrame.width(), drawFrame.height()), new android.graphics.Rect(0, 0, drawFrame.width(), drawFrame.height()), null);
//                }
//            }
//
//        }
//
//
//        canvas.restore();
//    }
//}

//package org.firstinspires.ftc.teamcode.calibration;
//
//import android.util.Size;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.smartcluster.meepmeep.AutonomousUtils;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.autonomous.vision.CaseDetectionVisionProcessor;
//import org.firstinspires.ftc.vision.VisionPortal;
//
//@TeleOp(group = "calibration")
//public class VisionCalibrationTeleOp extends LinearOpMode {
//    @Override
//    public void runOpMode() throws InterruptedException {
//        WebcamName c270 = hardwareMap.get(WebcamName.class, "C270");
//        CaseDetectionVisionProcessor processor = new CaseDetectionVisionProcessor(AutonomousUtils.AllianceColor.Blue, AutonomousUtils.StartPosition.Backdrop);
//        VisionPortal portal = new VisionPortal.Builder()
//                .setCamera(c270)
//                .setCameraResolution(new Size(1280, 720))
//                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
//                .setAutoStopLiveView(true)
//                .addProcessor(processor)
//                .build();
//
//        waitForStart();
//        while(opModeIsActive())
//        {
//            telemetry.addData("autoCase", processor.autoCase);
//            telemetry.update();
//        }
//    }
//}

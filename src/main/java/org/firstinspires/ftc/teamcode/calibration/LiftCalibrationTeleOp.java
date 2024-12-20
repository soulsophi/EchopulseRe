package org.firstinspires.ftc.teamcode.calibration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.smartcluster.oracleftc.commands.CommandScheduler;

import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem;

@TeleOp(group = "calibration")
public class LiftCalibrationTeleOp extends LinearOpMode {
    private final CommandScheduler scheduler = new CommandScheduler();
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        LiftSubsystem liftSubsystem = new LiftSubsystem(this);
        waitForStart();
        scheduler.schedule(liftSubsystem.telemetry());
        scheduler.schedule(liftSubsystem.manual());
        while (opModeIsActive())
        {
            scheduler.update();
            telemetry.update();
        }
    }
}

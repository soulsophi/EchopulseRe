package org.firstinspires.ftc.teamcode.calibration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.smartcluster.oracleftc.commands.CommandScheduler;

import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;

@TeleOp(group = "calibration")
public class IntakeCalibrationTeleOp extends LinearOpMode {
    private final CommandScheduler scheduler = new CommandScheduler();
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        IntakeSubsystem intakeSubsystem = new IntakeSubsystem(this);
        waitForStart();
        scheduler.schedule(intakeSubsystem.manual());
        scheduler.schedule(intakeSubsystem.telemetry());
        while (opModeIsActive())
        {
            scheduler.update();
            telemetry.update();
        }
    }
}

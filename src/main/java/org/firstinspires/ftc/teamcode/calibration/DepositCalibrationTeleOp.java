package org.firstinspires.ftc.teamcode.calibration;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.smartcluster.oracleftc.commands.CommandScheduler;
import com.smartcluster.oracleftc.utils.input.OracleGamepad;

import org.firstinspires.ftc.teamcode.subsystem.DepositSubsystem;

@TeleOp(group = "calibration")
public class DepositCalibrationTeleOp extends LinearOpMode {
    private final CommandScheduler scheduler = new CommandScheduler();
    @Override
    public void runOpMode() throws InterruptedException {

        DepositSubsystem depositSubsystem = new DepositSubsystem(this);
        OracleGamepad gamepad = new OracleGamepad(gamepad1);
        waitForStart();
        scheduler.schedule(depositSubsystem.manual());

        while (opModeIsActive())
        {
            if(gamepad.cross.pressed().get())  scheduler.schedule(depositSubsystem.open());
            if(gamepad.circle.pressed().get()) scheduler.schedule(depositSubsystem.close());
            if(gamepad.square.pressed().get()) scheduler.schedule(depositSubsystem.reset());

            scheduler.update();
            telemetry.update();
            gamepad.process();
        }
    }
}

package org.firstinspires.ftc.teamcode.calibration;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.smartcluster.oracleftc.commands.CommandScheduler;
import com.smartcluster.oracleftc.commands.helpers.ParallelCommand;
import com.smartcluster.oracleftc.commands.helpers.SequentialCommand;

import org.firstinspires.ftc.teamcode.subsystem.DepositSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.SliderSubsystem;

@TeleOp(group = "calibration")
public class TransferCalibrationTeleOp extends LinearOpMode {
    private final CommandScheduler scheduler = new CommandScheduler();
    @Override
    public void runOpMode() throws InterruptedException {
        DepositSubsystem depositSubsystem=new DepositSubsystem(this);
        SliderSubsystem sliderSubsystem = new SliderSubsystem(this);
        IntakeSubsystem intakeSubsystem = new IntakeSubsystem(this);
        waitForStart();
        scheduler.schedule(new SequentialCommand(
                depositSubsystem.reset(),
                sliderSubsystem.reset(),
                intakeSubsystem.reset(),
                new ParallelCommand(
                        sliderSubsystem.update(),
                        depositSubsystem.manual(),
                        intakeSubsystem.manual()
                )
        ));

        while(opModeIsActive())
        {
            telemetry.update();
            scheduler.update();
        }
    }
}

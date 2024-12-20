package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.smartcluster.oracleftc.commands.Command;
import com.smartcluster.oracleftc.hardware.Subsystem;
import com.smartcluster.oracleftc.hardware.SubsystemFlavor;
import com.smartcluster.oracleftc.math.control.SlewRateLimiter;
import com.smartcluster.oracleftc.utils.input.JoystickData;
import com.smartcluster.oracleftc.utils.input.OracleGamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.concurrent.atomic.AtomicReference;

@Config
public class MecanumSubsystem extends Subsystem {

    // Hardware
    private final DcMotor frontRight, frontLeft, backRight, backLeft;
    public static SlewRateLimiter forwardSlewRateLimiter = new SlewRateLimiter(2,-10,0);
    public static SlewRateLimiter strafeSlewRateLimiter = new SlewRateLimiter(2,-10,0);
    public static SlewRateLimiter rotateSlewRateLimiter = new SlewRateLimiter(1.5,-10,0);

    public MecanumSubsystem(OpMode opMode) {
        super(opMode);
        frontRight=hardwareMap.get(DcMotor.class, "frontRight");
        frontLeft=hardwareMap.get(DcMotor.class, "frontLeft");
        backRight=hardwareMap.get(DcMotor.class, "backRight");
        backLeft=hardwareMap.get(DcMotor.class, "backLeft");
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public SubsystemFlavor flavor() {
        return SubsystemFlavor.ControlHubOnly;
    }

    public Command drive(OracleGamepad gamepad)
    {
        final int[] face = {1};
        return Command.builder()
                .update(()->{

                    if (gamepad.triangle.pressed().get()) {
                        face[0] =-face[0];
                    }
                    JoystickData leftStick = gamepad.left_stick.get();
                    JoystickData rightStick = gamepad.right_stick.get();

                    double rx = Math.signum(rightStick.x)*rotateSlewRateLimiter.calculate(Math.abs(rightStick.x));// (gamepad.right_bumper.processAndGet()?1:2);

                    boolean boost = gamepad.left_bumper.get()||gamepad.right_bumper.get();
                    double x = leftStick.x;
                    double y= -leftStick.y;

                    double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                    double frontLeftPower = (y + x + rx) / denominator;
                    double backLeftPower = (y - x + rx) / denominator;
                    double frontRightPower = (y - x - rx) / denominator;
                    double backRightPower = (y + x - rx) / denominator;

                    frontRight.setPower(frontRightPower);
                    frontLeft.setPower(frontLeftPower);
                    backLeft.setPower(backLeftPower);
                    backRight.setPower(backRightPower);
                })
                .requires(this)
                .build();
    }

    public Command fieldOrientedDrive(OracleGamepad gamepad, IMU imu)
    {
        AtomicReference<Double> offsetAngle = new AtomicReference<>((double) 0);
        return Command.builder()
                .update(()->{

                    if (gamepad.triangle.pressed().get()) {
                        imu.resetYaw();
                    }

                    double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);


                    JoystickData leftStick = gamepad.left_stick.get();
                    JoystickData rightStick = gamepad.right_stick.get();

                    double r =rightStick.x;
                    double rx = Math.signum(r)*rotateSlewRateLimiter.calculate(Math.abs(r));// (gamepad.right_bumper.processAndGet()?1:2);

                    boolean boost = gamepad.left_bumper.get()||gamepad.right_bumper.get();
                    double x = leftStick.x / ((boost)?1:2);
                    double y= -leftStick.y / (boost?1:2);
                    // Rotate the movement direction counter to the bot's rotation
                    double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                    double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

                    rotX*=1.15;
                    double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
                    double frontLeftPower = (rotY + rotX + rx) / denominator;
                    double backLeftPower = (rotY - rotX + rx) / denominator;
                    double frontRightPower = (rotY - rotX - rx) / denominator;
                    double backRightPower = (rotY + rotX - rx) / denominator;

                    frontRight.setPower(frontRightPower);
                    frontLeft.setPower(frontLeftPower);
                    backLeft.setPower(backLeftPower);
                    backRight.setPower(backRightPower);
                })
                .requires(this)
                .build();
    }
}

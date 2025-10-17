package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class DriverControlled extends OpMode {
    private DcMotorEx
            fl_drive,
            fr_drive,
            bl_drive,
            br_drive,
            accelerator,
            intake_motor,
            climb_motor1,
            climb_motor2;

    private final ElapsedTime acceleratorRunningTime = new ElapsedTime();

    double
            axial,
            lateral,
            yaw,
            speed,
            acceleratorTime,
            denominator,
            leftBackPower,
            rightFrontPower,
            leftFrontPower,
            rightBackPower;

    @Override
    public void init() {
        fl_drive     = hardwareMap.get(DcMotorEx.class,"left_front");
        fr_drive     = hardwareMap.get(DcMotorEx.class,"right_front");
        bl_drive     = hardwareMap.get(DcMotorEx.class,"left_back");
        br_drive     = hardwareMap.get(DcMotorEx.class,"right_back");
        accelerator  = hardwareMap.get(DcMotorEx.class,"accelerator");
        intake_motor = hardwareMap.get(DcMotorEx.class,"intake");
        climb_motor1 = hardwareMap.get(DcMotorEx.class,"climb1");
        climb_motor2 = hardwareMap.get(DcMotorEx.class,"climb2");

        fl_drive.setDirection(DcMotorEx.Direction.REVERSE);
        fr_drive.setDirection(DcMotorEx.Direction.FORWARD);
        bl_drive.setDirection(DcMotorEx.Direction.REVERSE);
        br_drive.setDirection(DcMotorEx.Direction.FORWARD);
        intake_motor.setDirection(DcMotorEx.Direction.FORWARD);
        climb_motor2.setDirection(DcMotorEx.Direction.REVERSE);

        climb_motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        climb_motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        axial           = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
        lateral         =  gamepad1.left_stick_x;
        yaw             =  gamepad1.right_stick_x;
        speed           =  0.7;
        acceleratorTime =  acceleratorRunningTime.seconds();


        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        denominator     = Math.max(Math.abs(axial) + Math.abs(lateral) + Math.abs(yaw), 1);
        leftFrontPower  = ((axial + lateral + yaw) / denominator) * speed;
        rightFrontPower = ((axial - lateral - yaw) / denominator) * speed;
        leftBackPower   = ((axial - lateral + yaw) / denominator) * speed;
        rightBackPower  = ((axial + lateral - yaw) / denominator) * speed;

        fl_drive.setPower(leftFrontPower);
        bl_drive.setPower(leftBackPower);
        fr_drive.setPower(rightFrontPower);
        br_drive.setPower(rightBackPower);
        intake_motor.setPower(gamepad1.right_trigger - gamepad1.left_trigger);

        if (acceleratorTime >= 12){
            accelerator.setPower(0);
        } else {
            if (gamepad1.triangle){
                acceleratorRunningTime.reset();
                accelerator.setPower(1);
            }
        }

        if (gamepad1.dpad_left){
            climb_motor1.setPower(0);
            climb_motor2.setPower(0);
        } else if (gamepad1.dpad_up){
            climb_motor1.setPower(1);
            climb_motor2.setPower(1);
        } else if (gamepad1.dpad_down) {
            climb_motor1.setPower(-1);
            climb_motor2.setPower(-1);
        }
    }
}

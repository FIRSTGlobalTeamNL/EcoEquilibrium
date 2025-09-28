package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class DriverControlled extends OpMode {
    private DcMotor fl_drive;
    private DcMotor fr_drive;
    private DcMotor bl_drive;
    private DcMotor br_drive;
    private CRServo accelerator;
    private ElapsedTime acceleratorRunningTime;

    @Override
    public void init() {
        fl_drive = hardwareMap.get(DcMotor.class,"front_left_drive");
        fr_drive = hardwareMap.get(DcMotor.class,"front_right_drive");
        bl_drive = hardwareMap.get(DcMotor.class,"back_left_drive");
        br_drive = hardwareMap.get(DcMotor.class,"back_right_drive");
        accelerator = hardwareMap.get(CRServo.class,"accelerator");

        fl_drive.setDirection(DcMotor.Direction.FORWARD);
        fr_drive.setDirection(DcMotor.Direction.REVERSE);
        bl_drive.setDirection(DcMotor.Direction.FORWARD);
        br_drive.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {

        double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
        double lateral =  gamepad1.left_stick_x;
        double yaw     =  gamepad1.right_stick_x;

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.


        double denominator = Math.max(Math.abs(axial) + Math.abs(lateral) + Math.abs(yaw), 1);
        double leftFrontPower  = (axial + lateral + yaw) / denominator;
        double rightFrontPower = (axial - lateral - yaw) / denominator;
        double leftBackPower   = (axial - lateral + yaw) / denominator;
        double rightBackPower  = (axial + lateral - yaw) / denominator;

        fl_drive.setPower(leftFrontPower);
        bl_drive.setPower(leftBackPower);
        fr_drive.setPower(rightFrontPower);
        br_drive.setPower(rightBackPower);

    }
}

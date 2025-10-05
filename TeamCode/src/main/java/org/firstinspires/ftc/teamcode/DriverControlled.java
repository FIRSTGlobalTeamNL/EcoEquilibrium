package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class DriverControlled extends OpMode {
    private DcMotor fl_drive;
    private DcMotor fr_drive;
    private DcMotor bl_drive;
    private DcMotor br_drive;
    private DcMotor accelerator;
    private final ElapsedTime acceleratorRunningTime = new ElapsedTime();
    private DcMotor intake_motor;
    private DcMotor climb_motor1;
    private DcMotor climb_motor2;


    @Override
    public void init() {
        fl_drive     = hardwareMap.get(DcMotor.class,"left_front");
        fr_drive     = hardwareMap.get(DcMotor.class,"right_front");
        bl_drive     = hardwareMap.get(DcMotor.class,"left_back");
        br_drive     = hardwareMap.get(DcMotor.class,"right_back");
        accelerator  = hardwareMap.get(DcMotor.class,"accelerator");
        intake_motor = hardwareMap.get(DcMotor.class,"intake");
        climb_motor1  = hardwareMap.get(DcMotor.class,"climb1");
        climb_motor2  = hardwareMap.get(DcMotor.class,"climb2");

        fl_drive.setDirection(DcMotor.Direction.REVERSE);
        fr_drive.setDirection(DcMotor.Direction.FORWARD);
        bl_drive.setDirection(DcMotor.Direction.REVERSE);
        br_drive.setDirection(DcMotor.Direction.FORWARD);
        intake_motor.setDirection(DcMotor.Direction.FORWARD);
        climb_motor2.setDirection(DcMotor.Direction.REVERSE);

    }

    @Override
    public void loop() {

        double axial          = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
        double lateral        =  gamepad1.left_stick_x;
        double yaw            =  gamepad1.right_stick_x;
        double speed          =  0.7;
        double acceleratorTime = acceleratorRunningTime.seconds();

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.


        double denominator = Math.max(Math.abs(axial) + Math.abs(lateral) + Math.abs(yaw), 1);
        double leftFrontPower  = ((axial + lateral + yaw) / denominator) * speed;
        double rightFrontPower = ((axial - lateral - yaw) / denominator) * speed;
        double leftBackPower   = ((axial - lateral + yaw) / denominator) * speed;
        double rightBackPower  = ((axial + lateral - yaw) / denominator) * speed;

        fl_drive.setPower(leftFrontPower);
        bl_drive.setPower(leftBackPower);
        fr_drive.setPower(rightFrontPower);
        br_drive.setPower(rightBackPower);
        intake_motor.setPower(gamepad1.right_trigger - gamepad1.left_trigger);

        if (acceleratorTime == 12){
            this.accelerator.setPower(0);
        } else {
            if (gamepad1.triangle){
                acceleratorRunningTime.reset();
                this.accelerator.setPower(1);
            }
        }

        if (gamepad1.dpad_up){
            climb_motor1.setPower(1);
            climb_motor2.setPower(1);
        } else if (gamepad1.dpad_down) {
            climb_motor1.setPower(-1);
            climb_motor2.setPower(-1);
        }



    }
}

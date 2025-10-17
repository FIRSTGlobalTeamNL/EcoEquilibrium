package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.utility.InstantCommand;
import com.rowanmcalpin.nextftc.ftc.OpModeData;

public class climbMotors extends Subsystem {
    // BOILERPLATE
    public static final climbMotors INSTANCE = new climbMotors();

    // USER CODE
    private hardware.motors motor;

    public Command climb() {
        return new InstantCommand(() -> motor.setPower(1));
    }

    @Override
    public void initialize() {
        hardware.servos.intake.initServo(OpModeData.INSTANCE.getHardwareMap());
        motor = new hardware.motors("hook", DcMotorSimple.Direction.REVERSE, DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }
}

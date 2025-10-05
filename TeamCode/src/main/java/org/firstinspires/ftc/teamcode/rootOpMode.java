//package org.firstinspires.ftc.teamcode;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.qualcomm.hardware.lynx.LynxModule;
//import com.rowanmcalpin.nextftc.core.command.Command;
//import com.rowanmcalpin.nextftc.core.command.groups.ParallelDeadlineGroup;
//import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;
//import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
//import com.rowanmcalpin.nextftc.core.command.utility.NullCommand;
//import com.rowanmcalpin.nextftc.core.command.utility.conditionals.BlockingConditionalCommand;
//import com.rowanmcalpin.nextftc.core.command.utility.delays.Delay;
//import com.rowanmcalpin.nextftc.core.command.utility.delays.WaitUntil;
//import com.rowanmcalpin.nextftc.ftc.NextFTCOpMode;
//import com.rowanmcalpin.nextftc.ftc.OpModeData;
//import com.rowanmcalpin.nextftc.ftc.driving.MecanumDriverControlled;
//import com.rowanmcalpin.nextftc.ftc.gamepad.Button;
//import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;
//
//import org.firstinspires.ftc.teamcode.Subsystems.hardware;
//
//import java.util.List;
//
//public class rootOpMode extends NextFTCOpMode {
//    public rootOpMode() {
//        super(Arm.INSTANCE,
//                Camera.INSTANCE,
//                Elbow.INSTANCE,
//                IntakeClaw.INSTANCE,
//                OuttakeClaw.INSTANCE,
//                Slides.INSTANCE,
//                Wrist.INSTANCE
//        );
//    }
//    protected Command driverControlled;
//
//    protected void initOpMode() {
//
//        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
//
//        for (LynxModule hub : allHubs) {
//            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
//        }
//
//        OpModeData.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        OpModeData.telemetry.setMsTransmissionInterval(25);
//
//        FtcDashboard.getInstance().startCameraStream(hardware.camera, 30);
//
//        hardware.initSensors(hardwareMap);
//
//        hardware.motors.leftFront.initMotor(hardwareMap);
//        hardware.motors.leftBack.initMotor(hardwareMap);
//        hardware.motors.rightFront.initMotor(hardwareMap);
//        hardware.motors.rightBack.initMotor(hardwareMap);
//        MotorEx[] motors = new MotorEx[] {
//                new MotorEx(hardware.motors.leftFront.dcMotorEx),
//                new MotorEx(hardware.motors.rightFront.dcMotorEx),
//                new MotorEx(hardware.motors.leftBack.dcMotorEx),
//                new MotorEx(hardware.motors.rightBack.dcMotorEx)
//        };
//        driverControlled = new MecanumDriverControlled(motors, gamepadManager.getGamepad1());
//    }
//}
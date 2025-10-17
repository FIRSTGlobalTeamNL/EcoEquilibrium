package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

/**
 * This is the hardware class. You can use this as an interface for all your hardware reads and writes, so you can more easily manage and minimise them.
 * I hope that because of the static-ness of enumerators, this reduces NullPointExceptions.
 * If you save the motors and servo's in the order of their hardware ports, it dramatically saves time when your config gets wiped.
 * Also, because leftBack.getName is already visibly the name of leftBack, you don't need to have the name be "left_back", it can just be motor0 or something based on the port.
 * @author Dean van Beek - 3977 Stanislas Tech Team
 * @version 0.5, 28/12/2024 (untested)
 */
public class hardware {
    public static class mathFunctions{
        /**
         * This returns whether a specified value is within a second specified value by plus/minus a
         * specified accuracy amount.
         *
         * @param one first number specified.
         * @param two Second number specified.
         * @param accuracy the level of accuracy specified.
         * @return returns if the two numbers are within the specified accuracy of each other.
         */
        public static boolean roughlyEquals(double one, double two, double accuracy) {
            return (one < two + accuracy && one > two - accuracy);
        }

        /**
         * This returns whether a specified number is within a second specified number by plus/minus 0.0001.
         *
         * @param one first number specified.
         * @param two second number specified.
         * @return returns if a specified number is within plus/minus 0.0001 of the second specified number.
         */
        public static boolean roughlyEquals(double one, double two) {
            return roughlyEquals(one, two, 0.0001);
        }
    }
    //TODO: test if all these this.xxx are necessary

    public static boolean reduceHardwareCalls = true;

    /**
     * This enumerator contains all the motors, preferably in the order of their hardware ports.
     * They have a config name, direction and mode, which can all easily be initialized with initMotor.
     */
    public static class motors {

        /**
         * This creates a new motor, with a config name, a direction and a RunMode.
         * @param name the name you use in the config on the Driver Hub.
         * @param direction the direction you want the motor to spin in.
         * @param mode the RunMode in which the motor needs to run.
         */
        motors(String name, DcMotorSimple.Direction direction, DcMotorEx.RunMode mode) {
            this.name = name;
            this.direction = direction;
            this.mode = mode;
        }

        /**
         * This creates a new motor, with a config name, a direction and a RunMode defaulted as RUN_WITHOUT_ENCODER.
         * @see <a href="#motors(String, DcMotorSimple.Direction, DcMotor.Runmode)">the standard constructor.</a>
         */
        motors(String name, DcMotorSimple.Direction direction) {this(name, direction, DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);}

        /**
         * This creates a new motor, with a config name, a direction defaulted as FORWARD and a RunMode.
         * @see <a href="#motors(String, DcMotorSimple.Direction, DcMotor.Runmode)">the standard constructor.</a>

         */
        motors(String name, DcMotor.RunMode mode) {this(name, DcMotorSimple.Direction.FORWARD, mode);}

        /**
         * This creates a new motor, with a config name, a direction defaulted as FORWARD and a RunMode defaulted as RUN_WITHOUT_ENCODER.
         * @see <a href="#motors(String, DcMotorSimple.Direction, DcMotor.Runmode)">the standard constructor.</a>
         */
        motors(String name) {this(name, DcMotorSimple.Direction.FORWARD, DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);}

        // these are the variables each motors() has.
        //TODO: make function getCurrentPos and private the motor.
        public DcMotorEx dcMotorEx;

        private final String name;

        private final DcMotorSimple.Direction direction;

        private final DcMotorEx.RunMode mode;

        private double lastPower = 0;

        //TODO: documentation for everything below this.
        /**
         *
         * @return
         */
        public String getName() {return this.name;}

        /**
         *
         * @return
         */
        public DcMotorSimple.Direction getDirection() {return this.direction;}

        /**
         *
         * @return
         */
        public DcMotor.RunMode getMode() {return this.mode;}

        /**
         *
         * @return
         */
        public double getLastPower() {return this.lastPower;}

        /**
         * TODO: instead of adding HardwareMap here, which gets added from the OpMode to the Drivetrain to here, add it from OpMode once (because static class)
         * @param map
         */
        public void initMotor(HardwareMap map) {
            dcMotorEx = map.get(DcMotorEx.class, getName());
            dcMotorEx.setDirection(getDirection());
            dcMotorEx.setMode(getMode());
        }

        /**
         *
         * @param power
         */
        public void setPower(double power) {
            if (mathFunctions.roughlyEquals(power, getLastPower()) && reduceHardwareCalls) return;
            this.dcMotorEx.setPower(power);
            this.lastPower = power;
        }
    }

    /**
     *
     */
    public enum servos {
        accelerator("accelerator"),
        intake("intakeClaw", 600, 2400),
        outtakeClaw("claw", 600, 2400),
        wrist("wrist"),
        elbowRight("elbowRight");

        /**
         *
         * @param name
         * @param usLower
         * @param usUpper
         */
        servos(String name, int usLower, int usUpper) {this.name = name; PwmRange = new PwmControl.PwmRange(usLower,usUpper);}

        servos(String name){this(name, 500, 2500);}

        private final String name;

        private final PwmControl.PwmRange PwmRange;

        public ServoImplEx servo;

        private double lastPosition = 0;

        /**
         *
         * @return
         */
        public String getName() {return this.name;}

        /**
         *
         * @return
         */
        public double getLastPosition() {return this.lastPosition;}

        /**
         * TODO: documentation, link GM0
         * @param usLower
         * @param usUpper
         */
        public void setPwmRange(int usLower, int usUpper) {servo.setPwmRange(new PwmControl.PwmRange(usLower, usUpper));}

        /**
         *
         * @param map
         */
        public void initServo(HardwareMap map) {
            servo = map.get(ServoImplEx.class, getName());
            servo.setPwmRange(PwmRange);
        }

        /**
         * to reduce hardware calls
         * @param position
         */
        public void setServo(double position) {
            if (mathFunctions.roughlyEquals(position, getLastPosition()) && reduceHardwareCalls) return;
            servo.setPosition(position);
            this.lastPosition = position;
        }

        /**
         * Is a shorthand, but not usable for the differential or other combined servo positions.
         */
        public void setServo(servoPositions position){setServo(position.getPosition());}
    }

    /**
     *
     */
    public enum touchSensors {
        leftFrontBumper("leftFrontSensor"),
        rightFrontBumper("rightFrontSensor"),
        armDown("armDown"),
        armUp("armUp"),
        leftBackBumper("leftBackSensor"),
        rightBackBumper("rightBackSensor"),
        clawGrab("clawGrab"),
        clawScore("clawScore");

        /**
         *
         * @param name
         */
        touchSensors(String name) {this.name = name;}

        private final String name;

        private TouchSensor sensor;

        /**
         *
         * @return
         */
        public String getName() {return this.name;}

        /**
         *
         * @param map
         */
        public void initSensor(HardwareMap map) {
            sensor = map.get(TouchSensor.class, name);
        }

        public boolean pressed() {return sensor.isPressed();}
    }

    public static void initSensors(HardwareMap map) {
        for (touchSensors sensor : touchSensors.values()) {
            sensor.initSensor(map);
        }
    }

    /**
     *
     */
    public enum servoPositions {
        intakeGrip(.91),
        intakeRelease(.48),
        wristTransfer(.615),
        wristSampleCamera(.37),
        wristSpecimenCamera(.87),
        keepSlides(.5),
        releaseSlides(.0),
        outtakeGrip(.31),
        outtakeRelease(.65),
        outtakeClear(.9);

        servoPositions(double position) {this.position = position;}
        private final double position;
        public double getPosition(){return this.position;}
    }

//    public static OpenCvCamera camera;
//
//    public static void initCamera(HardwareMap map, OpenCvPipeline startPipeline) {
//        int cameraMonitorViewId = map.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", map.appContext.getPackageName());
//        camera = OpenCvCameraFactory.getInstance().createWebcam(map.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//
//        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                camera.startStreaming(CAMERA_WIDTH,CAMERA_HEIGHT, OpenCvCameraRotation.UPSIDE_DOWN);
//                camera.setPipeline(startPipeline);
//            }
//
//            @Override
//            public void onError(int errorCode) {}
//        });
//    }
//
//    private static final int CAMERA_WIDTH = 640; // width  of wanted camera resolution
//    private static final int CAMERA_HEIGHT = 360; // height of wanted camera resolution
}

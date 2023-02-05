package org.firstinspires.ftc.teamcode.discoduckbots.hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.Encoder;

public class HardwareStore {
    private MecanumDrivetrain mecanumDrivetrain;
    private ConeArm coneArm;
    private IMU imu;
    private ColorSensor colorSensor = null;
    private TouchSensor touchSensor = null;
    private DistanceSensor distanceSensor = null;

    private WebcamName webcam = null;
    public TouchSensor turretSensor;
    public DcMotorEx frontLeft ;
    public DcMotorEx frontRight ;
    public DcMotorEx backRight ;
    public DcMotorEx backLeft ;
    public DcMotor coneTurret;
    public DcMotor coneLift;
    public Servo coneGrabber;
    public RevBlinkinLedDriver ledDriver;

    public HardwareStore(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode opMode) {
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
         frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
         backRight = hardwareMap.get(DcMotorEx.class, "backRight");
         backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
         coneTurret = hardwareMap.get(DcMotor.class, "coneTurret");
         coneLift = hardwareMap.get(DcMotor.class, "coneLift");
         coneGrabber= hardwareMap.get(Servo.class, "coneGrabber");
        ledDriver = hardwareMap.get(RevBlinkinLedDriver.class, "led");
        ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);


        // frontRight.setDirection(DcMotorEx.Direction.FORWARD);
       //  frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
        turretSensor = hardwareMap.get(TouchSensor.class, "resetSensor");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        //distanceSensor2 = hardwareMap.get(DistanceSensor.class, "distanceSensorRight");



        //BNO055IMU gyro = hardwareMap.get(BNO055IMU.class, "imu");
       //imu = new IMU(gyro);
       //imu.initialize();
        //hardwareMap.get(DcMotorEx.class, "leftEncoder").setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //hardwareMap.get(DcMotorEx.class, "rightEncoder").setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //hardwareMap.get(DcMotorEx.class, "backLeft").setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


       //mecanumDrivetrain = createDrivetrain(telemetry, opMode, imu, colorSensor, frontLeft, frontRight, backLeft, backRight);
    }
/*
    protected MecanumDrivetrain createDrivetrain(Telemetry telemetry,
                                 LinearOpMode opMode,
                                 IMU imu,
                                 ColorSensor colorSensor,
                                                 DcMotorEx frontLeft,
                                                 DcMotorEx frontRight,
                                                 DcMotorEx backLeft,
                                                 DcMotorEx backRight){
        return new MecanumDrivetrain(telemetry, opMode, imu, colorSensor, frontLeft, frontRight, backLeft, backRight);
    }
*/
    public MecanumDrivetrain getMecanumDrivetrain() {
        return mecanumDrivetrain;
    }


    public IMU getImu(){
        return imu;
    }

    public ColorSensor getColorSensor(){
        return colorSensor;
    }

    //public TouchSensor getArmStoppingSensor() { return armStoppingSensor; }
    public DistanceSensor getDistanceSensor() { return distanceSensor; }

    public TouchSensor getTurretSensor() { return turretSensor; }


    public ConeArm getConeArm() { return coneArm;}

    public WebcamName getWebcam() {return webcam;}

   public RevBlinkinLedDriver getLedDriver() { return ledDriver;}

    public DcMotor getConeLift() {
        return coneLift;
    }

    public DcMotor getConeTurret() {
        return coneTurret;
    }

    public Servo getConeGrabber() {
        return coneGrabber;
    }
}

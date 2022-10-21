package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Robot class should contain all of your robot subsystems.
 * This class can then be instanced in all of your teleOp and Auton
 * classes to give consistent behavior across all of them.
 */
public class Arm {
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    private DcMotorEx elevator;

    private Servo grabby;
    private Servo rotate;

    private double ROTATE_FRONT = 0;
    private double ROTATE_BACK = 0.6;

    private double GRABBY_OPEN = 0;
    private double GRABBY_CLOSE = 0.6;

    public Arm(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        elevator = hardwareMap.get(DcMotorEx.class,"elevator");

        grabby = hardwareMap.get(Servo.class,"grabby");
        rotate = hardwareMap.get(Servo.class,"rotate");

        elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void initGrabber() {
        grabby.setPosition(GRABBY_CLOSE);
        rotate.setPosition(ROTATE_FRONT);
    }

}

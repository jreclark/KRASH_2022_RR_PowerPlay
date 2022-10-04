package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.Drive;

/**
 * Robot class should contain all of your robot subsystems.
 * This class can then be instanced in all of your teleOp and Auton
 * classes to give consistent behavior across all of them.
 */
public class Robot {
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    public Drive drive;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        drive = new Drive(hardwareMap, telemetry);
    }

}

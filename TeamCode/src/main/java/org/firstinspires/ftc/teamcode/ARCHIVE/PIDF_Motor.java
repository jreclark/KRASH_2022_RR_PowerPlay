package org.firstinspires.ftc.teamcode.ARCHIVE;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PIDF_Motor {
    Telemetry telemetry;
    DcMotorEx motor;
    private double kP, kI, kD;
    private double kG;  //Gravity compensation term between 0 and 1

    // specify coefficients/gains
    PIDCoefficients coeffs;
    // create the controller
    PIDFController controller;

    PIDF_Motor(DcMotorEx motor, PIDCoefficients pidCoeffs, double kG, Telemetry telemetry) {
        this.coeffs = pidCoeffs;
        // create the controller
        this.controller = new PIDFController(coeffs, 0, 0, 0, (x, v) -> kG);
        this.controller.setOutputBounds(-1.0, 1.0);

        this.telemetry = telemetry;
        this.motor = motor;
    }


    public void gotoSetpoint(int targetPosition) {
        controller.setTargetPosition((double) targetPosition);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double output = controller.update(motor.getCurrentPosition());

        motor.setPower(output);
    }

    public void holdCurrentPos(){
        controller.setTargetPosition(motor.getCurrentPosition());
    }

}

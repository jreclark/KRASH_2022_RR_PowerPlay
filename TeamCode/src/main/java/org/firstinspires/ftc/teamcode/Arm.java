package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * Robot class should contain all of your robot subsystems.
 * This class can then be instanced in all of your teleOp and Auton
 * classes to give consistent behavior across all of them.
 */
public class Arm {
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    private ElapsedTime timer = new ElapsedTime();
    private double m_timeout = 0;

    private DcMotorEx elevator;

    private Servo grabby;
    private Servo rotate;

    private double ROTATE_FRONT = 0;
    private double ROTATE_BACK = 0.6;

    private double GRABBY_OPEN = 0;
    private double GRABBY_CLOSE = 0.6;

    //Elevator PIDF Values
    private static final double KV = 0.1;

    //Elevator positions
    public static enum ElevatorPositions{
        HIGH(1500),
        MID(1000),
        SHORT(800),
        LOW(500),
        PIVOT_POINT(800),
        START_AUTO_GRAB(800),
        START_GROUND_GRAB(600),
        DELTA_DROP(100);

        public final int position;

        private ElevatorPositions(int position){
            this.position = position;
        }
    }

    public Arm(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        elevator = hardwareMap.get(DcMotorEx.class,"elevator");

        grabby = hardwareMap.get(Servo.class,"grabby");
        rotate = hardwareMap.get(Servo.class,"rotate");

        elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients origPIDF = elevator.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        elevator.setVelocityPIDFCoefficients(origPIDF.p, origPIDF.i, origPIDF.d, KV);
    }


    /***********************************
     * GRABBER METHODS
     ***********************************/

    public void initGrabber() {
        grabby.setPosition(GRABBY_CLOSE);
        rotate.setPosition(ROTATE_FRONT);
    }

    /***********************************
     * LIFT METHODS
     ***********************************/

    public void runElevator(double power){
        elevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elevator.setPower(power);
    }

    public void elevatorPositionControl(int position){
        elevator.setTargetPosition(position);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(1.0);
    }

    public void elevatorPositionByConstant(ElevatorPositions constant){
        elevatorPositionControl(constant.position);
    }

    public void holdElevator(){
        elevatorPositionControl(elevator.getCurrentPosition());
    }

    public int getElevatorPosition(){
        return elevator.getCurrentPosition();
    }

    public boolean elevatorIsBusy() {
        return elevator.isBusy();
    }

    public double getElevatorPower(){
        return elevator.getPower();
    }

    public double getElevatorCurrent(){
        return elevator.getCurrent(CurrentUnit.AMPS);
    }


    /*********************************************************************************
     * Following commented out in favor of PIDF methods above.
     *********************************************************************************/
//    public void elevatorToPosition(double power, int targetPosition, double timeout){
//        elevator.setTargetPosition(targetPosition);
//        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        if(elevator.getCurrentPosition() >= targetPosition){
//            power = Math.abs(power);
//        } else {
//            power = -Math.abs(power);
//        }
//
//        m_timeout = timeout;
//        elevator.setPower(power);
//        timer.reset();
//    }
//
//    public boolean elevatorDoneMoving(){
//        if(elevator.isBusy() && timer.seconds()<m_timeout){
//            return false;
//        } else {
//            elevator.setPower(0);
//            return true;
//        }
//    }


}

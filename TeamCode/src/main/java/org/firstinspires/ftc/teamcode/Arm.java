package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DigitalChannel;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.util.Utils;

/**
 * Robot class should contain all of your robot subsystems.
 * This class can then be instanced in all of your teleOp and Auton
 * classes to give consistent behavior across all of them.
 */
@Config
public class Arm {
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    private ElapsedTime timer = new ElapsedTime();
    private ElapsedTime rotateTimer = new ElapsedTime();
    private ElapsedTime autoGrabTimer = new ElapsedTime();
    private double m_timeout = 0;
    private double autoGrabTimeout = 1.25;
    private double stackGrabTimeout = 2;
    private boolean stackGrabRunning = false;


    private DcMotorEx elevator;

    private Servo grabby;
    private Servo rotate;
    double ROTATE_DELAY = 1.0;

    private DigitalChannel coneSensor;


    public static double ROTATE_FRONT = 0.78;
    public static double ROTATE_BACK = 0.1;

    public static double GRABBY_OPEN = 0.0;
    public static double GRABBY_CLOSE = 1.0;

    //Elevator PIDF Values
    public static double KV = 0.1;
    public static double KP = 15;
    public static double KvP = 8;
    public static double KvD = 0;
    public static double KvI = 2.5;

    //Elevator positions
    public static enum ElevatorPositions {
        HIGH(3747),//5250
        MID(2750),//3700
        SHORT(1600),//2250
        LOW(214),//300
        PIVOT_POINT(1600),//2250
        START_AUTO_GRAB(1113),//1560
        FIRST_AUTO_GRAB(680),//950
        SECOND_AUTO_GRAB(540),//760
        START_GROUND_GRAB(500),//700
        DELTA_DROP(360);//500

        public final int position;

        private ElevatorPositions(int position) {
            this.position = position;
        }
    }

    public Arm(HardwareMap hardwareMap, Telemetry telemetry, boolean isTele) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        coneSensor = hardwareMap.get(DigitalChannel.class, "cone_sensor");
        coneSensor.setMode(DigitalChannel.Mode.INPUT);

        elevator = hardwareMap.get(DcMotorEx.class, "elevator");

        grabby = hardwareMap.get(Servo.class, "grabby");
        rotate = hardwareMap.get(Servo.class, "rotate");

        elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if(!isTele) elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        setPIDFValues();
//        PIDFCoefficients origPIDF = elevator.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
//        KP = elevator.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION).p;
//        KvP = origPIDF.p;
//        KvI = origPIDF.i;
//        KvD = origPIDF.d;
//        elevator.setVelocityPIDFCoefficients(origPIDF.p, origPIDF.i, origPIDF.d, KV);
    }


    public void groundGrab() {
        elevatorPositionControl(25);
        if (elevator.getCurrentPosition() < 50) {
            setGrabberClosed();
        } else {
            setGrabberOpen();
        }
    }


    public void stackFirstGrab(){
        setGrabberOpen();
        elevatorPositionByConstant(ElevatorPositions.FIRST_AUTO_GRAB);
        autoGrabTimer.reset();

        while(autoGrabTimer.seconds() <= autoGrabTimeout){

        }
        setGrabberClosed();
        Utils.sleep(1500);
        elevatorPositionByConstant(ElevatorPositions.HIGH);
        Utils.sleep(1000);
    }

    public void stackSecondGrabbyandHold(){
        setGrabberOpen();
        elevatorPositionByConstant(ElevatorPositions.SECOND_AUTO_GRAB);
        autoGrabTimer.reset();

        while(autoGrabTimer.seconds() <= autoGrabTimeout){

        }
        setGrabberClosed();
        Utils.sleep(1250);
        elevatorPositionByConstant(ElevatorPositions.PIVOT_POINT);
        Utils.sleep(1000);
    }

    public void stackSecondGrab(){
        setGrabberOpen();
        elevatorPositionByConstant(ElevatorPositions.SECOND_AUTO_GRAB);
        autoGrabTimer.reset();

        while(autoGrabTimer.seconds() <= autoGrabTimeout){

        }
        setGrabberClosed();
        Utils.sleep(2000);
        elevatorPositionByConstant(ElevatorPositions.HIGH);
        Utils.sleep(1000);
    }


    public void startAutoStackGrab(){
        autoGrabTimer.reset();
        stackGrabRunning = true;
        setGrabberOpen();
        if(!coneSensor.getState() && getElevatorPosition() >= 0.9 * ElevatorPositions.START_AUTO_GRAB.position){
            runElevator(-0.2);
        }
    }

    public boolean autoGrabIsBusy(){
        if(!coneSensor.getState() && !autoGrabTimeoutDone() && getElevatorPosition() >= 25){
            return true;
        } else {
            runElevator(0.05);
            return false;
        }
    }

    public boolean autoGrabTimeoutDone(){
        return autoGrabTimer.seconds() > stackGrabTimeout;
    }

    public void autoGrabFinish(){
        setGrabberClosed();
        Utils.sleep(1000);
        stackGrabRunning = false;
    }

    public boolean getStackGrabRunning(){
        return stackGrabRunning;
    }

    public boolean getConeSwitch(){
        return coneSensor.getState();
    }

    /***********************************
     * GRABBER METHODS
     ***********************************/

    public void initGrabber() {
        grabby.setPosition(GRABBY_CLOSE);
        rotate.setPosition(ROTATE_FRONT);
    }

    public void setGrabberOpen() {
        grabby.setPosition(GRABBY_OPEN);
    }

    public void setGrabberClosed() {
        grabby.setPosition(GRABBY_CLOSE);
    }

    public void setRotateFront() {
        if (rotate.getPosition() != ROTATE_FRONT) {
            rotateTimer.reset();
        }
        rotate.setPosition(ROTATE_FRONT);
    }

    public void setRotateBack() {
        if (rotate.getPosition() != ROTATE_BACK) {
            rotateTimer.reset();
        }
        rotate.setPosition(ROTATE_BACK);
    }

    public boolean isRotateFront() {
        double pos = rotate.getPosition();
        if (pos == ROTATE_FRONT && rotateTimer.seconds() >= ROTATE_DELAY) {
            return true;
        } else {
            return false;
        }
    }

    public boolean isRotateBack() {
        double pos = rotate.getPosition();
        if (pos == ROTATE_BACK && rotateTimer.seconds() >= ROTATE_DELAY) {
            return true;
        } else {
            return false;
        }
    }

    /***********************************
     * ELEVATOR METHODS
     ***********************************/

    public void runElevator(double power) {
        int pos = elevator.getCurrentPosition();

        if (power > 0 && pos > .8 * ElevatorPositions.HIGH.position) {
            power = power * 0.5;
        } else if (power > 0 && pos >= ElevatorPositions.HIGH.position) {
            power = 0;
        } else if (power < 0 && pos < 800 && pos > 0) {
            power = power * 0.25;
        } else if (power < 0 && pos <= 0) {
            power = 0;
        }
        elevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elevator.setPower(power);
    }

    public void elevatorPositionControl(int position) {
        elevator.setTargetPosition(position);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(1.0);
    }

    public void elevatorPositionByConstant(ElevatorPositions constant) {
        elevatorPositionControl(constant.position);
    }

    public void holdElevator() {
        elevatorPositionControl(elevator.getCurrentPosition());
    }

    public int getElevatorPosition() {
        return elevator.getCurrentPosition();
    }

    public boolean elevatorIsBusy() {
        return elevator.isBusy();
    }

    public double getElevatorPower() {
        return elevator.getPower();
    }

    public double getElevatorCurrent() {
        return elevator.getCurrent(CurrentUnit.AMPS);
    }

    public boolean isSafeToRotate() {
        if (elevator.getCurrentPosition() > 0.96 * ElevatorPositions.PIVOT_POINT.position) {
            return true;
        } else {
            return false;
        }
    }

    public void telemetryPIDF() {
        telemetry.addData("KP", KP);
        telemetry.addData("KvP", KvP);
        telemetry.addData("KvI", KvI);
        telemetry.addData("KvD", KvD);
    }

    public void setPIDFValues() {
        elevator.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(KP, 0, 0, 0));
        elevator.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(KvP, KvI, KvD, KV));
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

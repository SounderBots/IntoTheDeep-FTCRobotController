package org.firstinspires.ftc.teamcode.subsystems.delivery;

import android.util.Log;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.SonicSubsystemBase;
import org.firstinspires.ftc.teamcode.subsystems.feedback.DriverFeedback;
import org.firstinspires.ftc.teamcode.util.SonicPIDController;
import org.firstinspires.ftc.teamcode.util.Units;

import java.util.Set;

public class DeliveryPivot extends SonicSubsystemBase {

    private static final String LOG_TAG = LogTags.LOG_TAG_ARM_CONTROL;
    private Motor motor;

    private Telemetry telemetry;

    GamepadEx gamepad;

    private DriverFeedback feedback;

    private int StartPositionFromCalibration = 1975;

    private int DeliveryPositionFromStart = 250;

    private int IntakePositionFromStart = -1500;

    private int SampleIntakePositionFromStart = -1800;

    private int StartPositionFromStart = 0;


    private boolean isTeleop = true;

    private int currentTarget = 0;

    SonicPIDController pidController;

    private boolean checkLimit = false;

    public DeliveryPivot(HardwareMap hardwareMap, GamepadEx gamepad, Telemetry telemetry, DriverFeedback feedback) {
        /* instantiate motors */
        this.motor  = new Motor(hardwareMap, "DeliveryPivot");

        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.motor.encoder.reset();

        this.gamepad = gamepad;
        this.telemetry = telemetry;
        this.feedback = feedback;

        pidController = new SonicPIDController(0.005, 0, 0.000);
    }

    private void SetTelop() {
        this.isTeleop = true;
    }

    private void SetAuto() {
        this.isTeleop = false;
    }

    public void RotateTowardsIntake() {
        SetTelop();
        setMotorOutputWithLimit(-1);
    }

    public void RotateTowardsDelivery() {
        SetTelop();
        setMotorOutputWithLimit(1);
    }

    public void RotateTowardsIntakeSlowly() {
        SetTelop();
        setMotorOutputWithLimit(-.33);
    }

    public void RotateTowardsDeliverySlowly() {
        SetTelop();
        setMotorOutputWithLimit(-.33);
    }

    private boolean isCurrentPositionWithinLimit() {
        if (checkLimit) {
            final double currentPos = motor.encoder.getPosition();
            return currentPos > SampleIntakePositionFromStart && currentPos < DeliveryPositionFromStart;
        } else {
            return true;
        }
    }

    private void setMotorOutputWithLimit(double output) {
        if (isCurrentPositionWithinLimit()) {
            motor.set(output);
        }
    }

    public void HoldArm() {
        SetTelop();
        motor.set(0);
    }

    public void Calibrate() {
        SetAuto();
        this.motor.encoder.reset();
        this.currentTarget = StartPositionFromCalibration;
    }

    public void AutoToDelivery() {
        SetAuto();
        this.currentTarget = DeliveryPositionFromStart;
    }

    public void AutoToIntake() {
        SetAuto();
        this.currentTarget = IntakePositionFromStart;
    }

    public void AutoToStart() {
        SetAuto();
        this.currentTarget = 0;
    }

    @Override
    public void periodic() {
        super.periodic();

        double position = motor.encoder.getPosition();
        Log.i(LOG_TAG, "Pivot position: " + position);
        Log.i(LOG_TAG, "Pivot angle: " + Units.radiansToDegrees(currentAngleFromLevel()));

        if (!isCurrentPositionWithinLimit()) {
            motor.stopMotor();
            return;
        }
        //telemetry.addData("target", currentTarget);
        //telemetry.addData("current", position);
        //telemetry.addData("telop", isTeleop);
        if(!isTeleop) {
            double power = pidController.calculatePIDAlgorithm(currentTarget - position);
            //telemetry.addData("power", power);


            if(Math.abs(currentTarget - position) < 15) {
                //telemetry.addData("done", true);
                motor.set(0);
            }
            else {
                double minPower = .1;

                if(Math.abs(power) < minPower) {
                    //telemetry.addData("minPower", true);

                    power = minPower * Math.abs(power) / power;
                }

                motor.set(power);
            }
        }

        //telemetry.update();
    }

    public void MoveToIntakeInAuto() {
        MoveToPositionInAuto(IntakePositionFromStart);
    }

    public void MoveToDeliveryInAuto() {
        MoveToPositionInAuto(DeliveryPositionFromStart);
    }

    public void MoveToStartInAuto() {
        MoveToPositionInAuto(StartPositionFromStart);
    }


    public void MoveToPositionInAuto(double target) {
        while(true) {

            double position = motor.encoder.getPosition();

            double power = pidController.calculatePIDAlgorithm(target - position);

            if(Math.abs(target - position) < 40) {
                motor.set(0);
                break;
            }
            else {
                double minPower = .2;

                if (Math.abs(power) < minPower) {
                    //telemetry.addData("minPower", true);

                    power = minPower * Math.abs(power) / power;
                }

                motor.set(power);
            }

        }
    }

    public void MoveToIntakeSampleInAuto() {
        double position = motor.encoder.getPosition();

        motor.set(-0.5);
        while(position > SampleIntakePositionFromStart) {
            position = motor.encoder.getPosition();
        }

        motor.set(0);
    }

    public void setCheckLimit(boolean checkLimit) {
        this.checkLimit = checkLimit;
    }

    // assume IntakePositionFromStart is level, and DeliveryPositionFromStart is 90 degree
    public double currentAngleFromLevel() {
        final double currentPos = motor.encoder.getPosition();
        return (currentPos - IntakePositionFromStart) / (DeliveryPositionFromStart - IntakePositionFromStart) * Math.PI / 2;
    }
}
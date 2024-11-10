package org.firstinspires.ftc.teamcode.subsystems.delivery;

import android.util.Log;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.SonicSubsystemBase;
import org.firstinspires.ftc.teamcode.subsystems.feedback.DriverFeedback;
import org.firstinspires.ftc.teamcode.util.SonicPIDController;

public class DeliverySlider extends SonicSubsystemBase {

    private static final String LOG_TAG = LogTags.LOG_TAG_ARM_CONTROL;
    private Motor motor;

    private Telemetry telemetry;

    GamepadEx gamepad;

    private DriverFeedback feedback;

    public static final int BasketDeliveryPosition = -3300;
    public static final int CollapsedPosition = -100;

    private int currentTarget = 0;

    SonicPIDController pidController;

    private boolean isTeleop = true;

    private static final int shadowLimit = 2000;

    private double currentShadow = 0;

    private boolean overShadowLimit = false;
    public DeliverySlider(HardwareMap hardwareMap, GamepadEx gamepad, Telemetry telemetry, DriverFeedback feedback) {
        /* instantiate motors */
        this.motor  = new Motor(hardwareMap, "DeliverySlider");

        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        this.gamepad = gamepad;
        this.telemetry = telemetry;
        this.feedback = feedback;

        this.motor.encoder.reset();

        //MoveToTransferPosition();

        pidController = new SonicPIDController(0.08, 0, 0);
    }

    private void SetTelop() {
        this.isTeleop = true;
    }

    private void SetAuto() {
        this.isTeleop = false;
    }

    public void Expand() {
        SetTelop();
        motor.set(1);
    }

    public void Collapse() {
        SetTelop();
        overShadowLimit = false;
        motor.set(-1);
    }

    public void Hold() {
        SetTelop();
        motor.set(0);
    }

    public void MoveToDeliveryPosition() {
        SetAuto();
        currentTarget = BasketDeliveryPosition;
    }

    public void MoveToTransferPosition() {
        currentTarget = CollapsedPosition;
    }

    @Override
    public void periodic() {
        super.periodic();

        double position = motor.encoder.getPosition();

        Log.i(LOG_TAG, "Slider position: " + position);
        //telemetry.addData("target", currentTarget);
        //telemetry.addData("current", position);
        //telemetry.addData("telop", isTeleop);

        if(!isTeleop) {
            double power = pidController.calculatePIDAlgorithm(currentTarget - position);
            //telemetry.addData("power", power);


            if(Math.abs(currentTarget - position) < 40) {
                //telemetry.addData("done", true);
                motor.set(0);
            }
            else {
                double minPower = .2;

                if(Math.abs(power) < minPower) {
                    //telemetry.addData("minPower", true);

                    power = minPower * Math.abs(power) / power;
                }

                motor.set(power);
            }
        } else {

            // expanding and over shadow limit
            if (motor.get() > 0 && overShadowLimit) {
                motor.stopMotor();
            }
        }

        //telemetry.update();
    }

    public void ExtendMaxInAuto() {
        MoveToPositionInAuto(BasketDeliveryPosition);
    }

    public void CollapseMinInAuto() {
        MoveToPositionInAuto(CollapsedPosition);
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

                    power = minPower * Math.abs(power) / power;
                }

                motor.set(power);
            }
        }
    }

    public void updateCurrentShadowLimit(double currentAngle) {
        // if current angle is less than pi/4 (or 45 degree) then reset
        if (currentAngle > Math.PI / 4) {
            currentShadow = 0;
            overShadowLimit = false;
        } else {
            currentShadow = Math.abs(Math.cos(currentAngle) * motor.encoder.getPosition() - CollapsedPosition);
            overShadowLimit = currentShadow >= shadowLimit;
        }
    }

}
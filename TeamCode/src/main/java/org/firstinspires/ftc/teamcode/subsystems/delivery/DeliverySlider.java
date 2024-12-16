package org.firstinspires.ftc.teamcode.subsystems.delivery;

import android.util.Log;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.SonicSubsystemBase;
import org.firstinspires.ftc.teamcode.subsystems.feedback.DriverFeedback;
import org.firstinspires.ftc.teamcode.util.SonicPIDFController;

import java.util.function.Supplier;

public class DeliverySlider extends SonicSubsystemBase {

    private static final double FULL_POWER = 1d;
    private static final double SLOW_POWER = 0.5d;
    private static final double HOLD_POWER = .1d;

    public enum Direction {
        EXPAND (-1, 1),
        COLLAPSE (1, -1);

        final int motorDirection;
        final int motor2Direction;

        Direction(int motorDirection, int motor2Direction) {
            this.motorDirection = motorDirection;
            this.motor2Direction = motor2Direction;
        }
    }
    private Motor motor;
    private Motor motor2;


    private Telemetry telemetry;

    GamepadEx gamepad;

    private DriverFeedback feedback;

    public static int BasketDeliveryPosition = -2300;
    public static int CollapsedPosition = -90;

    public static int StartPosition = -400;

    private int ExtendLimit = -880;

    private int currentTarget = 0;

    SonicPIDFController pidController;

    private boolean isTeleop = true;

    public static double recordedPosition;

    private Supplier<Boolean> pivotLowEnoughSupplier;

    private Direction currentDirection;

    private DeliveryPivot deliveryPivot;

    private double pivotPositionAtStartOfExpansion = 0;
    private double positionAtStartOfExpansion = 0;
    private double clawHeightAtStartOfExpansion = 0;

    public DeliverySlider(HardwareMap hardwareMap, DeliveryPivot pivot, GamepadEx gamepad, Telemetry telemetry, DriverFeedback feedback) {
        /* instantiate motors */
        this.motor  = new Motor(hardwareMap, "Slider1");
        this.motor2  = new Motor(hardwareMap, "Slider2");

        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        this.gamepad = gamepad;
        this.telemetry = telemetry;
        this.feedback = feedback;

        this.motor.encoder.reset();

        //MoveToTransferPosition();

        pidController = new SonicPIDFController(0.01, 0, 0, 0.05);
        this.deliveryPivot = pivot;
    }

    private void SetTelop() {
        this.isTeleop = true;
    }

    private void SetAuto() {
        this.isTeleop = false;
    }

    public void collapse() {
        SetTelop();
        setMotors(SLOW_POWER, Direction.COLLAPSE);
        motor.set(.5);
        motor2.set(-.5);

    }

    public void expand() {
        SetTelop();
        pivotPositionAtStartOfExpansion = deliveryPivot.getCurrentPosition();
        positionAtStartOfExpansion = motor.getCurrentPosition();
        clawHeightAtStartOfExpansion = Math.sin(DeliveryPivot.getAngleOfPosition(pivotPositionAtStartOfExpansion)) * positionAtStartOfExpansion;
        if (pivotLowEnoughSupplier != null) {
            double limit = pivotLowEnoughSupplier.get() ? ExtendLimit : BasketDeliveryPosition;
            if (pivotLowEnoughSupplier.get()) {
                setMotors(SLOW_POWER, Direction.EXPAND);
            } else {
                setMotors(almostFullyExpanded(limit) ? SLOW_POWER : FULL_POWER, Direction.EXPAND);
            }
        } else {
            // default if no pivotLowEnoughSupplier
            setMotors(FULL_POWER, Direction.EXPAND);
        }
    }

    public void ExpandSlowly() {
        SetTelop();
        setMotors(SLOW_POWER, Direction.EXPAND);
    }

    public void CollapseSlowly() {
        SetTelop();
        setMotors(.3, Direction.COLLAPSE);
    }

    public void Hold() {
        SetTelop();
        if (Direction.EXPAND == currentDirection && almostFullyExpanded(BasketDeliveryPosition)) {
            setMotors(HOLD_POWER, currentDirection);
        } else {
            setMotors(0, currentDirection);
        }
    }

    public void MoveToValidPosition() {
        SetAuto();
        currentTarget = ExtendLimit + 50;
    }

    public void MoveToDeliverySpecimenPosition() {
        SetAuto();
        currentTarget = StartPosition;
    }

    public void MoveToDeliverySamplePosition() {
        SetAuto();
        currentTarget = BasketDeliveryPosition - 50;
    }

    public void MoveToCollapsedPosition() {
        currentTarget = CollapsedPosition;
    }

    @Override
    public void periodic() {
        super.periodic();

        double position = motor.encoder.getPosition();
        recordedPosition = position;
        Log.i("armControl", "slider position = " + position + ", action: " + (motor.get() > 0 ? "extend" : (motor.get() < 0 ? "Collapse" : "Stop")) );

        boolean addTelemetry = false;
        if(addTelemetry) {
            telemetry.addData("slider target", currentTarget);
            telemetry.addData("slider current", position);
            telemetry.update();
        }

        if(!isTeleop) {
            double power = pidController.calculatePIDAlgorithm(currentTarget - position);

            if(Math.abs(currentTarget - position) < 40) {
                setMotors(0);
            }
            else {
                double minPower = 0;

                if(Math.abs(power) < minPower) {
                    power = minPower * Math.abs(power) / power;
                }

                setMotors(power);
            }
        } else {
            //Log.i("armControl", "low enough? " + pivotLowEnoughSupplier == null ? "null" : (pivotLowEnoughSupplier.get() ? "yes" : "no"));

            telemetry.addData("slider", position);
            //telemetry.addData("pivot supplier", pivotLowEnoughSupplier.get());
            telemetry.addData("motor", motor.get());
            telemetry.update();


            if (pivotLowEnoughSupplier != null
                    && pivotLowEnoughSupplier.get()
                    && Math.abs(motor.get()) > 0
                    && position < ExtendLimit) {
                setMotors(0);
                MoveToValidPosition();

            }

            if (position < BasketDeliveryPosition &&
                     Math.abs(motor.get()) > 0){
                motor.stopMotor();
                currentTarget = -BasketDeliveryPosition;
            }
        }

        //telemetry.update();
    }

    public void ExtendMaxInAuto() {
        MoveToPositionInAuto(BasketDeliveryPosition + 100);
    }

    public void ResetEncoder() {
        this.motor.encoder.reset();
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

    public void setPivotLowEnoughSupplier(Supplier<Boolean> pivotLowEnoughSupplier) {
        this.pivotLowEnoughSupplier = pivotLowEnoughSupplier;
    }

    public boolean isMotorStopped() {
        return motor.get() == 0;
    }

    public Motor getMotor() {
        return motor;
    }

    public SonicPIDFController getPidController() {
        return pidController;
    }

    public void setMotors(double power, Direction direction) {
        motor.set(Math.abs(power) * direction.motorDirection);
        motor2.set(Math.abs(power) * direction.motor2Direction);
        currentDirection = direction;
    }

    public void setMotors(double power) {
        motor.set(power);
        motor2.set(-power);
    }

    public int getMotorsPosition() {
        return (motor.getCurrentPosition() + motor2.getCurrentPosition()) / 2;
    }

    public boolean almostFullyExpanded(double limit) {
        return Math.abs(((double)getMotorsPosition())/limit) > 0.9;
    }
}
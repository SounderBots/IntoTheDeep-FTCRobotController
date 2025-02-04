package org.firstinspires.ftc.teamcode.subsystems.intake;

import android.util.Log;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.SonicSubsystemBase;
import org.firstinspires.ftc.teamcode.subsystems.feedback.DriverFeedback;
import org.firstinspires.ftc.teamcode.subsystems.delivery.DeliveryPivot;
import org.firstinspires.ftc.teamcode.subsystems.delivery.DeliverySlider;

public class RollingIntake extends SonicSubsystemBase {

    private CRServo leftServo;

    private CRServo rightServo;

    private Servo elbowServo;

    NormalizedColorSensor colorSensor;

    private Telemetry telemetry;

    GamepadEx gamepad;

    private DriverFeedback feedback;

    private IntakeState state;

    private enum IntakeState { Hold, Intake, Outtake, IntakeAuto }

    public RollingIntake(HardwareMap hardwareMap, GamepadEx gamepad, Telemetry telemetry, DriverFeedback feedback) {
        /* instantiate motors */
        this.rightServo  = hardwareMap.get(CRServo.class,"RightIntake");
        this.leftServo  = hardwareMap.get(CRServo.class,"LeftIntake");

        this.elbowServo = hardwareMap.get(Servo.class, "Elbow");

        this.colorSensor = hardwareMap.get(NormalizedColorSensor.class, "ColorSensor");

        this.gamepad = gamepad;
        this.telemetry = telemetry;
        this.feedback = feedback;

        this.elbowServo.setPosition(1);
        this.SetElbowInInStart();

        state = IntakeState.Hold;
    }

    @Override
    public void periodic() {
        super.periodic();

        double d = GetDepth();

        boolean addData = false;

        if(addData) {
            telemetry.addData("distance", d);
            telemetry.addData("IsDelivery", this.isInDeliveryPosition);
        }

        if(d < 50) {
            if (feedback != null) {
                feedback.TurnLedGreen();
            }
        } else {
            if (feedback != null) {
                feedback.TurnLedRed();
            }
        }

        if(state == IntakeState.Intake || state == IntakeState.IntakeAuto) {
            if(addData) {
                telemetry.addData("State", "Intake");
            }

            if(d > 50 || this.isInDeliveryPosition) {
                if(addData) {
                    telemetry.addData("power", 1);
                }

                if(state == IntakeState.Intake) {
                    this.leftServo.setPower(-1);
                    this.rightServo.setPower(1);
                } else if(state == IntakeState.IntakeAuto) {
                    this.leftServo.setPower(-.3);
                    this.rightServo.setPower(.3);
                }

            } else {
                if(state == IntakeState.Intake | state == IntakeState.IntakeAuto) {
                    this.leftServo.setPower(-0.4);
                    this.rightServo.setPower(0.4);
                    Sleep(40);

                } else {
                    this.leftServo.setPower(0);
                    this.rightServo.setPower(0);
                }

                if(addData) {
                    telemetry.addData("power", 0);
                }

                if (feedback != null) {
                    feedback.DriverRumbleBlip();
                    feedback.OperatorRumbleBlip();
                }

                state = IntakeState.Hold;
            }
        } else if (state == IntakeState.Outtake) {
            if(addData) {
                telemetry.addData("State", "Outtake");
            }

            if(d > 50) {
                if (feedback != null) {
                    feedback.DriverRumbleBlip();
                    feedback.OperatorRumbleBlip();
                }
            }
            this.leftServo.setPower(1);
            this.rightServo.setPower(-1);

        } else {

            if(addData) {
                telemetry.addData("State", "Hold");
                telemetry.addData("Pivot", DeliveryPivot.recordedPosition);
                telemetry.addData("Slider", DeliverySlider.recordedPosition);
            }

            if (DeliveryPivot.recordedPosition > 1000 && DeliverySlider.recordedPosition < -2200) {
                SetElbowInSampleDeliveryPosition();
            }

            this.leftServo.setPower(0);
            this.rightServo.setPower(0);
        }

        if(addData) {
            telemetry.update();
        }
    }

    public void Sleep(long timeout) {
        try {
            synchronized (this) {
                wait(timeout);
            }
        } catch (java.lang.InterruptedException e) {
        }
    }

    public boolean IsSampleIntaken() {
        double d = GetDepth();
        return d < 50;
    }

    boolean isInDeliveryPosition = false;

    public void SetInDeliveryPositionn(boolean isInDeliveryPosition) {
        this.isInDeliveryPosition = isInDeliveryPosition;
    }

    public void SetElbowInSpecimenDeliveryPosition() {
        this.elbowServo.setPosition(0.15);
    }

    public void SetElbowInSampleDeliveryPosition() {
        this.elbowServo.setPosition(0.4);
    }

    public void SetElbowInSampleSweepPosition() {
        this.elbowServo.setPosition(0.01);
    }

    public void SetElbowInIntakePosition() {
        setElbowToPosition(.735);
    }

    public void SetElbowInIntakePositionForSample3() {
        setElbowToPosition(.85);
    }

    public void setElbowToPosition(double position) {
        this.elbowServo.setPosition(position);
    }

    public void SetElbowInInStart() {
        this.elbowServo.setPosition(1);
    }

    boolean isElbowInIntake = true;

    public void ToggleElbowPosition() {
        if(isElbowInIntake) {
            SetElbowInSampleDeliveryPosition();
        } else {
            SetElbowInIntakePosition();
        }

        isElbowInIntake = !isElbowInIntake;
    }

    int elbowTogglePosition = 0;
    public void ToggleElbowAcrossAll() {

        telemetry.addData("elbow position", elbowTogglePosition % 4 );

        switch (elbowTogglePosition % 4) {
            case 0:
                SetElbowInInStart();
                break;
            case 1:
                SetElbowInIntakePosition();
                break;
            case 2:
                SetElbowInSampleDeliveryPosition();
                break;
            case 3:
                SetElbowInSampleSweepPosition();
                break;
        }

        elbowTogglePosition++;
    }

    public void Intake() {
        state = IntakeState.Intake;
    }

    public void Outtake() {
        state = IntakeState.Outtake;
    }

    public void Hold(){
        state = IntakeState.Hold;
    }

    public double GetDepth() {
        if (colorSensor instanceof DistanceSensor) {
            double depth = ((DistanceSensor) colorSensor).getDistance(DistanceUnit.MM);
            //telemetry.addData("Left distance (mm)", "%.3f", depth);
            return depth;
        }

        return 1000000;
    }

    public void IntakeInAuto() {
        this.state = IntakeState.IntakeAuto;
    }

    public void OuttakeInAuto() {
        this.leftServo.setPower(1);
        this.rightServo.setPower(-1);
    }

    public void IntakeSlowlyInAuto() {
        this.leftServo.setPower(-1);
        this.rightServo.setPower(1);
    }

    public void OuttakeSlowlyInAuto() {
        this.leftServo.setPower(.7);
        this.rightServo.setPower(-.7);
    }

    public void HoldInAuto() {
        telemetry.addLine("Holding out/intake");
        telemetry.update();

        this.leftServo.setPower(0);
        this.rightServo.setPower(0);
    }
}
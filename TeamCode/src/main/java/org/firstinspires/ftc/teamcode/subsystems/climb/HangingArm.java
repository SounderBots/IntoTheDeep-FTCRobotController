package org.firstinspires.ftc.teamcode.subsystems.climb;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.SonicSubsystemBase;
import org.firstinspires.ftc.teamcode.subsystems.feedback.DriverFeedback;
import org.firstinspires.ftc.teamcode.util.SonicPIDFController;

public class HangingArm extends SonicSubsystemBase {

    private static final String LOG_TAG = HangingArm.class.getSimpleName();

    private Motor motorLeft;

    private Telemetry telemetry;

    GamepadEx gamepad;

    private DriverFeedback feedback;

    SonicPIDFController leftPidController;
    SonicPIDFController rightPidController;



    public HangingArm(HardwareMap hardwareMap, Telemetry telemetry, GamepadEx gamepad, DriverFeedback feedback) {
        this.motorLeft = new Motor(hardwareMap, "HangArmLeft");
        this.telemetry = telemetry;
        this.gamepad = gamepad;
        this.feedback = feedback;

        leftPidController = new SonicPIDFController(0.08, 0, 0);
        rightPidController = new SonicPIDFController(0.08, 0, 0);
    }

    public void extend() {
        motorLeft.set(1);
    }

    public void collapse() {
        motorLeft.set(-1);
    }

    public void hold() {
        this.motorLeft.set(0);
    }
}

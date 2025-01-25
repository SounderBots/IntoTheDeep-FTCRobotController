package org.firstinspires.ftc.teamcode.opmodes.autonomous.command;

import android.util.Log;

import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.delivery.DeliverySlider;
import org.firstinspires.ftc.teamcode.util.SonicPIDFController;

public class MoveSliderCommand2 extends SounderBotCommandBase {

    private static final String LOG_TAG = MoveSliderCommand2.class.getSimpleName();
    public static final double HOLD_POWER = .2;

    DeliverySlider slider;
    Telemetry telemetry;

    double target;

    Motor motor;

    SonicPIDFController pidController;

    double position;

    boolean resetEncoder = false;

    DeliverySlider.Direction direction;

    boolean hold = false;

    public MoveSliderCommand2(DeliverySlider slider, Telemetry telemetry, double target, DeliverySlider.Direction direction) {
        this(slider, telemetry, target, false, direction);
    }

    public MoveSliderCommand2(DeliverySlider slider, Telemetry telemetry, double target, DeliverySlider.Direction direction, boolean hold) {
        this(slider, telemetry, target, false, direction, 800, hold);
    }

    public MoveSliderCommand2(DeliverySlider slider, Telemetry telemetry, double target, boolean resetEncoder, DeliverySlider.Direction direction) {
        this(slider, telemetry, target, resetEncoder, direction, 800);
    }

    public MoveSliderCommand2(DeliverySlider slider, Telemetry telemetry, double target, boolean resetEncoder, DeliverySlider.Direction direction, int timeout) {
        this(slider, telemetry, target, resetEncoder, direction, timeout, false);
    }

    public MoveSliderCommand2(DeliverySlider slider, Telemetry telemetry, double target, boolean resetEncoder, DeliverySlider.Direction direction, int timeout, boolean hold) {
        super(timeout);

        this.slider = slider;
        this.telemetry = telemetry;
        this.target = target;
        this.motor = slider.getMotor();
        this.pidController = slider.getPidController();
        this.resetEncoder = resetEncoder;
        this.direction = direction;
        this.hold = hold;

        Log.i(LOG_TAG, "Target set to: " + target + ", direction = " + direction.name());
        addRequirements(slider);
    }


    @Override
    public void doExecute() {
        position = motor.encoder.getPosition();
        Log.i(LOG_TAG, String.format("Current position = %f, direction = %s, target = %f", position, direction.name(), target));
        double power = Math.abs(pidController.calculatePIDAlgorithm(target - position));

        if(isTargetReached()) {
            Log.i(LOG_TAG, "target reached with direction " + direction.name());
            if (hold) {
                slider.setMotors(HOLD_POWER * direction.directionFactor);
            }

        } else {
            double minPower = .2;

            power = Math.max(minPower, Math.abs(power)) * direction.directionFactor;

            telemetry.addData("slider position", position);
            telemetry.addData("slider target", target);
            telemetry.addData("slider power", power);


            Log.i(LOG_TAG, "Power to motor: " + power);
            slider.setMotors(power);
        }

        telemetry.update();
    }

    @Override
    protected boolean isTargetReached() {
        return Math.abs(target - position) < 40;
    }

    @Override
    public void end(boolean interrupted) {
        if (DeliverySlider.Direction.EXPANDING == direction) {
            slider.setMotors(-.1);
        } else {
            slider.setMotors(0);
        }
    }
}

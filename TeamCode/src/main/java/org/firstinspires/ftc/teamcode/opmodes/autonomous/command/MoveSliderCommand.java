package org.firstinspires.ftc.teamcode.opmodes.autonomous.command;

import android.util.Log;

import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.Supplier;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.delivery.DeliverySlider;
import org.firstinspires.ftc.teamcode.util.SonicPIDFController;

public class MoveSliderCommand extends SounderBotCommandBase {

    private static final String LOG_TAG = MoveSliderCommand.class.getSimpleName();
    public static class EndAction {
        public static final double DEFAULT_END_POWER = .2;
        public double endPowerWithoutSign;
        Supplier<Boolean> stopMotorSignalProvider;

        public EndAction(double endPowerWithoutSign, Supplier<Boolean> stopMotorSignalProvider) {
            this.endPowerWithoutSign = endPowerWithoutSign;
            this.stopMotorSignalProvider = stopMotorSignalProvider;
        }

        public EndAction(Supplier<Boolean> stopMotorSignalProvider) {
            this(DEFAULT_END_POWER, stopMotorSignalProvider);
        }
    }
    DeliverySlider slider;
    Telemetry telemetry;

    double target;

    Motor motor;

    SonicPIDFController pidController;

    double position;

    EndAction endAction;

    int holdPosition = Integer.MIN_VALUE;

    double holdingPower = 0;

    boolean resetEncoder = false;

    int timeout = 4000;

    boolean addTelemetry = false;

    DeliverySlider.Direction direction;

    private double expandHoldPower = -.2;

    public MoveSliderCommand(DeliverySlider slider, Telemetry telemetry, double target, DeliverySlider.Direction direction) {
        this(slider, telemetry, target, false, direction);
    }

    public MoveSliderCommand(DeliverySlider slider, Telemetry telemetry, double target, boolean resetEncoder, DeliverySlider.Direction directio) {
        this(slider, telemetry, target, resetEncoder, directio, 800);
    }

    public MoveSliderCommand(DeliverySlider slider, Telemetry telemetry, double target, boolean resetEncoder, DeliverySlider.Direction direction, int timeout) {
        this(slider, telemetry,target, resetEncoder, direction, timeout, false);
    }

    boolean holdPowerOnTargetReach;

    public MoveSliderCommand(DeliverySlider slider, Telemetry telemetry, double target, boolean resetEncoder, DeliverySlider.Direction direction, int timeout, boolean holdPowerAfterTargetReach) {
        super(timeout);

        this.slider = slider;
        this.telemetry = telemetry;
        this.target = target;
        this.motor = slider.getMotor();
        this.pidController = slider.getPidController();
        this.resetEncoder = resetEncoder;
        this.direction = direction;
        this.holdPowerOnTargetReach = holdPowerAfterTargetReach;

        Log.i(LOG_TAG, "Target set to: " + target + ", direction = " + direction.name());
        addRequirements(slider);
    }

    public MoveSliderCommand withExpandHoldPower(double expandHoldPower) {
        this.expandHoldPower = expandHoldPower;
        return this;
    }

    public MoveSliderCommand withEndAction(EndAction endAction) {
        this.endAction = endAction;
        return this;
    }

    @Override
    public void doExecute() {
        position = motor.encoder.getPosition();
        Log.i(LOG_TAG, String.format("Current position = %f, direction = %s, target = %f", position, direction.name(), target));
        double power = Math.abs(pidController.calculatePIDAlgorithm(target - position));

        if(isTargetReached() || holdPosition != Integer.MIN_VALUE) {
            Log.i(LOG_TAG, "target reached with direction " + direction.name());
            if (endAction == null) {
                Log.i(LOG_TAG, "no end action");
                end(false);
                finished = true;
                onFlagEnabled(addTelemetry, () -> telemetry.addLine("Done"));

                if(resetEncoder) {
                    this.slider.ResetEncoder();
                }
            } else {
                if (holdPosition == Integer.MIN_VALUE) {
                    holdPosition = motor.getCurrentPosition();
                    holdingPower = endAction.endPowerWithoutSign * Math.signum(power);
                }

                if (endAction.stopMotorSignalProvider.get()) {
                    slider.setMotors(0);
                    finished = true;
                } else {
                    onFlagEnabled(addTelemetry, () -> telemetry.addLine("holding slider..."));
                    if (Math.abs(holdPosition - motor.getCurrentPosition()) < 50) {
                        slider.setMotors(0);
                    } else {
                        slider.setMotors(holdingPower);
                    }
                }

            }

        } else {
            double minPower = .2;

            power = Math.max(minPower, Math.abs(power)) * direction.directionFactor;

            final double telePower = power;
            onFlagEnabled(addTelemetry, () -> {
                telemetry.addData("slider", position);
                telemetry.addData("target", target);
                telemetry.addData("power", telePower);
            });



            Log.i(LOG_TAG, "Power to motor: " + power);
            slider.setMotors(power);
        }

        telemetry.update();
    }

    @Override
    protected boolean isTargetReached() {
        if(holdPowerOnTargetReach) {
            return false;
        }

        return Math.abs(target - position) < 40;
    }

    @Override
    public void end(boolean interrupted) {
        if (DeliverySlider.Direction.EXPANDING == direction) {
            slider.setMotors(expandHoldPower);
        } else {
            slider.setMotors(0);
        }
    }
}

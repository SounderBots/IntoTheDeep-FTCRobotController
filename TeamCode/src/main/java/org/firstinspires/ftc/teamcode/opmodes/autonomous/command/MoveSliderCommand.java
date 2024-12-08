package org.firstinspires.ftc.teamcode.opmodes.autonomous.command;

import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.Supplier;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.delivery.DeliverySlider;
import org.firstinspires.ftc.teamcode.util.SonicPIDFController;

public class MoveSliderCommand extends SounderBotCommandBase {

    DeliverySlider slider;
    Telemetry telemetry;

    double target;

    Motor motor;

    SonicPIDFController pidController;

    double position;

    int holdPosition = Integer.MIN_VALUE;

    double holdingPower = 0;

    boolean resetEncoder = false;

    public MoveSliderCommand(DeliverySlider slider, Telemetry telemetry, double target) {
        this(slider, telemetry, target, false);
    }

    public MoveSliderCommand(DeliverySlider slider, Telemetry telemetry, double target, boolean resetEncoder) {
        this.slider = slider;
        this.telemetry = telemetry;
        this.target = target;
        this.motor = slider.getMotor();
        this.pidController = slider.getPidController();
        this.resetEncoder = resetEncoder;

        addRequirements(slider);
    }

    @Override
    public void doExecute() {
        position = motor.encoder.getPosition();
        double power = pidController.calculatePIDAlgorithm(target - position);

//        if(isTargetReached()) {
//            motor.set(0);
//            finished = true;
//            telemetry.addLine("Done");
//
//            if(resetEncoder) {
//                this.slider.ResetEncoder();
//            }
//
//        } else {
            double minPower = .2;
            if (Math.abs(power) < minPower) {
                power = minPower * Math.abs(power) / power;
            }

            telemetry.addData("slider", position);
            telemetry.addData("target", target);
            telemetry.addData("power", power);

            motor.set(power);
//        }

        telemetry.update();
    }

    @Override
    protected boolean isTargetReached() {
        return Math.abs(target - position) < 100;
    }

    @Override
    public void end(boolean interrupted) {
        motor.set(0);
        if (resetEncoder) {
            slider.ResetEncoder();
        }
        finished = true;
        telemetry.addLine("Done");
    }
}

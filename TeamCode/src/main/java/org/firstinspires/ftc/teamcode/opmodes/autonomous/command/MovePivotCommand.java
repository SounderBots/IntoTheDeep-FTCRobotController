package org.firstinspires.ftc.teamcode.opmodes.autonomous.command;

import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.delivery.DeliveryPivot;
import org.firstinspires.ftc.teamcode.util.SonicPIDController;

public class MovePivotCommand extends SounderBotCommandBase{

    DeliveryPivot pivot;
    Telemetry telemetry;
    double target;
    double position;
    Motor motor;
    SonicPIDController pidController;

    public MovePivotCommand(DeliveryPivot pivot, Telemetry telemetry, double target) {
        this.pivot = pivot;
        this.telemetry = telemetry;
        this.target = target;
        this.motor = pivot.getMotor();
        this.pidController = pivot.getPidController();
        addRequirements(pivot);
    }

    @Override
    protected boolean isTargetReached() {
        return Math.abs(target - position) < 40;
    }

    @Override
    public void end(boolean interrupted) {
        motor.set(0);
    }

    @Override
    public void execute() {
        position = motor.encoder.getPosition();
        double power = pidController.calculatePIDAlgorithm(target - position);
        if (isTargetReached()) {
            motor.set(0);
            finished.set(true);
        } else {
            double minPower = .2;

            if (Math.abs(power) < minPower) {
                //telemetry.addData("minPower", true);

                power = minPower * Math.abs(power) / power;
            }

            motor.set(power);
        }
    }
}
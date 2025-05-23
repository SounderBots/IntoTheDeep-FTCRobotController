package org.firstinspires.ftc.teamcode.opmodes.autonomous.command;

import com.google.common.util.concurrent.Uninterruptibles;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.AutoMecanumDriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.util.SonicPIDFController;

import java.util.concurrent.TimeUnit;

public class TurnAngleRelativeCommand extends SounderBotCommandBase {
    private static final String LOG_TAG = TurnAngleRelativeCommand.class.getSimpleName();
    double minPower = 0.2;

    AutoMecanumDriveTrain driveTrain;
    GoBildaPinpointDriver odo;
    Telemetry telemetry;

    double targetTurnInRadians;

    double targetAngleInDegrees;

    double error = Double.MAX_VALUE;

    SonicPIDFController pidController = new SonicPIDFController(0.5, 0, 0.02);

    boolean isFirst = true;


    public TurnAngleRelativeCommand(AutoMecanumDriveTrain driveTrain, Telemetry telemetry, double turnInDegrees) {
        this.driveTrain = driveTrain;
        this.telemetry = telemetry;
        this.odo = driveTrain.getOdo();

        this.targetAngleInDegrees = -1 * turnInDegrees;
    }

    @Override
    protected boolean isTargetReached() {
        return Math.abs(error) < Math.toRadians(1);
    }

    @Override
    public void doExecute() {
        odo.update();

        if(isFirst) {
            isFirst = false;
            targetTurnInRadians = odo.getHeading() + Math.toRadians(targetAngleInDegrees);
        }

        odo.update();
        error = targetTurnInRadians - odo.getHeading();

        if (isTargetReached()) {
            driveTrain.stop();
            sleep(100);

            odo.update();
            error = targetTurnInRadians - odo.getHeading();
            if (isTargetReached()) {
                finished = true;
                return;
            }
        }

        double power = pidController.calculatePIDAlgorithm(error);

        if(Math.abs(power) < minPower) {
            power = minPower * Math.signum(power);
        }

        driveTrain.driveRobotCentric(0, power, 0);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        driveTrain.stop();
    }
}

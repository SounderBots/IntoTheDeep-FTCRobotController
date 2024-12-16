package org.firstinspires.ftc.teamcode.opmodes.autonomous.base;

import org.firstinspires.ftc.teamcode.command.AutonomousCommand;
import org.firstinspires.ftc.teamcode.opmodes.OpModeTemplate;
import org.firstinspires.ftc.teamcode.subsystems.delivery.DeliveryPivot;
import org.firstinspires.ftc.teamcode.subsystems.delivery.DeliverySlider;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.AutoMecanumDriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.feedback.DriverFeedback;
import org.firstinspires.ftc.teamcode.subsystems.intake.RollingIntake;
import org.firstinspires.ftc.teamcode.subsystems.vision.LimeLight;

public abstract class AutoOpModeBase extends OpModeTemplate {
    protected AutoMecanumDriveTrain driveTrain;
    protected RollingIntake rollingIntake;

    protected LimeLight limeLight;

    protected DeliveryPivot pivot;

    protected DeliverySlider slider;

    protected DriverFeedback feedback;

    protected AutonomousCommand autonomousCommand;

//    @Override
//    public void reset() {
//        if (autonomousCommand != null) {
//            autonomousCommand.cancel();
//            autonomousCommand.end(true);
//        }
//
//        try {
//            if (rollingIntake != null) {
//                rollingIntake.HoldInAuto();
//                rollingIntake.SetElbowInInStart();
//            }
//            if (pivot != null) {
//                pivot.MoveToStartInAuto();
//            }
//            if (slider != null) {
//                slider.CollapseMinInAuto();
//            }
//            if (driveTrain != null) {
//                driveTrain.stop();
//            }
//
//            sleep(500);
//        } catch (Exception e) {
//            telemetry.addData("CleaupError",e.getMessage());
//            telemetry.update();
//        }
//        super.end();
//        super.reset();
//
//    }

    @Override
    public void reset() {
        // Cancel the autonomous command if it's still running
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }

        try {
            // Return all mechanisms to safe positions
            if (rollingIntake != null) {
                rollingIntake.HoldInAuto();
                rollingIntake.SetElbowInInStart();
            }

            if (pivot != null) {
                pivot.MoveToStartInAuto();
            }

            if (slider != null) {
                slider.CollapseMinInAuto();
            }

            if (driveTrain != null) {
                driveTrain.stop();
            }

            // Brief pause to allow mechanisms to reach safe positions
            sleep(500);

        } catch (Exception e) {
            telemetry.addData("Cleanup Error", e.getMessage());
            telemetry.update();
        }

        end();
        // Call parent reset to handle CommandScheduler cleanup
        super.reset();
    }

    @Override
    public void initialize() {
        super.initialize();

        feedback = new DriverFeedback(hardwareMap, driverGamepad, operatorGamepad, telemetry);
        rollingIntake = new RollingIntake(hardwareMap, operatorGamepad, telemetry, feedback);
        pivot = new DeliveryPivot(hardwareMap, operatorGamepad, telemetry, null, rollingIntake);
        slider = new DeliverySlider(hardwareMap, pivot, operatorGamepad, telemetry, null);
        limeLight = new LimeLight(hardwareMap, telemetry);
        driveTrain = new AutoMecanumDriveTrain(hardwareMap, driverGamepad, telemetry, null, limeLight);

        register(driveTrain, pivot, slider, rollingIntake, feedback, limeLight);

        autonomousCommand = new AutonomousCommand(this, driveTrain, limeLight, rollingIntake, pivot, slider, telemetry);
        schedule(autonomousCommand);

    }



    /*
        Callback method to be overridden by AutoOpMode subclass
     */
    public abstract void executeOpMode();
}
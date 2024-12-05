package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class OpModeTemplate extends CommandOpMode {

    protected GamepadEx driverGamepad;
    protected GamepadEx operatorGamepad;

    private FtcDashboard dashboard;
    @Override
    public void initialize() {
        try {
            // Check if dashboard is already started
            dashboard = FtcDashboard.getInstance();
            if (dashboard == null) {
                // Only start if not already running
                FtcDashboard.start(hardwareMap.appContext);
                dashboard = FtcDashboard.getInstance();
            }

            // Send a test telemetry packet to verify dashboard is working
            if (dashboard != null) {
                telemetry.addData("Dashboard", "Started successfully");
            } else {
                telemetry.addData("Dashboard", "Failed to start");
            }
            telemetry.update();

        } catch (Exception e) {
            telemetry.addData("Dashboard Error", e.getMessage());
            telemetry.update();
        }
        TelemetryPacket telemetryPacket = new TelemetryPacket(true);
        telemetryPacket.put("Status", getClass().getSimpleName() + " initializing...");
        driverGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);

         // update telemetry every loop
//        schedule(new RunCommand(telemetry::update));
    }

    public void end(boolean interrupted) {
        super.stop();
        // Release gamepad resources
        if (driverGamepad != null) {
            driverGamepad = null;
        }
        if (operatorGamepad != null) {
            operatorGamepad = null;
        }
        dashboard = null;
    }

    public void Wait(long timeout) {
        try {
            synchronized (this) {
                wait(timeout);
            }
        } catch (java.lang.InterruptedException e) {
        }
    }

    public void end() {
        super.stop();
//        if (driverGamepad != null) {
//            driverGamepad = null;
//        }
//        if (operatorGamepad != null) {
//            operatorGamepad = null;
//        }
    }
}

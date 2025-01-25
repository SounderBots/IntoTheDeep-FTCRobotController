package org.firstinspires.ftc.teamcode.opmodes.autonomous.runs;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.autonomous.base.CommandAutoOpMode;

@Autonomous
@Config
public class DriveToTargetTest extends CommandAutoOpMode {
    public static double minPower = .3;
    public static double maxPower = .8;
    public static double distanceTolerance = 80;

    @Override
    protected Command createCommand() {
        return new SequentialCommandGroup(
                commandFactory.driveToTarget(300, 0, 0, minPower, maxPower, distanceTolerance),
                commandFactory.driveToTarget(300, 300, 0, minPower, maxPower, distanceTolerance),
                commandFactory.sleep(30000)
        );
    }
}

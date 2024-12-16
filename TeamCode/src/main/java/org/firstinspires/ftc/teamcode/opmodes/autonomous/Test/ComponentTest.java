package org.firstinspires.ftc.teamcode.opmodes.autonomous.Test;

import com.arcrobotics.ftclib.command.Command;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.autonomous.base.CommandAutoOpMode;

@Autonomous
@SuppressWarnings("unused")
public class ComponentTest extends CommandAutoOpMode {
    @Override
    protected Command createCommand() {
        return commandFactory
                .pivotToDelivery()
                .andThen(commandFactory.extendSlider())
                .andThen(commandFactory.collapseSlider());
    }
}

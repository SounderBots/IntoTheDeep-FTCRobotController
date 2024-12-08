package org.firstinspires.ftc.teamcode.opmodes.autonomous.Test;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ScheduleCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.autonomous.base.CommandAutoOpMode;

@Autonomous
public class OdoTest extends CommandAutoOpMode {

    @Override
    protected Command createCommand() {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                 commandFactory.driveToTarget(300, 400, -45, .1, 100),
                 commandFactory.pivotToDelivery(),
                 commandFactory.elbowToSpecimenPosition(),
                commandFactory.extendSlider()
            ),

            commandFactory.driveToTarget(180, 520, -45, 0.1, 100),

            // Sample #1
            commandFactory.extendSlider(commandFactory.outtake()),
            commandFactory.driveToTarget(430, 320, 0, 0.13),

            new ParallelCommandGroup(
                    commandFactory.collapseSlider(),

                commandFactory.pivotToGroundInTakeBegin(),
                commandFactory.elbowToIntakePosition()
            ),

            commandFactory.intakeFromGround(),

            new ParallelCommandGroup(
                    commandFactory.elbowToSpecimenPosition(),
                    commandFactory.pivotToDelivery()
                    ),

            commandFactory.extendSlider(),

            commandFactory.driveToTarget(180, 520, -45, 0.1, 150),

            // Sample #2
            commandFactory.extendSlider(commandFactory.outtake()),
            commandFactory.driveToTarget(400, 610, 0, 0.1),


            new ParallelCommandGroup(
                    commandFactory.collapseSlider(),
                    commandFactory.pivotToGroundInTakeBegin(),
                    commandFactory.elbowToIntakePosition()
            ),

            commandFactory.intakeFromGround(),

            new ParallelCommandGroup(
                    commandFactory.elbowToSpecimenPosition(),
                    commandFactory.pivotToDelivery()
            ),

            commandFactory.extendSlider(),
            commandFactory.driveToTarget(180, 520, -45, 0.1, 150),

            // Sample #3
            commandFactory.extendSlider(commandFactory.outtake()),

            commandFactory.driveToTarget(440, 540, 35, 0.1),

            commandFactory.collapseSlider(),

            new ParallelCommandGroup(
                    commandFactory.pivotToGroundInTakeBegin(),
                    commandFactory.elbowToIntakePosition()
            ),

            commandFactory.intakeFromGround(),

            new ParallelCommandGroup(
                    commandFactory.elbowToSpecimenPosition(),
                    commandFactory.pivotToDelivery()

            ),

            commandFactory.extendSlider(),

            commandFactory.driveToTarget(180, 520, -45, 0.1, 150),

            // Sample #4
            commandFactory.extendSlider(commandFactory.outtake()),

            new ParallelCommandGroup(
                    commandFactory.pivotToStart(),
                    commandFactory.collapseSlider()
            )
        );
    }
}

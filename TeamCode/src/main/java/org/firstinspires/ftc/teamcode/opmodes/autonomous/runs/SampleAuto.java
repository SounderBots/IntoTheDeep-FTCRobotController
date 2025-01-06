package org.firstinspires.ftc.teamcode.opmodes.autonomous.runs;

import android.util.Log;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.LogCatCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.PrintCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.autonomous.base.CommandAutoOpMode;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.command.CommandFactory;

@Autonomous
public class SampleAuto extends CommandAutoOpMode {

    boolean hold1End = false;
    boolean hold2End = false;
    boolean hold3End = false;
    @Override
    protected Command createCommand() {
        return new SequentialCommandGroup(


                //region preloaded sample
                // drive to bucket, pivot the arm and extend slider
                new ParallelCommandGroup(
                    commandFactory.driveToTarget(300, 400, -45, .13, .7, 100),
                    commandFactory.pivotToDelivery(),
                    commandFactory.elbowToSpecimenPosition(),
                    commandFactory.extendSlider()
                ),

                // ?
                commandFactory.driveToTarget(10, 420, -45, 0.13, .8, 50),

                // deliver preloaded sample
                commandFactory.outtake().andThen(new InstantCommand(() -> hold1End = true)),
                //endregion

                //region sample #1
                commandFactory.driveToTarget(440, 260, 0, 0.13, .7, 20),

                new ParallelCommandGroup(
                    commandFactory.collapseSlider(),
                    commandFactory.pivotToGroundInTakeBegin(),
                    commandFactory.elbowToIntakePosition()
                ),

                commandFactory.intakeFromGround2(1000),
                commandFactory.inCaseSampleIntakeFailed("Sample 1", new SequentialCommandGroup(
                    commandFactory.pivotToIntakeRetry(),
                        commandFactory.pivotToGroundInTakeBegin(),
                        commandFactory.intakeFromGround2(2000)
                )),

                new ParallelCommandGroup(
                    commandFactory.elbowToSpecimenPosition(),
                    commandFactory.pivotToDelivery()
                ),

                commandFactory.extendSlider(),

                commandFactory.driveToTarget(20, 440, -45, 0.13, .5, 100),

                // Sample #1
                commandFactory.outtake().andThen(new InstantCommand(() -> hold2End = true),
                //endregion for sample #1

                //region sample #2
                commandFactory.driveToTarget(430, 520, 0, 0.13, .8, 30)),

                new ParallelCommandGroup(
                    commandFactory.collapseSlider(),
                    commandFactory.pivotToGroundInTakeBegin(),
                    commandFactory.elbowToIntakePosition()
                ),

                commandFactory.intakeFromGround2(1000),

                commandFactory.inCaseSampleIntakeFailed("Sample 2", new SequentialCommandGroup(
                        commandFactory.pivotToIntakeRetry(),
                        commandFactory.pivotToGroundInTakeBegin(),
                        commandFactory.intakeFromGround2(2000)
                )),

                new ParallelCommandGroup(
                    commandFactory.elbowToSpecimenPosition(),
                    commandFactory.pivotToDelivery()
                ),

                commandFactory.driveToTarget(300, 400, 0, 0.13, .8, 100),
                commandFactory.extendSlider(),
                commandFactory.driveToTarget(80, 440, -45, 0.13, .8, 50),


                // Sample #2
                commandFactory.outtake().andThen(new InstantCommand(() -> hold3End = true)),
                //endregion sample #2

                //region sample #3
                commandFactory.driveToTarget(250, 540, 17, 0.13, .8, 30),
                commandFactory.collapseSlider(),
                new ParallelCommandGroup(
                    commandFactory.pivotToGroundInTakeBegin(),
                    commandFactory.extendSliderToIntakeSample3()
                ),
                commandFactory.elbowToIntakePosition(),
                commandFactory.sleep(300),
                commandFactory.intakeFromGroundForSample3(4000),

                commandFactory.inCaseSampleIntakeFailed("Sample 3", new SequentialCommandGroup(
                        commandFactory.pivotToIntakeRetry(),
                        commandFactory.pivotToGroundInTakeBegin(),
                        commandFactory.intakeFromGroundForSample3(6000)
                )),

                new ParallelCommandGroup(
                    commandFactory.elbowToSpecimenPosition(),
                    commandFactory.pivotToDelivery()

                ),

                commandFactory.driveToTarget(300, 400, 0, 0.13, .8, 100),
                commandFactory.extendSlider(),
                commandFactory.driveToTarget(80, 440, -45, 0.13, .8, 50),


                // Sample #3
                commandFactory.outtake().andThen(new InstantCommand(() -> hold3End = true)),
                commandFactory.driveToTarget(300, 400, 0, 0.13, .8, 100),
                //endregion sample #3

                new ParallelCommandGroup(
                    commandFactory.pivotToStart(),
                    commandFactory.collapseSlider(),
                    commandFactory.driveToTarget(2600, -290, -120, .13, 1, 100),
                    commandFactory.elbowToStartPosition()
                )

        );
    }
}

package org.firstinspires.ftc.teamcode.opmodes.autonomous.command;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.base.CommandScheduler;
import org.firstinspires.ftc.teamcode.subsystems.delivery.DeliveryPivot;
import org.firstinspires.ftc.teamcode.subsystems.delivery.DeliverySlider;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.AutoMecanumDriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.intake.RollingIntake;
import org.firstinspires.ftc.teamcode.subsystems.vision.LimeLight;

@SuppressWarnings("unused")
public class CommandFactory {

    private static final PIDController xPIDController = new PIDController(0.015, 0, 0.05);
    private static final PIDController yPIDController = new PIDController(0.015, 0, 0.05);
    private static final ProfiledPIDController rotPIDController = new ProfiledPIDController(0.015, 0, .05, new TrapezoidProfile.Constraints(
            Math.PI * 2,
            Math.PI
    ));

    private static final double MAX_VELOCITY = 400;

    private static final TrajectoryConfig trajectoryConfig = new TrajectoryConfig(MAX_VELOCITY, 1);



    private final AutoMecanumDriveTrain driveTrain;
    private final RollingIntake intake;
    private final LimeLight vision;
    private final Telemetry telemetry;
    private final DeliveryPivot pivot;
    private final DeliverySlider slider;

    private CommandScheduler commandScheduler;

    public CommandFactory(Telemetry telemetry, AutoMecanumDriveTrain driveTrain, RollingIntake intake, LimeLight vision, DeliveryPivot pivot, DeliverySlider slider) {
        this.driveTrain = driveTrain;
        this.intake = intake;
        this.vision = vision;
        this.telemetry = telemetry;
        this.pivot = pivot;
        this.slider = slider;
    }

    public void setCommandScheduler(CommandScheduler commandScheduler) {
        this.commandScheduler = commandScheduler;
    }

    public AlignToSample alignToSample() {
        return new AlignToSample(driveTrain, vision, telemetry);
    }

    public TelemetryCommand WriteTelemetry() {
        return  new TelemetryCommand(driveTrain, telemetry);
    }

    public MotorTurnCommand MotorTest() {
        return new MotorTurnCommand(driveTrain, telemetry);
    }

    public DriveToTargetCommand driveToTarget(double targetX, double targetY, double targetHeading, double minPower) {
        return new DriveToTargetCommand(driveTrain, telemetry, targetX, targetY, targetHeading, minPower, 1.0, 20);
    }

    public DriveToTargetCommand driveToTarget(double targetX, double targetY, double targetHeading, double minPower, double distanceTolerance) {
        return new DriveToTargetCommand(driveTrain, telemetry, targetX, targetY, targetHeading, minPower, 1.0, distanceTolerance);
    }

    public DriveToTargetCommand driveToTarget(double targetX, double targetY, double targetHeading, double minPower, double maxPower, double distanceTolerance) {
        return new DriveToTargetCommand(driveTrain, telemetry, targetX, targetY, targetHeading, minPower, maxPower, distanceTolerance);
    }

    public DriveToTargetCommand driveToTarget(double targetX, double targetY, double targetHeading) {
        return driveToTarget(targetX, targetY, targetHeading, 0.1);
    }

    public DriveToPositionCommand driveToPosition(double targetX, double targetY, double targetheading) {
        return new DriveToPositionCommand(driveTrain, telemetry).setTargetPosition(targetX, targetY, targetheading);
    }

    public DriveToTargetCommandAlterate driveToTargetAlternate(double targetX, double targetY, double targetHeading) {
        return new DriveToTargetCommandAlterate(driveTrain, telemetry, targetX, targetY, targetHeading);
    }

    public TurnAngleAbsoluteCommand turnAngleAbsolute(double angleInDegrees) {
        return new TurnAngleAbsoluteCommand(driveTrain, telemetry, angleInDegrees);
    }

    public MoveSliderCommand extendSlider() {
        return new MoveSliderCommand(slider, telemetry, DeliverySlider.BasketDeliveryPosition);
    }

    public MoveSliderCommand extendSlider(SounderBotCommandBase onTargetReachedCommand) {
        MoveSliderCommand moveSliderCommand = new MoveSliderCommand(slider, telemetry, DeliverySlider.BasketDeliveryPosition);
        moveSliderCommand.registerOnTargetReachedHandler(callback ->
                commandScheduler.schedule(onTargetReachedCommand.andThen(new InstantCommand(callback))));
        return moveSliderCommand;
    }

    public MoveSliderCommand extendSliderToSpeciment() {
        return new MoveSliderCommand(slider, telemetry, DeliverySlider.StartPosition);
    }

    public MoveSliderCommand extendSliderToDeliverSpeciman() {
        return new MoveSliderCommand(slider, telemetry, DeliverySlider.StartPosition-600);
    }

    public MoveSliderCommand collapseSlider() {
        return new MoveSliderCommand(slider, telemetry, DeliverySlider.CollapsedPosition, true);
    }

    public TurnAngleRelativeCommand turnAngleRelative(double angleInDegrees) {
        return new TurnAngleRelativeCommand(driveTrain, telemetry, angleInDegrees);
    }

    public SleeperCommand waitFor(long timeInMs) {
        return new SleeperCommand(timeInMs);
    }

    public MovePivotCommand pivotToInTake() {
        return new MovePivotCommand(pivot, telemetry, DeliveryPivot.IntakePositionFromStart);
    }

    public MovePivotCommand pivotToSpecimenInTake() {
        return new MovePivotCommand(pivot, telemetry, DeliveryPivot.IntakePositionFromStart - 250);
    }

    public MovePivotCommand pivotToGroundInTakeBegin() {
        return new MovePivotCommand(pivot, telemetry, DeliveryPivot.IntakePositionFromStart + 200);
    }

    public MovePivotCommand pivotToStart() {
        return new MovePivotCommand(pivot, telemetry, DeliveryPivot.StartPositionFromStart);
    }


    public MovePivotCommand pivotToDelivery() {
        return new MovePivotCommand(pivot, telemetry, DeliveryPivot.DeliveryPositionFromStart);
    }

    public MovePivotCommand pivotToSpecimenDelivery() {
        return new MovePivotCommand(pivot, telemetry, DeliveryPivot.StartPositionFromStart - 55);

    }

    public SingleRunCommand stopDriveTrain() {
        return new SingleRunCommand(driveTrain::stop, driveTrain);
    }

    public SingleRunCommand elbowToIntakePosition() {
        return new SingleRunCommand(intake::SetElbowInIntakePosition);
    }

    public SingleRunCommand elbowToDeliveryPosition() {
        return new SingleRunCommand(intake::SetElbowInSampleDeliveryPosition);
    }

    public SingleRunCommand elbowToSpecimenPosition() {
        return new SingleRunCommand(intake::SetElbowInSampleDeliveryPosition);
    }

    public SmartIntakeCommand intake() {
        return new SmartIntakeCommand(intake);
    }

    public SmartOuttakeCommand outtake() {
        return new SmartOuttakeCommand(intake);
    }

    public IntakeFromGround intakeFromGround() {
        return new IntakeFromGround(intake, pivot);
    }

    public RoboticCentricDriveCommand driveRobotCentric(double strafeSpeed, double forwardSpeed, double rotSpeed, long timeToDriveMs) {
        return new RoboticCentricDriveCommand(driveTrain, strafeSpeed, forwardSpeed, rotSpeed, timeToDriveMs);
    }
}

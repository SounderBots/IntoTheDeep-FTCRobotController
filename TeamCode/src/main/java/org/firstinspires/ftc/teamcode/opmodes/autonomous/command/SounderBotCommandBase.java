package org.firstinspires.ftc.teamcode.opmodes.autonomous.command;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.Robot;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Consumer;

public abstract class SounderBotCommandBase extends CommandBase {
    private static final String LOG_TAG = SounderBotCommandBase.class.getSimpleName();
    boolean finished = false;
    long TIME_OUT_MS = 1500; // 4 seconds

    long startTime = -1;

    boolean waitingForEndCallback = false;

    private Consumer<Runnable> onTargetReachedHandler;

    private final String CLASS_LOG_TAG = getClass().getSimpleName();
    @Override
    public final boolean isFinished() {
        return finished;
    }

    @Override
    public final void execute() {
        if (startTime < 0) {
            startTime = System.currentTimeMillis();
            doExecute();
        } else {
            if (isTimeout()) {
                if (!finished) {
                    onTimeout();
                }
                return;
            } else {
                doExecute();
            }
        }

        checkTarget();
    }

    private void checkTarget() {
        if (!finished && !waitingForEndCallback) {
            if (isTargetReached()) {
                RobotLog.ii(CLASS_LOG_TAG, "Target reached");
                Runnable endCallback = () -> {
                    RobotLog.ii(CLASS_LOG_TAG, "End callback triggered");
                    waitingForEndCallback = false;
                    end(false);
                    finished = true;
                };

                onTargetReached(endCallback);
                RobotLog.ii(CLASS_LOG_TAG, "onTargetReached finished");
                if (!finished) {
                    RobotLog.ii(CLASS_LOG_TAG, "Start wait for end callback");
                    waitingForEndCallback = true;
                    onWaitForEndCallback();
                }
            } else {
                RobotLog.ii(CLASS_LOG_TAG, "target not reached, keep execute");
            }
        } else if (waitingForEndCallback) {
            RobotLog.ii(CLASS_LOG_TAG, "Still waiting for the callback to finish");
            onWaitForEndCallback();
        } else {
            RobotLog.ii(CLASS_LOG_TAG, "already finished, do nothing");
        }
    }

    private boolean isTimeout() {
        long timeUsed = System.currentTimeMillis() - startTime;
        return timeUsed > TIME_OUT_MS;
    }

    protected void onWaitForEndCallback() {
    }

    protected abstract void doExecute();

    protected void onTimeout() {
        Log.w(LOG_TAG, String.format("Command (name=%s) reached timeout (timeout=%d seconds)", getClass().getSimpleName(), TIME_OUT_MS / 1000));
        finished = true;
        end(true);
    }

    protected abstract boolean isTargetReached();

    protected void sleep(long timeInMs) {
        try {
            long sleepStartTime = System.currentTimeMillis();
            while (!finished && !isTimeout()) {
                Thread.sleep(20);
                if (System.currentTimeMillis() - sleepStartTime > timeInMs) {
                    break;
                }
            }
        } catch (InterruptedException e) {
            // ok to be interrupted
            Log.i(LOG_TAG, "sleep interrupted");
        }
    }

    public void registerOnTargetReachedHandler(Consumer<Runnable> handler) {
        this.onTargetReachedHandler = handler;
    }

    public void onTargetReached(Runnable endCallback) {
        if (onTargetReachedHandler != null) {
            onTargetReachedHandler.accept(endCallback);
        } else {
            endCallback.run();
        }
    }
}

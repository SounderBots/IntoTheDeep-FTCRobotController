package org.firstinspires.ftc.teamcode.opmodes.autonomous.command;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;

public abstract class SounderBotCommandBase extends CommandBase {
    private static final String LOG_TAG = SounderBotCommandBase.class.getSimpleName();
    boolean finished = false;
    long TIME_OUT_MS = 1500; // 4 seconds

    long startTime = -1;

    private boolean onTargetReachedCalled = false;

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
            } else {
                if (!isTargetReached()) {
                    doExecute();
                } else {
                    if (!onTargetReachedCalled) {
                        onTargetReachedCalled = true;
                        onTargetReached();
                    }
                    if (shouldFinishWhenTargetReached()) {
                        finished = true;
                        end(false);
                    } else {
                        afterTargetReached();
                    }
                }
            }
        }
    }

    protected boolean shouldFinishWhenTargetReached() {
        return true;
    }

    protected void onTargetReached() {

    }

    protected void afterTargetReached() {

    }

    private boolean isTimeout() {
        long timeUsed = System.currentTimeMillis() - startTime;
        return timeUsed > TIME_OUT_MS;
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
}

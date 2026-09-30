/*
 * Copyright (c) 2026 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package teamcode.autocommands;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import static com.pedropathing.api.Paths.line;

import teamcode.subsystems.Pedro;
import trclib.robotcore.TrcEvent;
import trclib.robotcore.TrcRobot;
import trclib.robotcore.TrcStateMachine;

/**
 * This class implements a minimal autonomous command that drives one straight-line Pedro Pathing path from
 * (0, 0, 0) to a menu-selected target pose. It exists as a flashable smoke test for the Pedro integration
 * (build, deploy, drive one path) and as a worked example for writing real season autonomous: build a
 * {@link Path} per state, call {@link Pedro#followPath(Path, TrcEvent)}, then
 * {@link TrcStateMachine#waitForEvents(Object, TrcEvent...)} for the next state -- no PathChain equivalent is
 * needed, each segment is just another state.
 */
public class CmdPedroPathDrive implements TrcRobot.RobotCommand
{
    private static final String moduleName = CmdPedroPathDrive.class.getSimpleName();

    private enum State
    {
        DRIVE_PATH,
        DONE
    }   //enum State

    private final Pedro pedro;
    private final TrcEvent event;
    private final TrcStateMachine<State> sm;
    private double xTarget, yTarget, headingTarget;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param pedro specifies the Pedro Pathing subsystem to drive.
     */
    public CmdPedroPathDrive(Pedro pedro)
    {
        this.pedro = pedro;
        event = new TrcEvent(moduleName);
        sm = new TrcStateMachine<>(moduleName);
    }   //CmdPedroPathDrive

    /**
     * This method sets the target pose (relative to the robot's starting pose) and starts the command.
     *
     * @param xTarget specifies the target x position in inches.
     * @param yTarget specifies the target y position in inches.
     * @param headingTarget specifies the target heading in degrees.
     */
    public void startPath(double xTarget, double yTarget, double headingTarget)
    {
        this.xTarget = xTarget;
        this.yTarget = yTarget;
        this.headingTarget = headingTarget;
        sm.start(State.DRIVE_PATH);
    }   //startPath

    //
    // Implements the TrcRobot.RobotCommand interface.
    //

    /**
     * This method starts the RobotCommand. startPath must be called first to provide the target pose.
     */
    @Override
    public void start()
    {
        // No-op: startPath() both provides the target pose and starts the state machine.
    }   //start

    /**
     * This method cancels the command if it is active.
     */
    @Override
    public void cancel()
    {
        pedro.cancel();
        sm.stop();
    }   //cancel

    /**
     * This method checks if the current RobotCommand is running.
     *
     * @return true if the command is running, false otherwise.
     */
    @Override
    public boolean isActive()
    {
        return sm.isEnabled();
    }   //isActive

    /**
     * This method must be called periodically by the caller to drive the command sequence forward.
     *
     * @param elapsedTime specifies the elapsed time in seconds since the start of the robot mode.
     * @return true if the command sequence is completed, false otherwise.
     */
    @Override
    public boolean cmdPeriodic(double elapsedTime)
    {
        State state = sm.checkReadyAndGetState();

        if (state != null)
        {
            switch (state)
            {
                case DRIVE_PATH:
                    Path path = line(Pose.zero(), new Pose(xTarget, yTarget)).constant(Math.toRadians(headingTarget));
                    pedro.followPath(path, event);
                    sm.waitForEvents(State.DONE, event);
                    break;

                case DONE:
                default:
                    cancel();
                    break;
            }
        }

        return !sm.isEnabled();
    }   //cmdPeriodic

}   //class CmdPedroPathDrive

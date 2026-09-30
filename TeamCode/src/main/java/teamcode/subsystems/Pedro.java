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

package teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.hardware.HardwareMap;

import ftclib.driverio.FtcDashboard;
import teamcode.Robot;
import teamcode.RobotParams;
import teamcode.pedroPathing.Constants;
import trclib.pathdrive.TrcPose2D;
import trclib.robotcore.TrcEvent;
import trclib.robotcore.TrcRobot;
import trclib.robotcore.TrcTaskMgr;
import trclib.subsystem.TrcSubsystem;

/**
 * This class is the TRC adapter for Pedro Pathing. It wraps a Pedro {@link Follower} (which does all pose
 * tracking and path following) so it can be driven from TRC state machines the same way any other trclib
 * drive engine is: {@link #followPath(Path, TrcEvent)} signals a {@link TrcEvent} when the path completes,
 * matching the pattern used by {@link trclib.command.CmdPurePursuitDrive}.
 *
 * Unlike {@link trclib.pathdrive.TrcPurePursuitDrive}, which only registers its drive task while a path is
 * active, this subsystem keeps its task registered for the whole OpMode: {@link Follower#update()} must run
 * every loop regardless of whether a path is active, since it is also what applies manual/teleop drive
 * commands issued through {@link #subsystemControl(boolean, double...)}.
 */
public class Pedro extends TrcSubsystem<Pedro.Action>
{
    public enum Action
    {
        ToggleFieldOriented
    }   //enum Action

    public static final String SUBSYSTEM_NAME = "Pedro";
    private static final boolean NEED_ZERO_CAL = false;

    private final Robot robot;
    private final FtcDashboard dashboard;
    private final Follower follower;
    private final TrcTaskMgr.TaskObject pedroTaskObj;
    private TrcEvent pathCompletionEvent;
    private boolean isFollowingPath = false;
    private boolean fieldOriented = false;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param robot specifies the robot object to access other subsystems if necessary.
     * @param hardwareMap specifies the hardware map to look up the drivetrain motors and Pinpoint device.
     */
    public Pedro(Robot robot, HardwareMap hardwareMap)
    {
        super(SUBSYSTEM_NAME, NEED_ZERO_CAL);

        this.robot = robot;
        dashboard = FtcDashboard.getInstance();
        follower = Constants.create(hardwareMap);
        follower.setPose(Pose.zero());
        pedroTaskObj = TrcTaskMgr.createTask(instanceName + ".pedroTask", this::pedroTask);
        pedroTaskObj.registerTask(TrcTaskMgr.TaskType.OUTPUT_TASK);
    }   //Pedro

    /**
     * This method returns the underlying Pedro Follower for direct access (e.g. holding a pose, adjusting
     * max power).
     *
     * @return Pedro Follower.
     */
    public Follower getFollower()
    {
        return follower;
    }   //getFollower

    /**
     * This method starts following the given path and arranges for the given event to be signaled when the
     * path completes, the same way {@link trclib.command.CmdPurePursuitDrive} signals its completion event.
     *
     * @param path specifies the Pedro path to follow.
     * @param event specifies the event to signal when the path is done, can be null if not needed.
     */
    public void followPath(Path path, TrcEvent event)
    {
        if (event != null)
        {
            event.clear();
        }
        pathCompletionEvent = event;
        isFollowingPath = true;
        follower.follow(path);
    }   //followPath

    /**
     * This method returns the current robot pose as tracked by Pedro Pathing.
     *
     * @return current robot pose.
     */
    public TrcPose2D getCurrentPose()
    {
        return toTrcPose2D(follower.pose());
    }   //getCurrentPose

    /**
     * This method sets the robot's starting pose for Pedro Pathing, e.g. at the start of autonomous.
     *
     * @param pose specifies the starting pose.
     */
    public void setStartPose(TrcPose2D pose)
    {
        follower.setPose(toPedroPose(pose));
    }   //setStartPose

    /**
     * This method converts a TrcPose2D (inches, degrees) to a Pedro Pose (inches, radians).
     *
     * @param pose specifies the TrcPose2D to convert.
     * @return equivalent Pedro Pose.
     */
    private static Pose toPedroPose(TrcPose2D pose)
    {
        return new Pose(pose.x, pose.y, Math.toRadians(pose.angle));
    }   //toPedroPose

    /**
     * This method converts a Pedro Pose (inches, radians) to a TrcPose2D (inches, degrees).
     *
     * @param pose specifies the Pedro Pose to convert.
     * @return equivalent TrcPose2D.
     */
    private static TrcPose2D toTrcPose2D(Pose pose)
    {
        return new TrcPose2D(pose.x(), pose.y(), Math.toDegrees(pose.heading()));
    }   //toTrcPose2D

    /**
     * This method is called periodically to update the Pedro Follower. It drives path following forward and
     * applies whatever manual drive command was last set by subsystemControl, and signals the path completion
     * event once the active path reaches its parametric end.
     */
    private void pedroTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode, boolean slowPeriodicLoop)
    {
        follower.update();
        if (isFollowingPath && follower.atParametricEnd())
        {
            isFollowingPath = false;
            if (pathCompletionEvent != null)
            {
                pathCompletionEvent.signal();
                pathCompletionEvent = null;
            }
        }
    }   //pedroTask

    //
    // Implements TrcSubsystem abstract methods.
    //

    /**
     * This method cancels any pending operations.
     */
    @Override
    public void cancel()
    {
        follower.stop();
        isFollowingPath = false;
        pathCompletionEvent = null;
    }   //cancel

    /**
     * This method starts zero calibrate of the subsystem.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the motor.
     * @param completionEvent specifies the event to signal when the zero calibration is done,
     *        can be null if not provided.
     */
    @Override
    public void zeroCalibrate(String owner, TrcEvent completionEvent)
    {
        // Pedro does not need zero calibration.
        if (completionEvent != null)
        {
            completionEvent.signal();
        }
    }   //zeroCalibrate

    /**
     * This method resets the subsystem state. Typically, this is used to retract the subsystem for turtle mode.
     */
    @Override
    public void resetState()
    {
        // Pedro does not support resetState.
    }   //resetState

    /**
     * This method is called when gamepad analog control is operated on the subsystem.
     *
     * @param altFunc specifies true if the gamepad AltFunc button is pressed, false otherwise.
     * @param inputs specifies an array of analog values: [0]=strafe(x), [1]=drive(y), [2]=turn.
     */
    @Override
    public void subsystemControl(boolean altFunc, double... inputs)
    {
        double strafe = inputs[0];
        double drive = inputs[1];
        double turn = inputs[2];

        if (fieldOriented)
        {
            follower.manual(ManualDrive.fieldCentric(drive, strafe, turn, follower.pose().heading()));
        }
        else
        {
            follower.manual(drive, strafe, turn);
        }

        if (dashboard.isDashboardUpdateEnabled() && RobotParams.Preferences.showDriveBaseStatus)
        {
            dashboard.displayPrintf(
                14, "Pedro: Power=(x=%.2f,y=%.2f,rot=%.2f),Field=%b", strafe, drive, turn, fieldOriented);
        }
    }   //subsystemControl

    /**
     * This method is called to perform the subsystem action.
     *
     * @param action specifies the subsystem action to perform.
     * @param context specifies the context object for the action (not used).
     */
    @Override
    public void subsystemAction(Action action, Object context)
    {
        switch (action)
        {
            case ToggleFieldOriented:
                fieldOriented = !fieldOriented;
                robot.globalTracer.traceInfo(instanceName, ">>>>> FieldOriented=" + fieldOriented);
                break;

            default:
                break;
        }
    }   //subsystemAction

    /**
     * This method is called to perform the subsystem tune action.
     *
     * @param action specifies the subsystem tune action to perform.
     * @param tuneSubsystemName specifies the subsystem object to tune.
     */
    @Override
    public void tuneSubsystem(TuneAction action, String tuneSubsystemName)
    {
        // Pedro is tuned via the AutoTune wizard (see teamcode.pedroPathing.Tuning), not this mechanism.
    }   //tuneSubsystem

    /**
     * This method publishes the NetworkTable entries for the subsystem to the Dashboard.
     */
    @Override
    public void publishToDashboard()
    {
        // Not applicable for FTC.
    }   //publishToDashboard

    /**
     * This method update the dashboard with the subsystem status.
     *
     * @param lineNum specifies the starting line number to print the subsystem status.
     * @param slowLoop specifies true if this is a slow loop, false otherwise.
     * @return updated line number for the next subsystem to print.
     */
    @Override
    public int updateStatus(int lineNum, boolean slowLoop)
    {
        if (slowLoop && RobotParams.Preferences.showDriveBaseStatus)
        {
            dashboard.displayPrintf(lineNum++, "Pedro: pose=%s, following=%b", follower.pose(), isFollowingPath);
        }
        return lineNum;
    }   //updateStatus

    /**
     * This method is called to update subsystem parameter to the Dashboard.
     *
     * @param subsystemName specifies the name of the subsystem to be updated.
     */
    @Override
    public void updateParamsToDashboard(String subsystemName)
    {
        // Pedro doesn't support tuning through this mechanism.
    }   //updateParamsToDashboard

    /**
     * This method is called to update subsystem parameters from the Dashboard.
     *
     * @param subsystemName specifies the name of the subsystem to be updated.
     */
    @Override
    public void updateParamsFromDashboard(String subsystemName)
    {
    }   //updateParamsFromDashboard

}   //class Pedro

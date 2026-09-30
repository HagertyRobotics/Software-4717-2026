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

package teamcode.pedroPathing;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.follower.Follower;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * This class contains the Pedro Pathing configuration for this robot (localizer, drivetrain, and follow
 * algorithm) and the single factory method that builds a fully wired {@link Follower} from it.
 *
 * The hardware names below match the Pinpoint-based Mecanum drive already configured in
 * {@link teamcode.subsystems.DriveBase.MecanumRobotInfo} so both drive engines point at the same physical
 * hardware config. Everything else here (pod offsets/directions, Foresight gains) is a placeholder -- run the
 * Pedro Pathing AutoTune wizard (see {@link Tuning}) on the robot and paste its generated code over the
 * corresponding block below before using this in competition.
 */
public class Constants
{
    // Run the Mecanum AutoTune procedure and replace the directions below with its output.
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c ->
    {
        c.frontLeftName.set("flDriveMotor");
        c.frontRightName.set("frDriveMotor");
        c.backLeftName.set("blDriveMotor");
        c.backRightName.set("brDriveMotor");
        c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
    });

    // Run the Pinpoint AutoTune procedure and replace this block with its output (pod type, offsets,
    // directions). "pinpointOdo" matches the hardware config name used by DriveBase.MecanumRobotInfo.
    public static PinpointConfig localizerConfig = new PinpointConfig(c ->
    {
        c.name.set("pinpointOdo");
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    // Library defaults until the Foresight AutoTune procedure has been run; replace with its output.
    public static ForesightConfig foresightConfig = new ForesightConfig(c -> {});

    /**
     * This method creates a fully configured Pedro Pathing Follower for this robot.
     *
     * @param hardwareMap specifies the hardware map to look up the drivetrain motors and Pinpoint device.
     * @return configured Follower.
     */
    public static Follower create(HardwareMap hardwareMap)
    {
        return new Follower(
            new PinpointLocalizer(hardwareMap, localizerConfig),
            new Mecanum(hardwareMap, drivetrainConfig),
            new Foresight(foresightConfig));
    }   //create

}   //class Constants

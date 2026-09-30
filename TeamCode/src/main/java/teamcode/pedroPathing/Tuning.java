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
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;
import teamcode.pedroPathing.procedures.ForesightTuner;
import teamcode.pedroPathing.procedures.MecanumTuner;
import teamcode.pedroPathing.procedures.PinpointTuner;
import teamcode.pedroPathing.procedures.Tests;

/**
 * This class registers the Pedro Pathing AutoTune procedures relevant to this robot (Pinpoint localizer,
 * Mecanum drivetrain, Foresight follow algorithm). While the robot is connected to the Driver Station network
 * and TeamCode is installed, open the AutoTune webpage to run these procedures interactively; each one prints
 * generated Java code to paste into {@link Constants}.
 */
public class Tuning
{
    @Tuner
    public static Procedure mecanumTuner()
    {
        return new MecanumTuner();
    }   //mecanumTuner

    @Tuner
    public static Procedure pinpointTuner()
    {
        return new PinpointTuner();
    }   //pinpointTuner

    @Tuner
    public static Procedure foresightTuner()
    {
        return new ForesightTuner(
            h -> new PinpointLocalizer(h, Constants.localizerConfig),
            h -> new Mecanum(h, Constants.drivetrainConfig));
    }   //foresightTuner

    @Tuner
    public static Procedure tests()
    {
        return new Tests(
            h -> new Mecanum(h, Constants.drivetrainConfig),
            h -> new PinpointLocalizer(h, Constants.localizerConfig),
            () -> new Foresight(Constants.foresightConfig));
    }   //tests

}   //class Tuning

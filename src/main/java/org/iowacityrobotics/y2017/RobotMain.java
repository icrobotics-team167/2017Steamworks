package org.iowacityrobotics.y2017;

import org.iowacityrobotics.roboed.api.IRobot;
import org.iowacityrobotics.roboed.api.IRobotProgram;
import org.iowacityrobotics.roboed.api.RobotMode;
import org.iowacityrobotics.roboed.api.operations.IOpMode;
import org.iowacityrobotics.roboed.api.subsystem.ISubsystem;
import org.iowacityrobotics.roboed.impl.data.DataMappers;
import org.iowacityrobotics.roboed.impl.subsystem.impl.MecanumSubsystem;
import org.iowacityrobotics.roboed.impl.subsystem.impl.SingleJoySubsystem;
import org.iowacityrobotics.roboed.util.math.Vector3;

public class RobotMain implements IRobotProgram {

    private ISubsystem<Void, Vector3> joy;
    private ISubsystem<MecanumSubsystem.ControlDataFrame, Void> driveTrain;

    @Override
    public void init(IRobot robot) {
        IOpMode stdMode = robot.getOpManager().getOpMode("standard");
        joy = robot.getSystemRegistry().getProvider(SingleJoySubsystem.TYPE).getSubsystem(1);
        driveTrain = robot.getSystemRegistry().getProvider(MecanumSubsystem.TYPE).getSubsystem(0, 1, 2, 3);
        stdMode.onInit(() -> driveTrain.bind(joy.output().map(DataMappers.singleJoyMecanum())));
        stdMode.whileCondition(() -> true);
        robot.getOpManager().setDefaultOpMode(RobotMode.TELEOP, "standard");
    }

}

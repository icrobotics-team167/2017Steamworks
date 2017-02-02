package org.iowacityrobotics.y2017;

import com.ctre.CANTalon;
import org.iowacityrobotics.roboed.api.IRobot;
import org.iowacityrobotics.roboed.api.IRobotProgram;
import org.iowacityrobotics.roboed.api.RobotMode;
import org.iowacityrobotics.roboed.api.operations.IOpMode;
import org.iowacityrobotics.roboed.api.subsystem.ISubsystem;
import org.iowacityrobotics.roboed.impl.data.DataMappers;
import org.iowacityrobotics.roboed.impl.subsystem.impl.DualJoySubsystem;
import org.iowacityrobotics.roboed.impl.subsystem.impl.MecanumSubsystem;
import org.iowacityrobotics.roboed.util.collection.Pair;
import org.iowacityrobotics.roboed.util.math.Vector2;
import org.iowacityrobotics.roboed.util.math.Vector3;
import org.iowacityrobotics.roboed.util.robot.QuadraSpeedController;

public class RobotMain implements IRobotProgram {

    private ISubsystem<Void, Pair<Vector2, Vector2>> joy;
    private ISubsystem<MecanumSubsystem.ControlDataFrame, Void> driveTrain;

    @Override
    public void init(IRobot robot) {
        System.out.println("INIT");
        // Register custom subsystem type
        //robot.getSystemRegistry().registerProvider(ShooterSubsystem.TYPE, new ShooterSubsystem.Provider());

        // Initialize subsystems
        joy = robot.getSystemRegistry().getProvider(DualJoySubsystem.TYPE).getSubsystem(1);
        QuadraSpeedController talons = new QuadraSpeedController(
                new CANTalon(3),
                new CANTalon(4),
                new CANTalon(2),
                new CANTalon(5)
        );
        talons.getC().setInverted(true);
        talons.getD().setInverted(true);
        driveTrain = robot.getSystemRegistry().getProvider(MecanumSubsystem.TYPE_CUSTOM).getSubsystem(talons);

        // Set up standard teleop opmode
        IOpMode stdMode = robot.getOpManager().getOpMode("standard");
        final double threshold = 0.1D;
        stdMode.onInit(() -> driveTrain.bind(joy.output()
                .map(v -> {
                    if (Math.abs(v.getA().x()) <= threshold)
                        v.getA().x(0);
                    if (Math.abs(v.getA().y()) <= threshold)
                        v.getA().y(0);
                    return v;
                })
                .map(v -> Pair.of(v.getA().multiply(0.75D), v.getB().multiply(0.75D)))
                .map(DataMappers.dualJoyMecanum())));
        stdMode.whileCondition(() -> true);
        robot.getOpManager().setDefaultOpMode(RobotMode.TELEOP, "standard");
    }

}

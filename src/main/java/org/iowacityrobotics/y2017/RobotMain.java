package org.iowacityrobotics.y2017;

import com.ctre.CANTalon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.iowacityrobotics.roboed.api.IRobot;
import org.iowacityrobotics.roboed.api.IRobotProgram;
import org.iowacityrobotics.roboed.api.RobotMode;
import org.iowacityrobotics.roboed.api.data.Data;
import org.iowacityrobotics.roboed.api.operations.IOpMode;
import org.iowacityrobotics.roboed.api.subsystem.ISubsystem;
import org.iowacityrobotics.roboed.impl.data.DataMappers;
import org.iowacityrobotics.roboed.impl.data.Interpolators;
import org.iowacityrobotics.roboed.impl.subsystem.impl.DualTreadSubsystem;
import org.iowacityrobotics.roboed.impl.subsystem.impl.MecanumSubsystem;
import org.iowacityrobotics.roboed.impl.subsystem.impl.SingleJoySubsystem;
import org.iowacityrobotics.roboed.util.math.Vector3;
import org.iowacityrobotics.roboed.util.robot.QuadraSpeedController;

public class RobotMain implements IRobotProgram {

    private ISubsystem<Void, Vector3> joy;
    private ISubsystem<MecanumSubsystem.ControlDataFrame, Void> driveTrain;

    @Override
    public void init(IRobot robot) {
        System.out.println("INIT");
        // Register custom subsystem type
        //robot.getSystemRegistry().registerProvider(ShooterSubsystem.TYPE, new ShooterSubsystem.Provider());

        // Initialize subsystems
        joy = robot.getSystemRegistry().getProvider(SingleJoySubsystem.TYPE).getSubsystem(1);
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
        stdMode.onInit(() -> driveTrain.bind(joy.output()
                .map(DataMappers.deadZone(0.2D))
                .map(v -> v.x(v.x() * 0.75D).y(v.y() * 0.75D).z(v.z() * 0.75D))
                .map(DataMappers.singleJoyMecanum())));
        stdMode.whileCondition(() -> true);
        robot.getOpManager().setDefaultOpMode(RobotMode.TELEOP, "standard");
    }

}

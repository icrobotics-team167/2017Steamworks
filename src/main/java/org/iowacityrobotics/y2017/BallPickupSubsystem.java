package org.iowacityrobotics.y2017;

import com.ctre.CANTalon;
import org.iowacityrobotics.roboed.api.IRobot;
import org.iowacityrobotics.roboed.api.IRobotProgram;
import org.iowacityrobotics.roboed.api.RobotMode;
import org.iowacityrobotics.roboed.api.data.IDataSource;
import org.iowacityrobotics.roboed.api.operations.IOpMode;
import org.iowacityrobotics.roboed.api.subsystem.ISubsystem;
import org.iowacityrobotics.roboed.impl.data.DataMappers;
import org.iowacityrobotics.roboed.impl.subsystem.impl.DualJoySubsystem;
import org.iowacityrobotics.roboed.impl.subsystem.impl.MecanumSubsystem;
import org.iowacityrobotics.roboed.util.collection.Pair;
import org.iowacityrobotics.roboed.util.math.Vector2;
import org.iowacityrobotics.roboed.util.robot.QuadraSpeedController;

//Written by Hang Sun

public class BallPickupSubsystem extends FRCTerminalSubsystem<integer> {
    public static final ISubsystemType<Integer, Void, ISinglePortSubsystemProvider<Integer, Void>> TYPE = new FRCSubsystemType<>();

    private final CANTalon m;

    protected BallPickupSubsystem(int p) {
        super(TYPE);
        this.m = new CANTalon(p);
    }

    @Override
    protected void processData(double v) {
        m.set(v)
    }

    public static class Provider implements IDualPortSubsystemProvider<Integer, Void> {

        @Override
        Public ISubsystem<Integer, Void> getSubsystem(int port) {
            return new BallPickupSubsystem(port);
        }
    }
}
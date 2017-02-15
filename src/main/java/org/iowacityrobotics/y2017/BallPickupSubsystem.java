package org.iowacityrobotics.y2017;

import com.ctre.CANTalon;
import org.iowacityrobotics.roboed.api.subsystem.ISubsystem;
import org.iowacityrobotics.roboed.api.subsystem.ISubsystemType;
import org.iowacityrobotics.roboed.api.subsystem.provider.ISinglePortSubsystemProvider;
import org.iowacityrobotics.roboed.impl.subsystem.FRCSubsystemType;
import org.iowacityrobotics.roboed.impl.subsystem.FRCTerminalSubsystem;

//Written by Hang Sun

public class BallPickupSubsystem extends FRCTerminalSubsystem<Boolean> {
    public static final ISubsystemType<Boolean, Void, ISinglePortSubsystemProvider<Boolean, Void>> TYPE = new FRCSubsystemType<>();

    private final CANTalon m;

    protected BallPickupSubsystem(int p) {
        super(TYPE);
        this.m = new CANTalon(p);
    }

    @Override
    protected void processData(Boolean v) {
        m.set(v ? 0.75D : 0D);
    }

    public static class Provider implements ISinglePortSubsystemProvider<Boolean, Void> {
        @Override
        public ISubsystem<Boolean, Void> getSubsystem(int port) {
            return new BallPickupSubsystem(port);
        }
    }
}
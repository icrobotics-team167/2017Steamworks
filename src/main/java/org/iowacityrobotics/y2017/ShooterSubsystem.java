package org.iowacityrobotics.y2017;

import com.ctre.CANTalon;
import org.iowacityrobotics.roboed.api.subsystem.ISubsystem;
import org.iowacityrobotics.roboed.api.subsystem.ISubsystemType;
import org.iowacityrobotics.roboed.api.subsystem.provider.IDualPortSubsystemProvider;
import org.iowacityrobotics.roboed.impl.subsystem.FRCSubsystemType;
import org.iowacityrobotics.roboed.impl.subsystem.FRCTerminalSubsystem;

/**
 * @author Evan Geng
 */
public class ShooterSubsystem extends FRCTerminalSubsystem<Integer> {

    public static final ISubsystemType<Integer, Void, Provider> TYPE = new FRCSubsystemType<>();

    private final CANTalon m1, m2;

    protected ShooterSubsystem(int p1, int p2) {
        super(TYPE);
        this.m1 = new CANTalon(p1);
        this.m2 = new CANTalon(p2);
    }

    @Override
    protected void processData(Integer vel) {
        m1.set(vel);
        m2.set(-vel);
    }

    public static class Provider implements IDualPortSubsystemProvider<Integer, Void> {

        @Override
        public ISubsystem<Integer, Void> getSubsystem(int port1, int port2) {
            return new ShooterSubsystem(port1, port2);
        }

    }

}

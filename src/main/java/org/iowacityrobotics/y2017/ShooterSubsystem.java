package org.iowacityrobotics.y2017;

import com.ctre.CANTalon;
import edu.wpi.first.wpilibj.VictorSP;
import org.iowacityrobotics.roboed.api.subsystem.ISubsystem;
import org.iowacityrobotics.roboed.api.subsystem.ISubsystemType;
import org.iowacityrobotics.roboed.api.subsystem.provider.IDualPortSubsystemProvider;
import org.iowacityrobotics.roboed.impl.subsystem.FRCSubsystemType;
import org.iowacityrobotics.roboed.impl.subsystem.FRCTerminalSubsystem;

/**
 * @author Evan Geng
 */
public class ShooterSubsystem extends FRCTerminalSubsystem<Boolean> {

    public static final ISubsystemType<Boolean, Void, IDualPortSubsystemProvider<Boolean, Void>> TYPE = new FRCSubsystemType<>();

    private final CANTalon m1;
    private final VictorSP m2;

    protected ShooterSubsystem(int p1, int p2) {
        super(TYPE);
        this.m1 = new CANTalon(p1);
        this.m2 = new VictorSP(p2);
    }

    @Override
    protected void processData(Boolean shooting) {
        m1.set(shooting ? 1 : 0);
        m2.set(shooting ? -1 : 0);
    }

    public static class Provider implements IDualPortSubsystemProvider<Boolean, Void> {

        @Override
        public ISubsystem<Boolean, Void> getSubsystem(int port1, int port2) {
            return new ShooterSubsystem(port1, port2);
        }

    }

}

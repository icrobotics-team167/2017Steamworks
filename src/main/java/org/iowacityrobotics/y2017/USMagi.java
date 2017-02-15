package org.iowacityrobotics.y2017;

import edu.wpi.first.wpilibj.Ultrasonic;
import org.iowacityrobotics.roboed.api.data.Data;
import org.iowacityrobotics.roboed.api.data.IDataSource;
import org.iowacityrobotics.roboed.api.subsystem.ISubsystem;
import org.iowacityrobotics.roboed.api.subsystem.ISubsystemType;
import org.iowacityrobotics.roboed.api.subsystem.provider.IDualPortSubsystemProvider;
import org.iowacityrobotics.roboed.impl.subsystem.FRCSourceSubsystem;
import org.iowacityrobotics.roboed.impl.subsystem.FRCSubsystemType;

/**
 * It's ultrasonic
 * @author Evan Geng and/or Bruno (Mars) R.
 */
public class USMagi extends FRCSourceSubsystem<Double> {

    public static final ISubsystemType<Void, Double, IDualPortSubsystemProvider<Void, Double>> TYPE = new FRCSubsystemType<>();

    private final IDataSource<Double> source;

    public USMagi(Ultrasonic us) {
        super(TYPE);
        this.source = Data.provider(us::getRangeMM);
    }

    @Override
    public IDataSource<Double> output() {
        return source;
    }

    public static class Provider implements IDualPortSubsystemProvider<Void, Double> {

        @Override
        public ISubsystem<Void, Double> getSubsystem(int port1, int port2) {
            return new USMagi(new Ultrasonic(port1, port2));
        }

    }

}

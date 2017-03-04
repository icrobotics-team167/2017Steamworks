package org.iowacityrobotics.y2017;

import org.iowacityrobotics.roboed.data.sink.Sink;
import org.iowacityrobotics.roboed.data.source.Source;

import java.util.function.Supplier;

/**
 * WE MUST TAKE THE DERIVATIVE OF THE INTEGRAL OF THE DERIVATIVE
 * @author Evan Geng
 */
public class Differential extends Source<Double> {

    private final DiffSink in;

    public Differential() {
        this.in = new DiffSink();
    }

    @Override
    public Double get() {
        return in.deltaData / (double)in.deltaTime;
    }

    public Differential bind(Source<Double> src) {
        in.bind(src);
        return this;
    }

    private static class DiffSink extends Sink<Double> {

        private long lastTime, deltaTime;
        private double lastData, deltaData;

        @Override
        protected void process(Double data) {
            long time = System.currentTimeMillis();
            deltaTime = time - lastTime;
            deltaData = data - lastData;
            lastTime = time;
            lastData = data;
        }

    }

}
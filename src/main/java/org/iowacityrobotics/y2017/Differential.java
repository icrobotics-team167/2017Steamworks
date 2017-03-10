package org.iowacityrobotics.y2017;

import org.iowacityrobotics.roboed.data.sink.Sink;
import org.iowacityrobotics.roboed.data.source.Source;
import org.iowacityrobotics.roboed.util.logging.Logs;

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
        return in.init ? in.deltaData / (double)in.deltaTime : null;
    }

    public Differential bind(Source<Double> src) {
        in.bind(src);
        return this;
    }

    private static class DiffSink extends Sink<Double> {

        private long lastTime = -1L, deltaTime;
        private double lastData = -1D, deltaData;
        private boolean init = false;

        @Override
        protected void process(Double data) {
            Logs.info("frame of diff data");
            if (lastTime == -1L || lastData == -1D) {
                lastTime = System.currentTimeMillis();
                lastData = data;
                init = true;
            } else {
                long time = System.currentTimeMillis();
                deltaTime = time - lastTime;
                deltaData = data - lastData;
                lastTime = time;
                lastData = data;
            }
        }

    }

}
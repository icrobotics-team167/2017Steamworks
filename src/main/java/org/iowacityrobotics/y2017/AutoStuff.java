package org.iowacityrobotics.y2017;

import com.kauailabs.navx.frc.AHRS;
import org.iowacityrobotics.roboed.data.Data;
import org.iowacityrobotics.roboed.data.sink.Sink;
import org.iowacityrobotics.roboed.data.source.Source;
import org.iowacityrobotics.roboed.robot.Flow;
import org.iowacityrobotics.roboed.util.collection.Pair;
import org.iowacityrobotics.roboed.util.logging.Logs;
import org.iowacityrobotics.roboed.util.math.Vector2;
import org.iowacityrobotics.roboed.util.math.Vector4;

import static org.iowacityrobotics.y2017.RobotMain.WOPEN;

/**
 * who needs convention
 */
public abstract class AutoStuff {

    protected Sink<Vector4> snkDrive;
    protected AHRS ahrs;

    public AutoStuff(Sink<Vector4> snkDrive, AHRS ahrs) {
        this.snkDrive = snkDrive;
        this.ahrs = ahrs;
    }

    public abstract void execute(int routine);

    public void drive(Vector2 vec, long time) {
        Data.pushState();
        ahrs.reset();
        final Vector4 moveVec = new Vector4(vec.x(), vec.y(), 0, 0);
        snkDrive.bind(Data.source(() -> moveVec/*.w(ahrs.getAngle())*/));
        Flow.waitFor(time);
        Data.popState();
    }

    public void ptTurn(double speed, float deltaAngle) {
        Data.pushState();
        final Vector4 moveVec = new Vector4(0, 0, -speed * Math.signum(deltaAngle), ahrs.getAngle());
        snkDrive.bind(Data.source(() -> moveVec));
        Flow.waitUntil(new NavAngle(ahrs, deltaAngle));
        Data.popState();
    }

    public static class NoVision extends AutoStuff {

        private final Sink<Double> snkShoot, snkEgg;

        public NoVision(Sink<Vector4> snkDrive, AHRS ahrs, Sink<Double> snkShoot, Sink<Double> snkEgg) {
            super(snkDrive, ahrs);
            this.snkShoot = snkShoot;
            this.snkEgg = snkEgg;
        }

        @Override
        public void execute(int routine) {
            Vector4 vec4 = new Vector4();
            Vector2 vec2 = new Vector2();
            switch (routine) {
                case 420: // Red boiler, strafe left
                    boilerShoot(1, vec4);
                    break;
                case 666: // Blue boiler, strafe right
                    boilerShoot(-1, vec4);
                    break;
                default: // Something else
                    Logs.info("Robot is Lee Sin!");
                    if (routine == 0) {
                        Logs.info("That's the center!!!! We gotta turn before we drive");
                        // TODO Turn a bit, drive that way, turn back
                    }
                    Logs.info("across the line we go");
                    drive(vec2.y(0.71), 1650L);
                    vec2.y(0);
                    break;
            }
        }

        public void boilerShoot(int strafeMult, Vector4 vec4) {
            Data.pushState();
            snkShoot.bind(Data.source(() -> 1D));
            snkEgg.bind(Data.source(() -> 0.8));
            Flow.waitFor(10000L);
            Data.pushState();
            snkDrive.bind(Data.source(() -> vec4.x(1D * strafeMult).z(0.14D * strafeMult)));
            Flow.waitFor(3000L);
            Data.popState();
            ptTurn(0.3D, 180F);
            Data.popState();
            vec4.x(0).z(0);
        }

    }

    public static class WithVision extends AutoStuff {

        private final Sink<Double> snkShoot, snkEgg, snkWs;
        private final Source<Pair<Vector4, Vector4>> srcVis;

        public WithVision(Sink<Vector4> snkDrive, AHRS ahrs, Sink<Double> snkShoot, Sink<Double> snkEgg, Sink<Double> snkWs, Source<Pair<Vector4, Vector4>> srcVis) {
            super(snkDrive, ahrs);
            this.snkShoot = snkShoot;
            this.snkEgg = snkEgg;
            this.snkWs = snkWs;
            this.srcVis = srcVis;
        }

        @Override
        public void execute(int routine) {
            Vector4 vec4 = new Vector4();
            Vector2 vec2 = new Vector2();
            switch (routine) {
                case 420: // Red boiler, strafe left
                    Logs.info("shooting at red boiler");
                    boilerShoot();
                    // TODO Turn towards the gear
                    break;
                case 666: // Blue boiler, strafe right
                    Logs.info("shooting at blue boiler");
                    boilerShoot();
                    // TODO Turn towards the gear
                    break;
            }

            Logs.info("drive fwd");
            if (routine != 0) { // Case: routine is not 0
                drive(vec2.y(0.35D), 2170L); // Drive forwards to line
                ptTurn(0.35, -30 * Math.signum(routine)); // Turn 30 degrees times n
            } else { // Case: routine is 0
                drive(vec2.y(0.+3D), 400L); // Drive forwards a bit
            }
            vec2.y(0); // Reset vec2

            Logs.info("rotate normal");
            autoVisionRotate(srcVis, vec4); // Turn until normal to reflector tape

            Logs.info("Approach quietly... .. ... . .");
            autoVisionApproach(srcVis, vec4, vec2); // Approach the peg and place gear

            Logs.info("release gear and back off");
            Data.pushState();
            snkWs.bind(Data.source(() -> WOPEN));
            Flow.waitFor(500);
            drive(vec2.y(-0.35), 500L);
            Data.popState();
            vec2.y(0);
        }

        public void boilerShoot() {
            Data.pushState();
            snkShoot.bind(Data.source(() -> 1D));
            snkEgg.bind(Data.source(() -> 0.8));
            Flow.waitFor(4000L);
            Data.popState();
            Logs.info("done! let's go the other way");
        }

        public void autoVisionRotate(Source<Pair<Vector4, Vector4>> srcVis, Vector4 vec4) {
            Data.pushState();
            Source<Vector4> srcVisionRotate = srcVis.map(Data.mapper(
                    c -> c == null ? Vector4.ZERO : vec4.z(-0.26 * Math.signum(c.getB().w() - c.getA().w()))
            ));
            snkDrive.bind(srcVisionRotate);
            Flow.waitUntil(() -> {
                Pair<Vector4, Vector4> c = srcVis.get();
                return c != null && Math.abs(c.getA().w() - c.getB().w()) < 2;
            });
            Data.popState();
            vec4.z(0);
        }

        public void autoVisionApproach(Source<Pair<Vector4, Vector4>> srcVis, Vector4 vec4, Vector2 vec2) {
            Data.pushState();
            Source<Double> srcPegDiff = srcVis.map(Data.mapper(
                    c -> c == null ? null : (c.getA().x() + c.getB().x()) / 2D - 320
            ));
            Source<Vector4> srcVisionStrafe = srcPegDiff.map(Data.mapper(
                    d -> d == null ? vec4.x(0).y(0.24) : vec4.x(-0.4 * Math.signum(d)).y(0.3)
            ));
            snkDrive.bind(srcVisionStrafe);
            Flow.waitUntil(() -> VisionDataProvider.timeDiff() > 750L);
            Data.popState();
            vec4.x(0).y(0);
            drive(vec2.y(0.32), 750L);
            vec2.y(0);
        }

    }

}

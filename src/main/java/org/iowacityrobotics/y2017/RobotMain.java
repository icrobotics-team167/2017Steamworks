package org.iowacityrobotics.y2017;

import com.jcraft.jsch.ChannelExec;
import com.jcraft.jsch.ChannelSftp;
import com.jcraft.jsch.JSchException;
import com.jcraft.jsch.Session;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.iowacityrobotics.roboed.data.Data;
import org.iowacityrobotics.roboed.data.sink.Sink;
import org.iowacityrobotics.roboed.data.source.Source;
import org.iowacityrobotics.roboed.robot.Flow;
import org.iowacityrobotics.roboed.robot.IRobotProgram;
import org.iowacityrobotics.roboed.robot.RobotMode;
import org.iowacityrobotics.roboed.subsystem.MapperSystems;
import org.iowacityrobotics.roboed.subsystem.SinkSystems;
import org.iowacityrobotics.roboed.subsystem.SourceSystems;
import org.iowacityrobotics.roboed.util.collection.Pair;
import org.iowacityrobotics.roboed.util.logging.LogLevel;
import org.iowacityrobotics.roboed.util.logging.Logs;
import org.iowacityrobotics.roboed.util.math.Vector2;
import org.iowacityrobotics.roboed.util.math.Vector4;
import org.iowacityrobotics.roboed.util.robot.MotorTuple4;
import org.iowacityrobotics.roboed.vision.CameraType;
import org.iowacityrobotics.roboed.vision.VisionServer;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.io.BufferedInputStream;
import java.io.File;
import java.io.InputStream;
import java.util.Enumeration;
import java.util.function.Supplier;
import java.util.jar.JarEntry;
import java.util.jar.JarFile;

public class RobotMain implements IRobotProgram {

    private AHRS ahrs;
    private Sink<Vector4> snkDrive;
    private NetworkTable tbl;

    @Override
    public void init() {
        Logs.setLevel(LogLevel.DEBUG);

        // Deploy vision code
        /*Session sess = null;
        try (JarFile jar = new JarFile(new File(getClass().getProtectionDomain().getCodeSource().getLocation().getPath()))) {
            Logs.info("Establishing SSH connection to rpi...");
            sess = SSHUtil.connect("raspberrypi.local", 22, "pi", "raspberry");
            ChannelSftp sftp = (ChannelSftp) sess.openChannel("sftp");
            sftp.connect();
            sftp.cd("/home/pi");

            Logs.info("Unpacking and transferring vision code...");
            Enumeration<JarEntry> entries = jar.entries();
            while (entries.hasMoreElements()) {
                JarEntry entry = entries.nextElement();
                String jarPath = entry.getName();
                if (jarPath.startsWith("python/") && !jarPath.endsWith("python/")) {
                    try (InputStream in = new BufferedInputStream(jar.getInputStream(entry))) {
                        String name = jarPath.substring(jarPath.lastIndexOf('/') + 1);
                        Logs.info("Transferring file: {}", name);
                        sftp.put(in, name);
                    }
                }
            }
            sftp.exit();

            Logs.info("Executing vision code...");
            ChannelExec exec = (ChannelExec) sess.openChannel("exec");
            exec.setCommand("killall -9 python; setsid python visionNetworkRun.py </dev/zero &>vision.log &");
            exec.connect();
            exec.disconnect();
        } catch (JSchException e) {
            Logs.warn("Host not found (?). Ignoring vision deploy.");
        } catch (Exception e) {
            Logs.error("Could not deploy vision code!", e);
            throw new RuntimeException(e);
        } finally {
            if (sess != null)
                sess.disconnect();
        }
        Logs.info("Vision code deploy done.");*/

        Logs.info("Initializing camera stream.");
        Supplier<Mat> cam = VisionServer.getCamera(CameraType.USB, 0);
        VisionServer.putImageSource("usb-cam", () -> {
            Mat frame = cam.get();
            //Imgproc.line(frame, new Point(), new Point(), new Scalar(1, 0, 0));
            return frame;
        });

        // Nav board
        ahrs = new AHRS(SerialPort.Port.kMXP);
        ahrs.reset();
        Source<Double> srcAccelY = Data.source(() -> (double)ahrs.getRawAccelY());
        Differential srcJerk = new Differential();

        // Drive train
        Source<Boolean> srcRev = SourceSystems.CONTROL.button(1, 4)
                .map(MapperSystems.CONTROL.toggle());
        Source<Double> srcSneak = SourceSystems.CONTROL.axis(1, 3);
        Source<Vector4> srcDrive = SourceSystems.CONTROL.dualJoy(1)
                .map(MapperSystems.DRIVE.dualJoyMecanum())
                .map(Data.mapper(v -> v.multiply(-1)))
                .inter(srcRev, Data.inter((v, r) -> {
                    double mult = r ? -1 : 1;
                    return v.x(v.x() * mult).y(v.y() * mult);
                }))
                .inter(srcSneak, Data.inter((v, t) -> {
                    double mult = 1 - 0.25 * t;
                    return v.x(v.x() * mult).y(v.y() * mult);
                }));
        MotorTuple4 talons = MotorTuple4.ofCANTalons(1, 4, 6, 3);
        talons.getFrontLeft().setInverted(true);
        talons.getFrontRight().setInverted(true);
        snkDrive = SinkSystems.DRIVE.mecanum(talons);

        // Climber
        Source<Double> srcClimb = SourceSystems.CONTROL.button(2, 8)
                .map(MapperSystems.CONTROL.buttonValue(0D, -1D));
        Sink<Double> snkClimb = SinkSystems.MOTOR.canTalon(2);

        // Shooter
        Source<Boolean> srcShoot = SourceSystems.CONTROL.button(2, 1);
        Sink<Double> snkShoot = SinkSystems.MOTOR.canTalon(8);

        // Agitator
        Source<Boolean> srcEggBtn = SourceSystems.CONTROL.button(2, 4);
        Source<Boolean> srcEgg = srcEggBtn.inter(srcShoot, Data.inter((a, b) -> a || b));
        Sink<Double> snkEgg = SinkSystems.MOTOR.victorSp(8);

        // Pickup
        Source<Double> srcPickup = SourceSystems.CONTROL.button(2, 2)
                .map(MapperSystems.CONTROL.buttonValue(0D, 0.75D));
        Sink<Double> snkPickup = SinkSystems.MOTOR.canTalon(7);

        // Windshield wiper
        //Source<Double> asdf = Data.source(() -> SmartDashboard.getNumber("theServoAngle", 0));
        Source<Double> srcWs = SourceSystems.CONTROL.button(2, 3)
                .map(MapperSystems.CONTROL.buttonValue(103D, 45D));
        Sink<Double> snkWs = SinkSystems.MOTOR.servo(0);

        // Ultrasonic sensor
        Source<Double> srcUs = SourceSystems.SENSOR.analog(0)
                .map(Data.mapper(v -> v * 3.4528D));

        // Vision data source
        Source<Pair<Vector4, Vector4>> srcVis = Data.cached(new VisionDataProvider(), 50L);

        tbl = NetworkTable.getTable("gearPlacerVision");

        // Auto routine control
        SendableChooser<Integer> rtCtrl = new SendableChooser<>();
        rtCtrl.addObject("left", -1);
        rtCtrl.addDefault("center", 0);
        rtCtrl.addObject("right", 1);
        rtCtrl.addObject("red boiler", 420);
        rtCtrl.addObject("blue boiler", 666);
        SmartDashboard.putData("routine", rtCtrl);

        SendableChooser<Boolean> doVision = new SendableChooser<>();
        doVision.addObject("do the vision stuff pls", true);
        doVision.addObject("don't do that!!!!", false);
        SmartDashboard.putData("do vision?", doVision);

        // Teleop mode
        RobotMode.TELEOP.setOperation(() -> {
            snkDrive.bind(srcDrive);
            snkClimb.bind(srcClimb);
            snkShoot.bind(srcShoot.map(MapperSystems.CONTROL.buttonValue(0D, 1D)));
            snkEgg.bind(srcEgg.map(MapperSystems.CONTROL.buttonValue(0D, 0.8D)));
            snkPickup.bind(srcPickup);
            snkWs.bind(srcWs);
            Flow.waitInfinite();
        });

        // Auto mode
        RobotMode.AUTO.setOperation(() -> {
            // Define some vecs for efficiency reasons
            final Vector2 vec2 = new Vector2();
            final Vector4 vec4 = new Vector4();

            // Determine routine number n <- {-1, 0, 1}
            int routine = rtCtrl.getSelected();

            // VISION???????
            boolean canSee = doVision.getSelected();

            // Let's raise the bar lads
            snkWs.bind(Data.source(() -> 103D));

            if (routine == 420) { // Red boiler, strafe left
                Data.pushState();
                snkShoot.bind(Data.source(() -> 1D));
                snkEgg.bind(Data.source(() -> 0.8));
                Flow.waitFor(10000L);
                Data.pushState();
                snkDrive.bind(Data.source(() -> vec4.x(1D).z(0.14D)));
                Flow.waitFor(3000L);
                Data.popState();
                ptTurn(0.3D, 180F);
                Data.popState();
                vec4.x(0).z(0);
            } else if (routine == 666) { // Blue boiler, strafe right
                Data.pushState();
                snkShoot.bind(Data.source(() -> 1D));
                snkEgg.bind(Data.source(() -> 0.8));
                Flow.waitFor(10000L);
                Data.pushState();
                snkDrive.bind(Data.source(() -> vec4.x(-1D).z(-0.14D)));
                Flow.waitFor(3000L);
                Data.popState();
                ptTurn(0.3D, 180F);
                Data.popState();
                vec4.x(0).z(0);
            } else {
                if (canSee) {
                    Logs.info("Step 1: drive fwd");
                    if (routine != 0) { // Case: routine is not 0
                        drive(vec2.y(0.35D), 2170L); // Drive forwards to line
                        //ptTurn(0.35, -30 * Math.signum(routine)); // Turn 30 degrees times n
                    } else { // Case: routine is 0
                        drive(vec2.y(0.3D), 400L); // Drive forwards a bit
                    }
                    vec2.y(0); // Reset vec2

                    int iterations = 3;
                    for (int i = 0; i < iterations; i++) {
                        Logs.info("Step 2: rotate by vision");
                        autoVisionRotate(srcVis, vec4);

                        Logs.info("Step 3: strafe by vision");
                        autoVisionStrafe(srcVis, vec4);

                        if (i != iterations - 1)
                            drive(vec2.y(0.35D), 1650L);
                    }

                    Logs.info("Step 4: drive into peg");
                    Data.pushState();
                    snkDrive.bind(Data.source(() -> vec4.y(0.265D)));
                    Flow.waitUntil(() -> tbl.getNumberArray("x", new double[1]).length < 1);
                    Flow.waitFor(750L);
                    Data.popState();

                    Logs.info("Step 5: release gear and back off");
                    Data.pushState();
                    snkWs.bind(Data.source(() -> 45D));
                    Flow.waitFor(500);
                    drive(vec2.y(-0.35), 500L);
                    Data.popState();
                    vec2.y(0);

                    Logs.info("Step 6: Cross the line");
                    switch (routine) {
                        case -1:
                            ptTurn(0.35, -30);
                            drive(vec2.y(0.3), 1000L);
                            break;
                        case 0:
                            ptTurn(0.35, 30);
                            drive(vec2.y(0.3), 800L);
                            ptTurn(0.35, -30);
                            drive(vec2.y(0.3), 1000L);
                            break;
                        case 1:
                            ptTurn(0.35, 30);
                            drive(vec2.y(0.3), 1000L);
                            break;
                    }
                    vec2.y(0);
                } else {
                    Logs.info("Robot is Lee Sin!");
                    if (routine != 0) {
                        Logs.info("That ain't the center! Bye");
                        drive(vec2.y(0.71), 1650L);
                        vec2.y(0);
                    } else {
                        Logs.info("That's the center!!!! We're gonna give that gear the big smackdown");
                        drive(vec2.y(0.375), 1500L);
                        vec2.y(0);

                        Logs.info("Here we go boys");
                        Data.pushState();
                        snkWs.bind(Data.source(() -> 45D));
                        Flow.waitFor(500);
                        drive(vec2.y(-0.35), 500L);
                        Data.popState();
                        vec2.y(0);

                        Logs.info("Gear placed (?). Crossing line.");
                        ptTurn(0.35, 30);
                        drive(vec2.y(0.3), 800L);
                        ptTurn(0.35, -30);
                        drive(vec2.y(0.3), 1000L);
                    }
                }
            }

            Flow.waitInfinite();
        });
    }

    private void autoVisionRotate(Source<Pair<Vector4, Vector4>> srcVis, Vector4 vec4) {
        Data.pushState(); // Push current state
        Source<Vector4> srcVisionRotate = srcVis.map(Data.mapper(
                c -> c == null ? Vector4.ZERO : vec4.z(-0.26 * Math.signum(c.getB().w() - c.getA().w()))
        ));
        snkDrive.bind(srcVisionRotate);
        Flow.waitUntil(() -> {
            Pair<Vector4, Vector4> c = srcVis.get();
            return c != null && Math.abs(c.getA().w() - c.getB().w()) < 2;
        });
        Data.popState(); // Pop previous state
        vec4.z(0);
    }

    private void autoVisionStrafe(Source<Pair<Vector4, Vector4>> srcVis, Vector4 vec4) {
        Data.pushState();
        Source<Double> srcPegDiff = srcVis.map(Data.mapper(
                c -> c == null ? null : (c.getA().x() + c.getB().x()) / 2D - (320)
        ));
        Source<Vector4> srcVisionStrafe = srcPegDiff.map(Data.mapper(
                d -> d == null ? Vector4.ZERO : vec4.x(-0.4 * Math.signum(d))
        ));
        snkDrive.bind(srcVisionStrafe);
        Flow.waitUntil(() -> {
            Double diff = srcPegDiff.get();
            return diff != null && Math.abs(diff) < 3;
        });
        Data.popState();
        vec4.x(0);
    }

    private void drive(Vector2 vec, long time) {
        Data.pushState();
        ahrs.reset();
        final Vector4 moveVec = new Vector4(vec.x(), vec.y(), 0, 0);
        snkDrive.bind(Data.source(() -> moveVec/*.w(ahrs.getAngle())*/));
        Flow.waitFor(time);
        Data.popState();
    }

    private void ptTurn(double speed, float deltaAngle) {
        Data.pushState();
        final Vector4 moveVec = new Vector4(0, 0, -speed * Math.signum(deltaAngle), ahrs.getAngle());
        snkDrive.bind(Data.source(() -> moveVec));
        Flow.waitUntil(new NavAngle(ahrs, deltaAngle));
        Data.popState();
    }

}
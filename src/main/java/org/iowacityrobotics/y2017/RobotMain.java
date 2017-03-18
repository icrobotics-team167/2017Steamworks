package org.iowacityrobotics.y2017;

import com.jcraft.jsch.ChannelExec;
import com.jcraft.jsch.ChannelSftp;
import com.jcraft.jsch.Session;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SerialPort;
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

import java.io.BufferedInputStream;
import java.io.File;
import java.io.InputStream;
import java.nio.file.Files;
import java.nio.file.StandardCopyOption;
import java.util.Enumeration;
import java.util.LinkedList;
import java.util.List;
import java.util.jar.JarEntry;
import java.util.jar.JarFile;

public class RobotMain implements IRobotProgram {

    private AHRS ahrs;
    private Sink<Vector4> snkDrive;

    @Override
    public void init() {
        Logs.setLevel(LogLevel.DEBUG);

        // Deploy vision code
        Session sess = null;
        try (JarFile jar = new JarFile(new File(getClass().getProtectionDomain().getCodeSource().getLocation().getPath()))) {
            Logs.info("Establishing SSH connection to rpi...");
            sess = SSHUtil.connect("raspberrypi.local", 22, "pi", "raspberry");
            ChannelSftp sftp = (ChannelSftp)sess.openChannel("sftp");
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
            ChannelExec exec = (ChannelExec)sess.openChannel("exec");
            exec.setCommand("screen -S vision; python ~/visionNetworkRun.py; detach");
            exec.connect();
            exec.disconnect();
        } catch (Exception e) {
            Logs.error("Could not deploy vision code!", e);
            throw new RuntimeException(e);
        } finally {
            if (sess != null)
                sess.disconnect();
        }

        // Nav board
        ahrs = new AHRS(SerialPort.Port.kMXP);
        ahrs.reset();
        Source<Double> srcAccelY = Data.source(() -> (double)ahrs.getRawAccelY());
        Differential srcJerk = new Differential();

        // Drive train
        Source<Vector4> srcDrive = SourceSystems.CONTROL.dualJoy(1)
                .map(MapperSystems.DRIVE.dualJoyMecanum())
                .map(Data.mapper(v -> v.x(v.x() * 0.75).y(v.y() * -0.75).z(v.z() * -1)));
        MotorTuple4 talons = MotorTuple4.ofCANTalons(1, 4, 6, 3);
        talons.getFrontLeft().setInverted(true);
        talons.getFrontRight().setInverted(true);
        snkDrive = SinkSystems.DRIVE.mecanum(talons);

        // Climber
        Source<Double> srcClimb = SourceSystems.CONTROL.button(1, 8)
                .map(MapperSystems.CONTROL.buttonValue(0D, -0.75D));
        Sink<Double> snkClimb = SinkSystems.MOTOR.canTalon(2);

        // Shooter
        Source<Boolean> srcShoot = SourceSystems.CONTROL.button(2, 1);
        Sink<Double> snkShoot = SinkSystems.MOTOR.canTalon(8);

        // Agitator
        Source<Boolean> srcEggBtn = SourceSystems.CONTROL.button(2, 4);
        Source<Boolean> srcEgg = srcEggBtn.inter(srcShoot, Data.inter((a, b) -> a || b));
        Sink<Double> snkEgg = SinkSystems.MOTOR.victorSp(5);

        // Pickup
        Source<Double> srcPickup = SourceSystems.CONTROL.button(2, 2)
                .map(MapperSystems.CONTROL.buttonValue(0D, 0.75D));
        Sink<Double> snkPickup = SinkSystems.MOTOR.canTalon(7);

        // Windshield wiper
        Source<Double> srcWs = SourceSystems.CONTROL.button(2, 3)
                .map(MapperSystems.CONTROL.buttonValue(175D, 115D));
        Sink<Double> snkWs = SinkSystems.MOTOR.servo(0);

        // Ultrasonic sensor
        Source<Double> srcUs = SourceSystems.SENSOR.analog(0)
                .map(Data.mapper(v -> v * 3.4528D));

        // Vision data source
        Source<Pair<Vector4, Vector4>> srcVis = Data.cached(new VisionDataProvider(), 50L);

        // Auto routine control
        SendableChooser<Integer> rtCtrl = new SendableChooser<>();
        rtCtrl.addObject("left", -1);
        rtCtrl.addDefault("center", 0);
        rtCtrl.addObject("right", 1);
        SmartDashboard.putData("routine", rtCtrl);

        // Teleop mode
        RobotMode.TELEOP.setOperation(() -> {
            snkDrive.bind(srcDrive);
            snkClimb.bind(srcClimb);
            snkShoot.bind(srcShoot.map(MapperSystems.CONTROL.buttonValue(0D, 0.9D)));
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

            Logs.info("Step 1: drive fwd");

            if (routine != 0) { // Case: routine is not 0
                drive(vec2.y(0.35D), 1650L); // Drive forwards a bit
                ptTurn(0.35, 30 * Math.signum(routine)); // Turn 30 degrees times n
            } else { // Case: routine is 0
                drive(vec2.y(0.35D), 500L); // Drive forwards a bit
            }
            vec2.y(0); // Reset vec2

            Logs.info("Step 2: rotate by vision");
            Flow.waitFor(1300L);

            Data.pushState(); // Push current state
            Source<Vector4> srcVisionRotate = srcVis.map(Data.mapper(
                    c -> c == null ? Vector4.ZERO : vec4.z(0.31 * Math.signum(c.getB().w() - c.getA().w()))
            ));
            snkDrive.bind(srcVisionRotate);
            Flow.waitUntil(() -> {
                Pair<Vector4, Vector4> c = srcVis.get();
                return c != null && Math.abs(c.getA().w() - c.getB().w()) < 4;
            });
            Data.popState(); // Pop previous state
            vec4.z(0);

            Logs.info("Step 3: strafe by vision");
            Flow.waitFor(1300L);

            Data.pushState();
            Source<Double> srcPegDiff = srcVis.map(Data.mapper(
                c -> c == null ? null : (c.getA().x() + c.getB().x()) / 2D - 268
            ));
            Source<Vector4> srcVisionStrafe = srcPegDiff.map(Data.mapper(
                    d -> d == null ? Vector4.ZERO : vec4.x(-0.4 * Math.signum(d))
            ));
            snkDrive.bind(srcVisionStrafe);
            srcJerk.bind(srcAccelY);
            Flow.waitUntil(() -> {
                Double diff = srcPegDiff.get();
                return diff != null && Math.abs(diff) < 5;
            });
            Data.popState();
            vec4.x(0);

            Logs.info("Step 4: drive into peg");
            Flow.waitFor(1300L);

            Data.pushState();
            snkDrive.bind(Data.source(() -> vec4.y(0.32D)));
            Flow.waitUntil(() -> {
                Double jerk = srcJerk.get();
                if (jerk == null)
                    return false;
                Logs.info(jerk);
                return jerk > 0.1D;
            });
            Data.popState();

            Logs.info("Step 5: release gear and back off");
            Flow.waitFor(1300L);

            Data.pushState();
            snkWs.bind(Data.source(() -> 115D));
            Flow.waitFor(500);
            drive(vec2.y(-0.35), 500L);
            Data.popState();
            vec2.y(0);

            // TODO Cross line
        });
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
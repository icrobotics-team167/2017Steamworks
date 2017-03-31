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
import org.iowacityrobotics.roboed.util.math.Vector4;
import org.iowacityrobotics.roboed.util.robot.MotorTuple4;
import org.iowacityrobotics.roboed.vision.CameraType;
import org.iowacityrobotics.roboed.vision.VisionServer;
import org.opencv.core.Mat;

import java.io.BufferedInputStream;
import java.io.File;
import java.io.InputStream;
import java.util.Enumeration;
import java.util.function.Supplier;
import java.util.jar.JarEntry;
import java.util.jar.JarFile;

public class RobotMain implements IRobotProgram {

    public static final double WCLOSED = 103D, WOPEN = 45D;

    private AHRS ahrs;
    private Sink<Vector4> snkDrive;
    private NetworkTable tbl;

    @Override
    public void init() {
        Logs.setLevel(LogLevel.DEBUG);

        // Deploy vision code
        Session sess = null;
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
        Logs.info("Vision code deploy done.");

        /*Logs.info("Initializing camera stream.");
        Supplier<Mat> cam = VisionServer.getCamera(CameraType.USB, 0);
        VisionServer.putImageSource("usb-cam", () -> {
            Mat frame = cam.get();
            //Imgproc.line(frame, new Point(), new Point(), new Scalar(1, 0, 0));
            return frame;
        });*/

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
        talons.getRearLeft().setInverted(true);
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
                .map(MapperSystems.CONTROL.buttonValue(WCLOSED, WOPEN));
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

        SendableChooser<Supplier<AutoStuff>> doVision = new SendableChooser<>();
        doVision.addObject("do the vision stuff pls", () -> new AutoStuff.WithVision(
                snkDrive, ahrs, snkShoot, snkEgg, snkWs, srcVis
        ));
        doVision.addObject("don't do that!!!!", () -> new AutoStuff.NoVision(
                snkDrive, ahrs, snkShoot, snkEgg
        ));
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
            int routine = rtCtrl.getSelected(); // Determine routine number n <- {-1, 0, 1}
            AutoStuff auto = doVision.getSelected().get(); // Check if we want to do vision
            snkWs.bind(Data.source(() -> WCLOSED)); // Raise the windshield
            auto.execute(routine); // Do the routine
            Flow.waitInfinite(); // Stall, in case we have the gear on board
        });
    }

}
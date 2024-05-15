package org.firstinspires.ftc.teamcode;

import com.amarcolini.joos.command.Component;
import com.amarcolini.joos.command.Robot;
import com.amarcolini.joos.dashboard.JoosConfig;
import com.amarcolini.joos.geometry.Angle;
import com.amarcolini.joos.hardware.CRServo;
import com.amarcolini.joos.hardware.IMUAngleSensor;
import com.amarcolini.joos.hardware.Motor;
import com.amarcolini.joos.hardware.MotorGroup;
import com.amarcolini.joos.hardware.Servo;
import com.amarcolini.joos.util.NanoClock;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;

import kotlin.Pair;

import java.util.List;

//TODO: change all IDs to what they are in your robot configuration
@JoosConfig
public class SampleRobot extends Robot {
    public final IMUAngleSensor headingSensor = new IMUAngleSensor(
            hMap, "imu",
            RevHubOrientationOnRobot.LogoFacingDirection.UP, // TODO: Set these to what they are on your robot
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
    );

    public static Angle frontLeftOffset = Angle.deg(19);
    public static Angle backLeftOffset = Angle.deg(355.15);
    public static Angle backRightOffset = Angle.deg(218.6);
    public static Angle frontRightOffset = Angle.deg(2.3);

    public final SampleSwerveDrive drive = new SampleSwerveDrive(
            new SampleSwerveModule(
                    new AxonAngleSensor(
                            hMap.get(AnalogInput.class, "front_left_sensor"),
                            frontLeftOffset, false
                    ),
                    new Motor(hMap, "front_left", Motor.Type.GOBILDA_MATRIX),
                    new CRServo(hMap, "front_left_servo")
            ),
            new SampleSwerveModule(
                    new AxonAngleSensor(
                            hMap.get(AnalogInput.class, "back_left_sensor"),
                            backLeftOffset, false
                    ),
                    new Motor(hMap, "back_left", Motor.Type.GOBILDA_MATRIX),
                    new CRServo(hMap, "back_left_servo")
            ),
            new SampleSwerveModule(
                    new AxonAngleSensor(
                            hMap.get(AnalogInput.class, "back_right_sensor"),
                            backRightOffset, false
                    ),
                    new Motor(hMap, "back_right", Motor.Type.GOBILDA_MATRIX).reversed(),
                    new CRServo(hMap, "back_right_servo")
            ),
            new SampleSwerveModule(
                    new AxonAngleSensor(
                            hMap.get(AnalogInput.class, "front_right_sensor"),
                            frontRightOffset, false
                    ),
                    new Motor(hMap, "front_right", Motor.Type.GOBILDA_MATRIX).reversed(),
                    new CRServo(hMap, "front_right_servo")
            )
    );

    public final Outtake outtake = new Outtake(
            new Servo(hMap, "arm"),
            new Servo(hMap, "wrist")
    );

    public final Slides slides = new Slides(
            new Motor(hMap, "lift", Motor.Type.GOBILDA_435)
    );

    private double lastUpdate;

    @Override
    public void init() {
        register(drive, outtake, slides);

        List<LynxModule> modules = hMap.getAll(LynxModule.class);
        modules.forEach((m) -> m.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));
        register(Component.of(() -> modules.forEach(LynxModule::clearBulkCache)));

        NanoClock clock = NanoClock.getSystem();
        register(Component.of(() -> {
            double now = clock.seconds();
            double dt = now - lastUpdate;
            telem.addData("Loop speed (ms)", Math.round(dt * 1000.0))
                    .addData("Loop speed (hZ)", Math.round(1.0 / dt));
            lastUpdate = now;
        }));

        drive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }
}
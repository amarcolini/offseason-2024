package org.firstinspires.ftc.teamcode.tuning;

import com.amarcolini.joos.command.Command;
import com.amarcolini.joos.command.CommandOpMode;
import com.amarcolini.joos.command.SequentialCommand;
import com.amarcolini.joos.command.WaitCommand;
import com.amarcolini.joos.dashboard.JoosConfig;
import com.amarcolini.joos.drive.SwerveModule;
import com.amarcolini.joos.geometry.Angle;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SampleRobot;

@TeleOp
@JoosConfig
public class ModuleTuningTest extends CommandOpMode {
    @Register
    private SampleRobot robot;
    public static Angle startAngle = Angle.deg(0);
    public static Angle endAngle = Angle.deg(60);
    public static double delay = 3.0;
    private int moduleIndex = 0;

    private Angle targetOrientation = Angle.deg(0.0);

    @Override
    public void preInit() {
        unregister(robot.drive);
        new SequentialCommand(
                Command.of(() -> {
                    robot.drive.getModules().get(moduleIndex).setModuleOrientation(startAngle);
                    targetOrientation = startAngle;
                }),
                new WaitCommand(delay),
                Command.of(() -> {
                    robot.drive.getModules().get(moduleIndex).setModuleOrientation(endAngle);
                    targetOrientation = endAngle;
                }),
                new WaitCommand(delay)
        ).onExecute(() -> robot.drive.getModules().get(moduleIndex).update()).repeatForever().schedule();

        schedule(true, () -> {
            telem.addLine("Tuning module " + moduleIndex + ". Press A/X to switch modules.");
            Angle orientation = robot.drive.getModules().get(moduleIndex).getModuleOrientation();
            telem.addData("module orientation", orientation)
                    .addData("target orientation", targetOrientation)
                    .addData("error", targetOrientation.minus(orientation).normDelta());
        });

        map(gamepad().p1.a0::isJustActivated, () -> {
            SwerveModule old = robot.drive.getModules().get(moduleIndex);
            old.setModuleOrientation(old.getModuleOrientation());
            moduleIndex = moduleIndex + 1 > robot.drive.getModules().size() ? 0 : moduleIndex + 1;
        });
    }
}
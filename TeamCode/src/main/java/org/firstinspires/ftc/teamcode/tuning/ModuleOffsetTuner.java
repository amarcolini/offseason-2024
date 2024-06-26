package org.firstinspires.ftc.teamcode.tuning;

import com.amarcolini.joos.command.CommandOpMode;
import com.amarcolini.joos.drive.PIDSwerveModule;
import com.amarcolini.joos.drive.SwerveModule;
import com.amarcolini.joos.geometry.Angle;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.AxonAngleSensor;
import org.firstinspires.ftc.teamcode.SampleRobot;
import org.firstinspires.ftc.teamcode.SampleSwerveDrive;
import org.firstinspires.ftc.teamcode.SampleSwerveModule;

import java.util.List;

@TeleOp
public class ModuleOffsetTuner extends CommandOpMode {
    @Register
    private SampleRobot robot;

    @Override
    public void preInit() {
        unregister(robot.drive);
        for (SwerveModule module : robot.drive.getModules()) {
            if (module instanceof PIDSwerveModule) ((PIDSwerveModule) module).setModulePower(0.0);
        }
        telem.addLine("Turn all modules to their 0 orientation (pointing forwards).").setRetained(true);
        schedule(true, () -> {
            List<SwerveModule> modules = robot.drive.getModules();
            List<Angle> orientations = robot.drive.getModuleOrientations();
            for (int i = 0; i < modules.size(); i++) {
                SwerveModule module = modules.get(i);
                if (module instanceof SampleSwerveModule && ((SampleSwerveModule) module).moduleOrientationSensor instanceof AxonAngleSensor) {
                    Angle offset = ((AxonAngleSensor) ((SampleSwerveModule) module).moduleOrientationSensor).offset;
                    telem.addData("offset for module " + i, orientations.get(i).minus(offset).unaryMinus());
                }
                telem.addData("orientation of module " + i, orientations.get(i));
            }
        });
    }
}
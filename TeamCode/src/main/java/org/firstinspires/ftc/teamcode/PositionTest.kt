package org.firstinspires.ftc.teamcode

import com.amarcolini.joos.command.CommandOpMode
import com.amarcolini.joos.dashboard.JoosConfig
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@TeleOp
@JoosConfig
class PositionTest : CommandOpMode() {
    private val robot by robot<SampleRobot>()

    companion object {
        var armPosition = 0.5
        var wristPosition = 0.0
        var slidePosition = 0.0
    }

    override fun preInit() {

    }

    override fun preStart() {
        robot.slides.motor.resetEncoder()
        schedule(true) {
            robot.slides.setTarget(slidePosition)
            robot.outtake.arm.position = armPosition
            robot.outtake.wrist.position = wristPosition
        }
    }
}
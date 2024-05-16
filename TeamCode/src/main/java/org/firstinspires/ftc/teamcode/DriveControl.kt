package org.firstinspires.ftc.teamcode

import com.amarcolini.joos.command.Command
import com.amarcolini.joos.command.CommandOpMode
import com.amarcolini.joos.command.TimeCommand
import com.amarcolini.joos.dashboard.JoosConfig
import com.amarcolini.joos.geometry.Pose2d
import com.amarcolini.joos.util.rad
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import kotlin.math.pow

/**
 * Drive controls:
 *  - joysticks to drive
 *  - A to toggle intake
 *  - Dpad left to reverse intake
 *  - Y to toggle transfer positions (works anywhere)
 *  - X to toggle arm rest/out
 *  - B to release switch
 */
@TeleOp
@JoosConfig
class DriveControl : CommandOpMode() {
    private val robot by robot<SampleRobot>()

    companion object {
        /**
         * The speed that the position of the slides can change.
         */
        var slideSpeed = 1000.0
    }

    override fun preInit() {
        robot.outtake.rest().schedule()
        schedule {
            robot.slides.resetEncoder()
        }

        schedule(true) {
            val leftStick = gamepad.p1.getLeftStick()
            val rightStick = gamepad.p1.getRightStick()
            robot.drive.setDrivePower(
                Pose2d(
                    -leftStick.y,
                    -leftStick.x,
                    -rightStick.x.rad
                )
            )
        }

        robot.slides.defaultCommand = TimeCommand { t, dt ->
            if (robot.outtake.state != Outtake.State.TRANSFER) {
                val new =
                    robot.slides.targetPosition + slideSpeed * dt * (gamepad.p1.right_trigger.value - gamepad.p1.left_trigger.value).pow(
                        3
                    )
                robot.slides.setTarget(new.coerceIn(0.0, 450.0))
            }
            false
        }.requires(robot.slides).setInterruptable(true)

        map(gamepad.p1.a0::isJustActivated, Command.of {
            robot.intake.state = when (robot.intake.state) {
                Intake.State.INTAKING -> Intake.State.STOPPED
                Intake.State.REVERSE -> Intake.State.STOPPED
                Intake.State.STOPPED -> Intake.State.INTAKING
            }
        }.requires(robot.intake))

        map(gamepad.p1.y0::isJustActivated, robot.transfer())

        map(gamepad.p1.x0::isJustActivated, Command.select(robot.outtake) {
            when (robot.outtake.state) {
                Outtake.State.TRANSFER -> Command.empty()
                Outtake.State.REST -> robot.outtake.out()
                Outtake.State.OUT -> robot.outtake.rest()
            }
        })

        map(gamepad.p1.b0::isJustActivated, Command.select(robot.outtake) {
            when (robot.outtake.state) {
                Outtake.State.TRANSFER -> robot.outtake.release()
                Outtake.State.REST -> Command.empty()
                Outtake.State.OUT -> if (robot.outtake.isOpen) robot.outtake.close() else robot.outtake.release()
            }
        })

        map(gamepad.p1.dpad_left::isJustActivated, Command.of {
            robot.intake.state = Intake.State.REVERSE
        }.requires(robot.intake))
    }
}
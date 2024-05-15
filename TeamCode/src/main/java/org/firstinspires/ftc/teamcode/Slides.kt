package org.firstinspires.ftc.teamcode

import com.amarcolini.joos.command.AbstractComponent
import com.amarcolini.joos.control.PIDCoefficients
import com.amarcolini.joos.control.PIDController
import com.amarcolini.joos.dashboard.JoosConfig
import com.amarcolini.joos.hardware.Motor
import com.amarcolini.joos.util.NanoClock

@JoosConfig
class Slides(val motor: Motor) : AbstractComponent() {
    companion object {
        val pidCoeffs = PIDCoefficients(0.02, 0.01, 0.0001)
        var kG = 0.4
    }

    private val controller = PIDController(pidCoeffs, object : NanoClock {
        override fun seconds(): Double = System.nanoTime() * 1e-9
    })
    fun setTarget(position: Double) {
        controller.targetPosition = position
        controller.reset()
        controller.update(motor.currentPosition.toDouble())
    }

    init {
        subcomponents += motor
        controller.setOutputBounds(-1.0, 1.0)
        controller.tolerance = 15.0
    }

    override fun update() {
        telem.addData("target", controller.targetPosition)
        telem.addData("lift position", motor.currentPosition)
        val output = controller.update(motor.currentPosition.toDouble(), motor.velocity)
        motor.power = output + kG
        telem.addData("output", output)
        telem.addData("motor power", motor.power)
    }
}
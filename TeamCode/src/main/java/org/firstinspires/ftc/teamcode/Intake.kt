package org.firstinspires.ftc.teamcode

import com.amarcolini.joos.command.AbstractComponent
import com.amarcolini.joos.hardware.Motor

class Intake(val motor: Motor) : AbstractComponent() {
    init {
        subcomponents += motor
        motor.zeroPowerBehavior = Motor.ZeroPowerBehavior.BRAKE
    }

    enum class State(val power: Double) {
        INTAKING(-1.0),
        REVERSE(0.3),
        STOPPED(0.0)
    }

    var state = State.STOPPED
        set(value) {
            motor.power = value.power
            field = value
        }
}
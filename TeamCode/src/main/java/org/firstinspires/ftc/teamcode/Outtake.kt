package org.firstinspires.ftc.teamcode

import com.amarcolini.joos.command.AbstractComponent
import com.amarcolini.joos.command.Command
import com.amarcolini.joos.dashboard.JoosConfig
import com.amarcolini.joos.geometry.Vector2d
import com.amarcolini.joos.hardware.CRServo
import com.amarcolini.joos.hardware.Servo

@JoosConfig
class Outtake(val arm: Servo, val wrist: Servo, val latch: Servo) : AbstractComponent() {
    companion object {
        var armTransfer = 1.0
        var wristTransfer = 1.0
        var armRest = 0.8
        var wristRest = 0.5
        var armOut = 0.6
        var wristOut = 0.0
        var latchClose = 0.0
        var latchOpen = 1.0
    }

    enum class State {
        TRANSFER,
        REST,
        OUT
    }

    var state: State = State.REST
        private set

    var isOpen = false
        private set

    init {
        subcomponents += listOf(arm, wrist)
    }

    fun rest(): Command = Command.of {
        latch.position = latchClose
        isOpen = false
        arm.position = armRest
        state = State.REST
    }.requires(this) wait 0.5 then {
        wrist.position = wristRest
    }

    fun out(): Command = Command.of {
//        latch.position = latchClose
        arm.position = armOut
        wrist.position = wristOut
        state = State.OUT
    }.requires(this) wait 1.0

    fun release(): Command = Command.of {
        latch.position = latchOpen
        isOpen = true
    }.requires(this) wait 0.5

    fun close(): Command = Command.of {
        latch.position = latchClose
        isOpen = false
    }.requires(this) wait 0.5


    fun transfer(): Command = Command.of {
        state = State.TRANSFER
        wrist.position = wristTransfer
        latch.position = latchOpen
        isOpen = true
    } wait 1.0 then {
        arm.position = armTransfer
    } wait 1.0
}
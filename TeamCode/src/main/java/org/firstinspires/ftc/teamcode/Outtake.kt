package org.firstinspires.ftc.teamcode

import com.amarcolini.joos.command.AbstractComponent
import com.amarcolini.joos.command.Command
import com.amarcolini.joos.dashboard.JoosConfig
import com.amarcolini.joos.geometry.Vector2d
import com.amarcolini.joos.hardware.CRServo
import com.amarcolini.joos.hardware.Servo

@JoosConfig
class Outtake(val arm: Servo, val wrist: Servo) : AbstractComponent() {
    companion object {

    }
}
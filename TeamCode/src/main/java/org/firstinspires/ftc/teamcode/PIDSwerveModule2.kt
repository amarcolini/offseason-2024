package org.firstinspires.ftc.teamcode

import com.amarcolini.joos.control.PIDCoefficients
import com.amarcolini.joos.control.PIDController
import com.amarcolini.joos.drive.SwerveModule
import com.amarcolini.joos.geometry.Angle
import com.amarcolini.joos.util.rad
import com.amarcolini.joos.util.wrap
import kotlin.math.PI
import kotlin.math.abs

/**
 * A basic [SwerveModule] implementation using a [PIDController] to hold module position.
 */
abstract class PIDSwerveModule2(
    pidCoefficients: PIDCoefficients
) : SwerveModule() {
    private var targetSpeed: (Double) -> Unit = { mult: Double ->
        setCorrectedDrivePower(0.0)
    }
    @JvmField
    protected val pidController = PIDController(pidCoefficients)

    init {
        pidController.setInputBounds(-PI / 2, PI / 2)
    }

    final override fun setModuleOrientation(angle: Angle) {
        pidController.targetPosition = angle.radians
    }

    final override fun setWheelVelocity(velocity: Double, acceleration: Double) {
        targetSpeed = {
            setCorrectedWheelVelocity(velocity * it, acceleration * it)
        }
    }

    fun getTargetOrientation() = pidController.targetPosition.rad

    /**
     * Sets the corrected wheel velocity (and acceleration) of the wheel motor.
     */
    protected abstract fun setCorrectedWheelVelocity(velocity: Double, acceleration: Double)

    final override fun setDrivePower(power: Double) {
        targetSpeed = {
            setCorrectedDrivePower(power * it)
        }
    }

    /**
     * Sets the corrected wheel motor power (normalized voltage) on the interval `[-1.0, 1.0]`.
     */
    protected abstract fun setCorrectedDrivePower(power: Double)

    override fun update() {
        val orientation = getModuleOrientation()
        val direction = pidController.targetPosition.rad.vec() dot orientation.vec()
//        val direction =
//            if (abs((pidController.targetPosition - orientation.radians).wrap(-PI, PI)) <= (PI / 2)) 1.0
//            else -1.0
        setModulePower(pidController.update(orientation.radians))
        targetSpeed.invoke(direction)
    }

    /**
     * Sets the power of the actuator which rotates the module.
     */
    abstract fun setModulePower(power: Double)
}
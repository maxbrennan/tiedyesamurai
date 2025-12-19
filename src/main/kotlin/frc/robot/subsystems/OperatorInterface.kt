package frc.robot.subsystems

import teaforge.platform.RoboRio.HidValue

data class DriverControls(
    val leftJoystickX: Double = 0.0,
    val leftJoystickY: Double = 0.0,
    val rightJoystickX: Double = 0.0,
    val prevX: Boolean = false,
    val x: Boolean = false,
)

data class ManipulatorControls(
    val leftJoystickY: Double = 0.0,
    val prevX: Boolean = false,
    val x: Boolean = false,
    val dpadDown: Boolean = false,
    val dpadDownPressed: Boolean = false,
    val dpadUp: Boolean = false,
    val dpadUpPressed: Boolean = false,
)

data class OperatorInterfaceState(
    val driver: DriverControls = DriverControls(),
    val manipulator: ManipulatorControls = ManipulatorControls(),
)

fun OperatorInterfaceState.updateDriver(controllerState: HidValue): OperatorInterfaceState {
    val axisValues = controllerState.axisValues
    return try {
        copy(
            driver = driver.copy(
                leftJoystickX = axisValues[0],
                leftJoystickY = -axisValues[1],
                rightJoystickX = axisValues[2],
                prevX = driver.x,
                x = controllerState.buttonValues[1],
            )
        )
    } catch (_: Exception) {
        this
    }
}

fun OperatorInterfaceState.updateManipulator(controllerState: HidValue): OperatorInterfaceState {
    val axisValues = controllerState.axisValues
    return try {
        val dpadDown = controllerState.buttonValues[4]
        val dpadUp = controllerState.buttonValues[5]
        val currentX = controllerState.buttonValues[1]
        copy(
            manipulator = manipulator.copy(
                leftJoystickY = -axisValues[1],
                prevX = manipulator.x,
                x = currentX,
                dpadDown = dpadDown,
                dpadUp = dpadUp,
                dpadDownPressed = !manipulator.dpadDown && dpadDown,
                dpadUpPressed = !manipulator.dpadUp && dpadUp
            )
        )
    } catch (_: Exception) {
        this
    }
}

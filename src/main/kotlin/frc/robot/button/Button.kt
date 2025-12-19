package frc.robot.button

import frc.robot.Message
import frc.robot.Model
import teaforge.platform.RoboRio.*

typealias ButtonMessage = Message.Button

object ButtonSubsystem {

    data class Model(
        val robotActivationButtonDownPreviousValue: DioPortStatus,
        val switchedOn: Boolean
    )

    sealed interface Message {
        data class RobotActivationButtonValue(val value: DioPortStatus) : Message
    }

    fun update(msg: Message, model: Model): Pair<Model, List<Effect<ButtonMessage>>> {
        return when (msg) {
            is Message.RobotActivationButtonValue -> {
                val updatedSwitchedOn =
                    when (Pair(msg.value, model.robotActivationButtonDownPreviousValue)
                    ) {
                        Pair(DioPortStatus.Closed, DioPortStatus.Open) ->
                            !model.switchedOn

                        else -> model.switchedOn
                    }

                val effects: List<Effect<ButtonMessage>> =
                    if (updatedSwitchedOn) {
                        listOf(Effect.SetPwmMotorSpeed(PwmPort.Zero, 0.5))
                    } else {
                        listOf(Effect.SetPwmMotorSpeed(PwmPort.Zero, 0.0))
                    }

                val updatedModel =
                    model.copy(
                        switchedOn = updatedSwitchedOn,
                        robotActivationButtonDownPreviousValue = msg.value
                    )

                Pair(updatedModel, effects)
            }
        }
    }

    fun subscriptions(model: Model): List<Subscription<ButtonMessage>> {
        return listOf(
            Subscription.DioPortValue(
                port = DioPort.Six,
                millisecondsBetweenReads = 50,
                message = { value -> ButtonMessage(Message.RobotActivationButtonValue(value)) }
            )
        )
    }

    fun init(args: List<String>): Pair<Model, List<Effect<ButtonMessage>>> {
        return Pair(Model(DioPortStatus.Open, false), emptyList())
    }

}

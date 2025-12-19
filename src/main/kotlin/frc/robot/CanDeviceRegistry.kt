package frc.robot

import teaforge.platform.RoboRio.CanDeviceToken

/**
 * Lightweight registry that maps [CanDevice] descriptors to the tokens returned by the platform
 * during device initialization.
 */
class CanDeviceRegistry private constructor(
    private val backing: Map<CanDevice<*>, CanDeviceToken>
) {
    constructor() : this(emptyMap())

    fun plus(device: CanDevice<*>, token: CanDeviceToken): CanDeviceRegistry =
        CanDeviceRegistry(backing + (device to token))

    @Suppress("UNCHECKED_CAST")
    fun <T : CanDeviceToken> get(device: CanDevice<T>): Maybe<T> =
        (backing[device] as? T)?.let { Maybe.Some(it) } ?: Maybe.None

    fun contains(device: CanDevice<*>) = device in backing
}

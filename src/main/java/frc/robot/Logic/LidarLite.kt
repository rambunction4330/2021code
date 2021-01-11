package frc.robot.Logic


import edu.wpi.first.hal.I2CJNI
import edu.wpi.first.wpilibj.*
import java.nio.ByteBuffer



class LidarLite(port: I2C.Port)  {

    private val k_deviceAddress: Byte = 0x62

    private var m_port = port.value

    private val m_buffer = ByteBuffer.allocateDirect(2)

    init {
        I2CJNI.i2CInitialize(m_port)
    }

    fun startMeasuring() {
        writeRegister(0x04, 0x08 or 32) // default plus bit 5
        writeRegister(0x11, 0xff)
        writeRegister(0x00, 0x04)
    }

    fun stopMeasuring() {
        writeRegister(0x11, 0x00)
    }

    fun getDistance(): Int {
        return (readShort(0x8f).toInt() / 2.54).toInt()
    }

    private fun writeRegister(address: Int, value: Int): Int {
        m_buffer.put(0, address.toByte())
        m_buffer.put(1, value.toByte())

        return I2CJNI.i2CWrite(m_port.toInt(), k_deviceAddress, m_buffer, 2.toByte())
    }

    private fun readShort(address: Int): Short {
        m_buffer.put(0, address.toByte())
        I2CJNI.i2CWrite(m_port.toInt(), k_deviceAddress, m_buffer, 1.toByte())
        I2CJNI.i2CRead(m_port.toInt(), k_deviceAddress, m_buffer, 2.toByte())
        return m_buffer.getShort(0)
    }

//    override fun setPIDSourceType(pidSource: PIDSourceType) {
//        if (pidSource != PIDSourceType.kDisplacement) {
//            throw IllegalArgumentException("Only displacement is supported")
//        }
//    }
//
//    override fun getPIDSourceType(): PIDSourceType {
//        return PIDSourceType.kDisplacement
//    }
//
//    override fun pidGet(): Double {
//        return getDistance().toDouble()
//    }
}
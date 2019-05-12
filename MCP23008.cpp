//
// The MCP23008 driver for the WireMaster system
// ---------------------------------------------------------------------------
// (c)2018 by Lucky Resistor. See LICENSE for details.
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along
// with this program; if not, write to the Free Software Foundation, Inc.,
// 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//
#include "MCP23008.hpp"


#include "../hal-common/GPIO.hpp"
#include "../hal-common/StatusTools.hpp"
#include "../hal-common/Timer.hpp"


namespace lr {


inline MCP23008::Status MCP23008::statusFromWireMaster(const WireMaster::Status status)
{
    return ((status==WireMaster::Status::Success)?(MCP23008::Status::Success):(MCP23008::Status::Error));
}


MCP23008::MCP23008(
    WireMaster *bus,
    Address address,
    GPIO::PinNumber resetPin,
    bool externalPullUp)
:
    _bus(bus, static_cast<uint8_t>(address)),
    _resetPin(resetPin),
    _noResetMode(externalPullUp?(GPIO::Mode::HighImpendance):(GPIO::Mode::High))
{
}


MCP23008::Status MCP23008::initialize()
{
    if (_resetPin != GPIO::cNoPin) {
        GPIO::setMode(_resetPin, _noResetMode);
    }
    return Status::Success;
}


MCP23008::Status MCP23008::test(bool testReset)
{
    // Check if the reset line is correctly configured.
    if (_resetPin != GPIO::cNoPin && _noResetMode == GPIO::Mode::HighImpendance) {
        GPIO::setMode(_resetPin, GPIO::Mode::Input);
        Timer::delayMilliseconds(1);
        // Check if the line is drawn high using the external pull up.
        if (GPIO::getState(_resetPin) != true) {
            return Status::Error;
        }
        GPIO::setMode(_resetPin, _noResetMode);
        Timer::delayMilliseconds(1);
    }
    // Now try to read from the chip.
    uint8_t data;
    if (hasError(_bus.readRegister(Register::IODIR, data))) {
        return Status::Error;
    }
    if (hasError(_bus.writeRegister(Register::IODIR, data))) {
        return Status::Error;
    }
    if (testReset && (_resetPin != GPIO::cNoPin)) {
        if (hasError(_bus.changeBits(Register::IOCON, ConfigurationMask(Configuration::SEQOP), WireMaster::BitOperation::Set))) {
            return Status::Error;
        }
        reset();
        WireMaster::BitResult bitResult;
        if (hasError(_bus.testBits(Register::IOCON, ConfigurationMask(Configuration::SEQOP), bitResult))) {
            return Status::Error;
        }
    }
    return Status::Success;
}


void MCP23008::reset()
{
    if (_resetPin != GPIO::cNoPin) {
        GPIO::setMode(_resetPin, GPIO::Mode::Low);
        Timer::delay(10_us);
        GPIO::setMode(_resetPin, _noResetMode);
        Timer::delay(10_us);
    }
}


MCP23008::Status MCP23008::setDirections(const PinMask pinMask, const Direction direction)
{
    return writeRegister(Register::IODIR, pinMask, direction==Direction::Input);
}


MCP23008::Status MCP23008::setInputDirections(const PinMask pinMask)
{
    const uint8_t value = static_cast<uint8_t>(pinMask);
    return statusFromWireMaster(_bus.writeRegister(Register::IODIR, value));
}


MCP23008::Status MCP23008::setInputPolarities(const PinMask pinMask, const InputPolarity inputPolarity)
{
    return writeRegister(Register::IPOL, pinMask, inputPolarity==InputPolarity::Inverted);
}


MCP23008::Status MCP23008::setAllInputPolarities(const PinMask pinMask)
{
    const uint8_t value = static_cast<uint8_t>(pinMask);
    return statusFromWireMaster(_bus.writeRegister(Register::IPOL, value));
}


MCP23008::Status MCP23008::setPullUps(const PinMask pinMask, const PullUp pullUp)
{
    return writeRegister(Register::GPPU, pinMask, pullUp==PullUp::Enabled);
}


MCP23008::Status MCP23008::setAllPullUps(const PinMask pinMask)
{
    const uint8_t value = static_cast<uint8_t>(pinMask);
    return statusFromWireMaster(_bus.writeRegister(Register::GPPU, value));
}


MCP23008::Status MCP23008::getInputs(PinMask &pinMask)
{
    uint8_t data;
    if (hasError(_bus.readRegister(Register::OLAT, data))) {
        return Status::Error;
    }
    pinMask = PinMask(data);
    return Status::Success;
}


MCP23008::Status MCP23008::setOutputs(const PinMask pinMask, const Output output)
{
    return writeRegister(Register::OLAT, pinMask, output==Output::High);
}


MCP23008::Status MCP23008::setOutputsHigh(const PinMask pinMask)
{
    return writeRegister(Register::OLAT, pinMask, true);
}


MCP23008::Status MCP23008::setOutputsLow(const PinMask pinMask)
{
    return writeRegister(Register::OLAT, pinMask, false);
}

    
MCP23008::Status MCP23008::setAllOutputs(const PinMask pinMask)
{
    return writeRegister(Register::OLAT, Pin::GP_All, static_cast<uint8_t>(pinMask));
}


MCP23008::Status MCP23008::flipOutputs(const PinMask pinMask)
{
    return statusFromWireMaster(_bus.changeBits(Register::OLAT, static_cast<uint8_t>(pinMask), WireMaster::BitOperation::Flip));
}


MCP23008::Status MCP23008::setInterruptOnChange(const PinMask pinMask)
{
    const uint8_t value = static_cast<uint8_t>(pinMask);
    return statusFromWireMaster(_bus.writeRegister(Register::GPINTEN, value));
}


MCP23008::Status MCP23008::setDefaultValueHigh(const PinMask pinMask)
{
    const uint8_t value = static_cast<uint8_t>(pinMask);
    return statusFromWireMaster(_bus.writeRegister(Register::DEFVAL, value));
}


MCP23008::Status MCP23008::setCompareWithDefaultValue(const PinMask pinMask)
{
    const uint8_t value = static_cast<uint8_t>(pinMask);
    return statusFromWireMaster(_bus.writeRegister(Register::INTCON, value));
}


MCP23008::Status MCP23008::setIntPinConfiguration(const IntPinConfig intPinConfig)
{
    ConfigurationMask config;
    switch (intPinConfig) {
    case IntPinConfig::ActiveLow:
        break;
    case IntPinConfig::ActiveHigh:
        config |= Configuration::INTPOL;
        break;
    case IntPinConfig::OpenDrain:
        config |= Configuration::ODR;
        break;
    }
    const uint8_t mask = Configuration::INTPOL|Configuration::ODR;
    return statusFromWireMaster(_bus.writeBits(Register::IOCON, mask, static_cast<uint8_t>(config)));
}


MCP23008::Status MCP23008::getInterruptFlags(PinMask &pinMask)
{
    uint8_t data;
    if (hasError(_bus.readRegister(Register::INTF, data))) {
        return Status::Error;
    }
    pinMask = PinMask(data);
    return Status::Success;
}


MCP23008::Status MCP23008::getInterruptCapturedInputs(PinMask &pinMask)
{
    uint8_t data;
    if (hasError(_bus.readRegister(Register::INTCAP, data))) {
        return Status::Error;
    }
    pinMask = PinMask(data);
    return Status::Success;
}


MCP23008::Status MCP23008::writeRegister(const Register reg, const PinMask pinMask, const bool enable)
{
    return writeRegister(reg, pinMask, static_cast<uint8_t>(enable?0xffu:0x00u));
}


MCP23008::Status MCP23008::writeRegister(const Register reg, const PinMask pinMask, const uint8_t value)
{
    // Note: All this tests are faster than sending a single bit over the I2C bus.
    if (pinMask == 0x00u) {
        return Status::Success;
    } else if (pinMask == 0xffu) {
        const uint8_t data = static_cast<uint8_t>(value);
        return statusFromWireMaster(_bus.writeRegister(reg, data));
    } else {
        const uint8_t mask = static_cast<uint8_t>(pinMask);
        return statusFromWireMaster(_bus.writeBits(reg, mask, value));
    }
}


}

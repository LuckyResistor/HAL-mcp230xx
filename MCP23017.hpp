#pragma once
//
// The MCP23017 driver for the WireMaster system
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


#include "../hal/Flags.hpp"
#include "../hal/GPIO.hpp"
#include "../hal/StatusTools.hpp"
#include "../hal/WireMasterChipRegister.hpp"

#include <cstdint>


namespace lr {


/// A driver for the MCP23017 16bit I/O expander.
///
/// This driver is using the WireMaster interface for the communication.
/// Optionally the pin for the reset line can be defined. The driver
/// assumes there is an external pull-up resistor on this line.
///
/// This driver assumes bank=0 and seqop=0 to access the registers.
///
/// All high level functions are not performant, because for simple
/// bit manipulations, the register is read before the new value
/// is written back. If you need high performance, use the low-level
/// functions and manipulate the registers directly.
///
/// All high level functions are written to access both ports of the
/// chip with one call. Use simple low-level calls using the `bus()`
/// to read/write only one port.
///
class MCP23017
{
public:
    /// The selected hardware address
    ///
    enum Address : uint8_t {
        Address0 = 0x20, ///< Hardware address 0 A2:L A1:L A0:L
        Address1, ///< Hardware address 0 A2:L A1:L A0:H
        Address2, ///< Hardware address 0 A2:L A1:H A0:L
        Address3, ///< Hardware address 0 A2:L A1:H A0:H
        Address4, ///< Hardware address 0 A2:H A1:L A0:L
        Address5, ///< Hardware address 0 A2:H A1:L A0:H
        Address6, ///< Hardware address 0 A2:H A1:H A0:L
        Address7, ///< Hardware address 0 A2:H A1:H A0:H
    };

    /// The configuration flags.
    ///
    enum class Configuration : uint8_t {
        INTPOL = (1u<<1), ///< The interrupt polarity (1=active-high).
        ODR = (1u<<2), ///< Interrupt is open-drain (1=open-drain).
        HAEN = (1u<<3), ///< Hardware address enable (1=enable).
        DISSLW = (1u<<4), ///< Slew rate control bit for SDA output (1=disable).
        SEQOP = (1u<<5), ///< Sequential operation mode bit (1=disable).
        MIRROR = (1u<<6), ///< Mirror interrupt pins (1=internally connected).
        BANK = (1u<<7), ///< The selected bank.
    };
    LR_DECLARE_FLAGS(Configuration, ConfigurationMask);

    /// The pin.
    ///
    enum Pin : uint16_t {
        GPA0 = (1u<<0), ///< Pin 0 on port A
        GPA1 = (1u<<1), ///< Pin 1 on port A
        GPA2 = (1u<<2), ///< Pin 2 on port A
        GPA3 = (1u<<3), ///< Pin 3 on port A
        GPA4 = (1u<<4), ///< Pin 4 on port A
        GPA5 = (1u<<5), ///< Pin 5 on port A
        GPA6 = (1u<<6), ///< Pin 6 on port A
        GPA7 = (1u<<7), ///< Pin 7 on port A
        GPB0 = (1u<<8), ///< Pin 0 on port B
        GPB1 = (1u<<9), ///< Pin 1 on port B
        GPB2 = (1u<<10), ///< Pin 2 on port B
        GPB3 = (1u<<11), ///< Pin 3 on port B
        GPB4 = (1u<<12), ///< Pin 4 on port B
        GPB5 = (1u<<13), ///< Pin 5 on port B
        GPB6 = (1u<<14), ///< Pin 6 on port B
        GPB7 = (1u<<15),  ///< Pin 7 on port B
        GP_All = 0xffff, ///< All pins on both ports
        GPA_All = 0x00ff, ///< All pins on port A
        GPB_All = 0xff00, ///< All pins on port B.
    };
    LR_DECLARE_FLAGS(Pin, PinMask);

    /// The pin direction.
    ///
    enum class Direction : uint8_t {
        Output, ///< Set the pin as output.
        Input ///< Set the pin as input.
    };
    
    /// The input polarity.
    ///
    enum class InputPolarity : uint8_t {
        Normal, ///< A high level will result in a one bit.
        Inverted ///< A low level will result in a one bit.
    };

    /// The pull up state.
    ///
    enum class PullUp : uint8_t {
        Disabled, ///< The pull up is disabled.
        Enabled, ///< The pull up is enabled.
    };
    
    /// The output state.
    ///
    enum class Output : uint8_t {
        Low, ///< The output is low (GND).
        High ///< The output is high (VCC).
    };

    /// Interrupt pin configuration.
    ///
    enum class IntPinConfig : uint8_t {
        ActiveLow, ///< The interrupt pin is pulled low on interrupt.
        ActiveHigh, ///< The interrupt pin is pulled high on interrupt.
        OpenDrain ///< The interrupt pin is open drain and pulled low on interrupt.
    };

    /// The status of function calls
    ///
    using Status = CallStatus;

public:
    /// Create a new instance of the driver.
    ///
    /// @param[in] bus The bus to use to communicate with the chip.
    /// @param address The hardware address of the chip.
    /// @param resetPin The pin number which is connected to reset.
    /// @param externalPullUp `true` if an external pull-up is connected,
    ///          `false` if the pin should be set into high state.
    ///
    MCP23017(
        WireMaster *bus,
        Address address,
        GPIO::PinNumber resetPin = GPIO::cNoPin,
        bool externalPullUp = true);

    /// Initialize the chip.
    ///
    /// If a reset pin is defined, this method makes sure the
    /// reset pin is set to floating.
    ///
    /// @return Always `Success`.
    ///
    Status initialize();
    
    /// Test the chip.
    ///
    /// This test will write and read a register to see if the chip.
    /// reacts to communication. If `testReset` is set to `true`, a
    /// register is set to a non default value, a hardware reset is
    /// performed and the register is read again. It verifies if the
    /// register value was successfully reset to the default value.
    ///
    /// @param[in] testReset Flag if the reset line shall be tested as well.
    ///   This will reset the chip.
    /// @return `Success` if the chip is accessible and reacts to
    ///   register reads and writes.
    ///
    Status test(bool testReset = false);
    
    /// Reset the chip.
    ///
    /// If a reset pin is specified, this method will perform a
    /// hardware reset of the chip.
    ///
    void reset();
    
    /// Set the pin direction.
    ///
    /// Usage: `io.setDirection(MCP23017::GPA0|MCP23017::GPB6, MCP23017::Direction::Output);`
    ///
    /// @param[in] pinMask The combination of pins.
    /// @param[in] direction The direction to set for the given pins.
    /// @return `Success` if the direction was set successfully.
    ///
    Status setDirections(const PinMask pinMask, const Direction direction);

    /// Set the pin direction for all pins.
    ///
    /// This function will set all pins in the mask as inputs
    /// and all other pins as outputs.
    ///
    /// @param[in] pinMask The pins to set as inputs.
    /// @return `Success` if the polarity was set successfully.
    ///
    Status setInputDirections(const PinMask pinMask);

    /// Set the input polarity.
    ///
    /// @param[in] pinMask The combination of pins.
    /// @param[in] inputPolarity The input polarity for the given pins.
    /// @return `Success` if the polarity was set successfully.
    ///
    Status setInputPolarities(const PinMask pinMask, const InputPolarity inputPolarity);

    /// Set the input polarity for all pins.
    ///
    /// This function will set revresed input polarity for all
    /// pins in the mask normal polarity for all others.
    ///
    /// @param[in] pinMask The pins to set for reverse input polarity.
    /// @return `Success` if the polarity was set successfully.
    ///
    Status setAllInputPolarities(const PinMask pinMask);

    /// Enable/Disable pull-up resistors.
    ///
    /// Enable/Disable the 100k pull-up resistors in the chip.
    ///
    /// @param[in] pinMask The combination of pins.
    /// @param[in] pullUp If the pull-ups shall be enabled.
    /// @return `Success` if the polarity was set successfully.
    ///
    Status setPullUps(const PinMask pinMask, const PullUp pullUp);

    /// Set the pull up state for all pins.
    ///
    /// This function will enable the pull-up resistor for all
    /// pins in the mask and disable it for all others.
    ///
    /// @param[in] pinMask The pins to enable the pull-up resistors.
    /// @return `Success` if the polarity was set successfully.
    ///
    Status setAllPullUps(const PinMask pinMask);

    /// Read the inputs from both ports.
    ///
    /// @param[out] pinMask The pin mask read from the registers.
    /// @return `Success` if the polarity was set successfully.
    ///
    Status getInputs(PinMask &pinMask);

    /// Set outputs.
    ///
    /// @param[in] pinMask The combination of pins.
    /// @param[in] output The state to set for the selected pins.
    /// @return `Success` if the polarity was set successfully.
    ///
    Status setOutputs(const PinMask pinMask, const Output output);

    /// Set outputs into high state.
    ///
    /// @param[in] pinMask The pins to set into the high state.
    /// @return `Success` if the polarity was set successfully.
    ///
    Status setOutputsHigh(const PinMask pinMask);

    /// Set outputs into low state.
    ///
    /// @param[in] pinMask The pins to set into the low state.
    /// @return `Success` if the polarity was set successfully.
    ///
    Status setOutputsLow(const PinMask pinMask);

    /// Flip the state of the selected outputs.
    ///
    /// @param[in] pinMask The pins to flip the state.
    /// @return `Success` if the polarity was set successfully.
    ///
    Status flipOutputs(const PinMask pinMask);

    /// Enable interrupt on change for the selected pins.
    ///
    /// This will enable interrupt on change for all pins in
    /// the mask and disable it for all other pins.
    ///
    /// @param[in] pinMask The pins to enable interrupt on change.
    /// @return `Success` if the change was successful.
    ///
    Status setInterruptOnChange(const PinMask pinMask);

    /// Set the default value register for comparision to generate an interrupt.
    ///
    /// This will set the default value to high/one for all
    /// pins in the mask, and to low/zero for all other pins.
    ///
    /// @param[in] pinMask The pins to set a high/one default value.
    /// @return `Success` if the change was successful.
    ///
    Status setDefaultValueHigh(const PinMask pinMask);
    
    /// Set pins to compare against the default value to generate an interrupt.
    ///
    /// This will set all pins in the mask to be compared
    /// against the default value and all other pins compared
    /// against the previous value (interrupt on change).
    ///
    /// @param[in] pinMask The pins to set compared against the default value.
    /// @return `Success` if the change was successful.
    ///
    Status setCompareWithDefaultValue(const PinMask pinMask);
    
    /// Set the interrupt pin configuration.
    ///
    /// @param[in] intPinConfig The configuration for the interrupt pins.
    /// @param[in] mirror Flag if the two interrupt pins are connected.
    ///    `true` if the two pins are connected, `false` if each port has
    ///    its separate interrupt pin.
    /// @return `Success` if the change was successful.
    ///
    Status setIntPinConfiguration(const IntPinConfig intPinConfig, bool mirror);

    /// Get the current interrupt status for the pins.
    ///
    /// @param[out] pinMask The pins where the interrupt flag is set.
    /// @return `Success` if the polarity was set successfully.
    ///
    Status getInterruptFlags(PinMask &pinMask);

    /// Get the status of the inputs at the time of an interrupt.
    ///
    /// @param[out] pinMask The pins captured at the time of an interrupt.
    /// @return `Success` if the polarity was set successfully.
    ///
    Status getInterruptCapturedInputs(PinMask &pinMask);

public:
    /// @name Low Level Functions
    /// Low level functions to directly access all registers.
    /// @{

    /// All registers available in the chip.
    ///
    enum class Register : uint8_t {
        PortA       = 0x00, ///< The increment for port A
        PortB       = 0x01, ///< The increment for port B
        IODIR       = 0x00, ///< IO direction.
        IPOL        = 0x02, ///< Input polarity.
        GPINTEN     = 0x04, ///< Interrupt on change pins.
        DEFVAL      = 0x06, ///< Default values.
        INTCON      = 0x08, ///< Interrupt control.
        IOCON       = 0x0A, ///< Configuration.
        GPPU        = 0x0C, ///< Pull-ups
        INTF        = 0x0E, ///< Interrupt flags.
        INTCAP      = 0x10, ///< Interrupt captured.
        GPIO        = 0x12, ///< Port IO
        OLAT        = 0x14, ///< Output latch.
        IODIRA      = PortA+IODIR,
        IODIRB      = PortB+IODIR,
        IPOLA       = PortA+IPOL,
        IPOLB       = PortB+IPOL,
        GPINTENA    = PortA+GPINTEN,
        GPINTENB    = PortB+GPINTEN,
        DEFVALA     = PortA+DEFVAL,
        DEFVALB     = PortB+DEFVAL,
        INTCONA     = PortA+INTCON,
        INTCONB     = PortB+INTCON,
        IOCONA      = PortA+IOCON,
        IOCONB      = PortB+IOCON,
        GPPUA       = PortA+GPPU,
        GPPUB       = PortB+GPPU,
        INTFA       = PortA+INTF,
        INTFB       = PortB+INTF,
        INTCAPA     = PortA+INTCAP,
        INTCAPB     = PortB+INTCAP,
        GPIOA       = PortA+GPIO,
        GPIOB       = PortB+GPIO,
        OLATA       = PortA+OLAT,
        OLATB       = PortB+OLAT,
    };

    /// The number of registers in the chip.
    ///
    constexpr inline static uint8_t getRegisterCount() {
        return 0x13;
    };

    /// Directly access the low-level bus functions
    ///
    inline const WireMasterRegisterChip<Register>& bus() const {
        return _bus;
    }

    /// Set/clear flags in a register..
    ///
    /// This helper function slightly optimises writing the bits to the registers.
    ///
    Status writeRegister(const Register reg, const PinMask pinMask, const bool enable);

    /// Write a register.
    ///
    /// This helper function slightly optimises writing the bits
    /// to the registers.
    ///
    Status writeRegister(const Register reg, const PinMask pinMask, const uint16_t value);

    /// @}

private:
    /// Convert the status from the WireMaster interface.
    ///
    inline static Status statusFromWireMaster(const WireMaster::Status status);
    
private:
    const WireMasterRegisterChip<Register> _bus; ///< The bus for the communication.
    const GPIO::PinNumber _resetPin; ///< The reset pin to reset the chip.
    const GPIO::Mode _noResetMode; ///< The regular pin mode without reset.
};


LR_DECLARE_OPERATORS_FOR_FLAGS(MCP23017::ConfigurationMask);
LR_DECLARE_OPERATORS_FOR_FLAGS(MCP23017::PinMask);


}


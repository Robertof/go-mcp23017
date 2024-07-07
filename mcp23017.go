/*
MCP23017 I2C Library based on Adafruit Arduino Driver
https://github.com/adafruit/Adafruit-MCP23017-Arduino-Library/blob/master/Adafruit_MCP23017.cpp
*/
package mcp23017

import (
	"errors"
	"fmt"
	"sync"

	"github.com/racerxdl/go-mcp23017/i2c"
)

var (
	ErrNoInterrupt = errors.New("no interrupt has triggered")
)

const (
	_Address  = 0x20
	_IODIRA   = 0x00
	_IODIRB   = 0x01
	_IPOLA    = 0x02
	_IPOLB    = 0x03
	_GPINTENA = 0x04
	_GPINTENB = 0x05
	_DEFVALA  = 0x06
	_DEFVALB  = 0x07
	_INTCONA  = 0x08
	_INTCONB  = 0x09
	_IOCONA   = 0x0A
	_IOCONB   = 0x0B
	_GPPUA    = 0x0C
	_GPPUB    = 0x0D
	_INTFA    = 0x0E
	_INTFB    = 0x0F
	_INTCAPA  = 0x10
	_INTCAPB  = 0x11
	_GPIOA    = 0x12
	_GPIOB    = 0x13
	_OLATA    = 0x14
	_OLATB    = 0x15
	INTERR    = 0xFF
)

type PinMode uint8
type PinLevel bool
type DevicePort uint8
type PinMask uint16

const (
	INPUT  PinMode = 0
	OUTPUT         = 1
)

const (
	PORTA DevicePort = 0
	PORTB            = 1
)

const (
	LOW  PinLevel = false
	HIGH          = true
)

var cachedRegs = []uint8{
	_IODIRA,
	_IODIRB,
	_GPIOA,
	_GPIOB,
	_INTCONA,
	_INTCONB,
}

var defaultValues = map[uint8]uint8{
	_IODIRA:   0xFF,
	_IPOLA:    0x00,
	_GPINTENA: 0x00,
	_DEFVALA:  0x00,
	_INTCONA:  0x00,
	_IOCONA:   0x00,
	_GPPUA:    0x00,
	_INTFA:    0x00,
	_INTCAPA:  0x00,
	_GPIOA:    0x00,
	_OLATA:    0x00,
	_IODIRB:   0xFF,
	_IPOLB:    0x00,
	_GPINTENB: 0x00,
	_DEFVALB:  0x00,
	_INTCONB:  0x00,
	_IOCONB:   0x00,
	_GPPUB:    0x00,
	_INTFB:    0x00,
	_INTCAPB:  0x00,
	_GPIOB:    0x00,
	_OLATB:    0x00,
}

// Same i2c bus can be called from several instances
var busLocks map[int]*sync.Mutex

func init() {
	busLocks = make(map[int]*sync.Mutex)
}

func regIsCacheable(reg uint8) bool {
	for _, v := range cachedRegs {
		if v == reg {
			return true
		}
	}

	return false
}

// SetDefaultPinMode sets the pinMode on Reset
func SetDefaultPinMode(mode PinMode) {
	if mode == INPUT {
		defaultValues[_IODIRA] = 0xFF
		defaultValues[_IODIRB] = 0xFF
	} else {
		defaultValues[_IODIRA] = 0x00
		defaultValues[_IODIRB] = 0x00
	}
}

// SetDefaultValues sets the default value on reset
func SetDefaultValues(portA, portB uint8) {
	defaultValues[_GPIOA] = portA
	defaultValues[_GPIOB] = portB
}

// HasPin returns whether this mask contains the given pin.
func (pm PinMask) HasPin(pin uint8) bool {
	return pm&(1<<pin) == (1 << pin)
}

// Pins returns all pins set in the mask.
func (pm PinMask) Pins() (pins []uint8) {
	for p := uint8(0); p < 16; p += 1 {
		if pm.HasPin(p) {
			pins = append(pins, p)
		}
	}

	return pins
}

// reg returns which ports should be read based on the bits set in the mask.
// If a register should NOT be read, the corresponding return value is 0.
func (p PinMask) reg(portA, portB uint8) (a, b uint8) {
	left := p & 0xFF
	right := (p >> 8) & 0xFF

	if left != 0 {
		a = portA
	}

	if right != 0 {
		b = portB
	}

	return a, b
}

type Device struct {
	dev           *i2c.I2C
	bus           int
	cachedRegVals map[uint8]uint8
}

// Open opens a MCP23017 device and returns a handler.
// Returns error in case devNum > 8 or if device is not present
func Open(bus, devNum uint8) (*Device, error) {
	if devNum > 8 {
		return nil, fmt.Errorf("only 8 devices are supported on a single I2C bus")
	}

	dev, err := i2c.NewI2C(_Address+devNum, int(bus))

	if err != nil {
		return nil, err
	}

	if _, ok := busLocks[int(bus)]; !ok {
		busLocks[int(bus)] = &sync.Mutex{}
	}

	d := &Device{
		dev:           dev,
		bus:           int(bus),
		cachedRegVals: map[uint8]uint8{},
	}

	for k, v := range defaultValues {
		d.cachedRegVals[k] = v
	}

	err = d.Reset()

	if err != nil {
		_ = dev.Close()
		return nil, err
	}

	return d, nil
}

// Rewrites all registers from cached values
func (d *Device) Rewrite() error {
	busLocks[d.bus].Lock()
	defer busLocks[d.bus].Unlock()

	for k, v := range d.cachedRegVals {
		if regIsCacheable(k) {
			err := d.writeReg(k, v)
			if err != nil {
				return fmt.Errorf("error writing register 0x%02x: %s", k, err)
			}
		}
	}

	return nil
}

// Reset resets the register to default values
func (d *Device) Reset() error {
	busLocks[d.bus].Lock()
	defer busLocks[d.bus].Unlock()
	for k, v := range defaultValues {
		err := d.writeReg(k, v)
		if err != nil {
			return fmt.Errorf("error setting defaults to device: %s", err)
		}
	}

	return nil
}

// PinMode sets the specified pin (0-15 range) mode
func (d *Device) PinMode(pin uint8, mode PinMode) error {
	v := uint8(0)
	if mode == INPUT {
		v = 1
	}
	return d.updateRegisterBit(pin, v, _IODIRA, _IODIRB)
}

// IsPresent performs a read in GPIOA to check if the device is alive.
func (d *Device) IsPresent() bool {
	_, err := d.ReadGPIO(0)
	if err != nil {
		return false
	}

	return true
}

// DigitalWrite sets pin (0-15 range) to specified level
func (d *Device) DigitalWrite(pin uint8, level PinLevel) error {
	busLocks[d.bus].Lock()
	defer busLocks[d.bus].Unlock()

	v := uint8(0)
	if level == HIGH {
		v = 1
	}

	bit := bitForPin(pin)
	addr := regForPin(pin, _GPIOA, _GPIOB)

	// Read Current State cached
	gpio := d.readCacheReg(addr)

	// Set bit
	gpio = bitWrite(gpio, bit, v)

	// Write GPIO
	addr = regForPin(pin, _GPIOA, _GPIOB)

	return d.writeReg(addr, gpio)
}

// DigitalRead returns the level of the specified pin (0-15 range)
func (d *Device) DigitalRead(pin uint8) (PinLevel, error) {
	busLocks[d.bus].Lock()
	defer busLocks[d.bus].Unlock()

	bit := bitForPin(pin)
	addr := regForPin(pin, _GPIOA, _GPIOB)

	v, err := d.dev.ReadRegU8(addr)

	if err != nil {
		return LOW, err
	}

	if (v>>bit)&0x1 > 0 {
		return HIGH, nil
	}

	return LOW, nil
}

// SetPullUp enables/disables pull up in specified pin (0-15 range)
func (d *Device) SetPullUp(pin uint8, enabled bool) error {
	v := uint8(0)
	if enabled {
		v = 1
	}

	return d.updateRegisterBit(pin, v, _GPPUA, _GPPUB)
}

// SetInverte enables/disables polarity inversion for the specified pin (0-15 range)
func (d *Device) SetInvert(pin uint8, enabled bool) error {
	v := uint8(0)
	if enabled {
		v = 1
	}

	return d.updateRegisterBit(pin, v, _IPOLA, _IPOLB)
}

// WriteGPIOAB sets GPIO AB to specified value
func (d *Device) WriteGPIOAB(value uint16) error {
	busLocks[d.bus].Lock()
	defer busLocks[d.bus].Unlock()

	d.cachedRegVals[_GPIOA] = uint8(value & 0xFF)
	d.cachedRegVals[_GPIOB] = uint8(value >> 8 & 0xFF)

	return d.dev.WriteRegU16BE(_GPIOA, value)
}

// ReadGPIOAB reads both PORT A and B and returns a 16 bit value containing AB
func (d *Device) ReadGPIOAB() (uint16, error) {
	busLocks[d.bus].Lock()
	defer busLocks[d.bus].Unlock()

	return d.dev.ReadRegU16BE(_GPIOA)
}

// ReadGPIO reads the specified port and returns it's value
func (d *Device) ReadGPIO(n DevicePort) (uint8, error) {
	busLocks[d.bus].Lock()
	defer busLocks[d.bus].Unlock()

	p := uint8(_GPIOA)
	if n == PORTB {
		p = _GPIOB
	}

	return d.dev.ReadRegU8(p)
}

// SetupInterrupts configures the device interrupts for both ports A and B.
// mirroring will OR both INTA / INTB pins
// openDrain will set INT pin to specified value or Open Drain
// polarity will set LOW or HIGH on interrupt
// Default values after reset are: false, false, LOW)
func (d *Device) SetupInterrupts(mirroring, openDrain bool, polarity PinLevel) error {
	busLocks[d.bus].Lock()
	defer busLocks[d.bus].Unlock()

	p := uint8(0)
	if polarity == HIGH {
		p = 1
	}

	m := uint8(0)
	if mirroring {
		m = 1
	}

	od := uint8(0)
	if openDrain {
		od = 1
	}

	// Configure Port A
	r, err := d.dev.ReadRegU8(_IOCONA)
	if err != nil {
		return err
	}

	r = bitWrite(r, 6, m)
	r = bitWrite(r, 2, od)
	r = bitWrite(r, 1, p)

	err = d.writeReg(_IOCONA, r)
	if err != nil {
		return err
	}
	d.cachedRegVals[_IOCONA] = r

	// Configure Port B
	r, err = d.dev.ReadRegU8(_IOCONB)
	if err != nil {
		return err
	}

	r = bitWrite(r, 6, m)
	r = bitWrite(r, 2, od)
	r = bitWrite(r, 1, p)

	err = d.writeReg(_IOCONB, r)
	if err != nil {
		return err
	}

	return nil
}

// EnableInterrupts enables or disables interrupts for a pin.
func (d *Device) EnableInterrupts(pin uint8, enabled bool) error {
	v := uint8(0)
	if enabled {
		v = 1
	}

	return d.updateRegisterBit(pin, v, _GPINTENA, _GPINTENB)
}

// SetInterrupts sets GPINTENA and GPINTENB to the specified value.
func (d *Device) SetEnabledInterrupts(value uint16) error {
	busLocks[d.bus].Lock()
	defer busLocks[d.bus].Unlock()

	return d.dev.WriteRegU16LE(_GPINTENA, value)
}

// GetInterruptPins returns the pins that triggered a interrupt.
// If no interrupt has been triggered, this returns `ErrNoInterrupt`.
func (d *Device) GetInterruptPins() (out PinMask, err error) {
	busLocks[d.bus].Lock()
	defer busLocks[d.bus].Unlock()

	mask, err := d.dev.ReadRegU16LE(_INTFA)

	if err != nil {
		return 0, err
	}

	if mask == 0 {
		return 0, ErrNoInterrupt
	}

	return PinMask(mask), nil
}

// GetInterruptPinValues returns the values of pins that triggered an interrupt as a mask.
// If pins 1 and 10 triggered an interrupt and were HIGH, this returns 10000000010.
// If no interrupt has been triggered, this returns `ErrNoInterrupt`.
//
// Invoking this resets the interrupts and allows them to fire again.
func (d *Device) GetInterruptPinValues() (PinMask, error) {
	pins, err := d.GetInterruptPins()

	if err != nil {
		return 0, err
	}

	busLocks[d.bus].Lock()
	defer busLocks[d.bus].Unlock()

	// determine which ports we need to query
	regA, regB := pins.reg(_INTCAPA, _INTCAPB)

	// if we need to read both registers, just do a single u16 read and unpack the result.
	if regA != 0 && regB != 0 {
		levels, err := d.dev.ReadRegU16LE(regA)

		if err != nil {
			return 0, err
		}

		return pins & PinMask(levels), nil
	}

	reg, shift := regA, uint8(0)

	if regB != 0 {
		reg, shift = regB, 8
	}

	levels, err := d.dev.ReadRegU8(reg)

	if err != nil {
		return 0, err
	}

	return PinMask(uint16(levels)<<shift) & pins, nil
}

// Close Closes the device
func (d *Device) Close() error {
	return d.dev.Close()
}

// region Private
func (d *Device) updateRegisterBit(pin, value, portA, portB uint8) error {
	busLocks[d.bus].Lock()
	defer busLocks[d.bus].Unlock()
	addr := regForPin(pin, portA, portB)
	bit := bitForPin(pin)

	regVal, err := d.dev.ReadRegU8(addr)
	if err != nil {
		return err
	}

	regVal = bitWrite(regVal, bit, value)

	return d.writeReg(addr, regVal)
}

func (d *Device) writeReg(addr, val uint8) error {
	d.cachedRegVals[addr] = val
	return d.dev.WriteRegU8(addr, val)
}

func (d *Device) readCacheReg(addr uint8) uint8 {
	return d.cachedRegVals[addr]
}

// endregion

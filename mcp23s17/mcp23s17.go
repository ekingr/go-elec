package mcp23s17

// MCP23S17 control module for raspberry pi over SPI
// https://ww1.microchip.com/downloads/en/devicedoc/20001952c.pdf
// Check that SPI interface is enabled on the pi (/boot/config.txt)
// https://pkg.go.dev/periph.io/x/conn/v3@v3.6.9/spi

// Pinout
// 09 VDD = 3.3V (pi17)
// 10 VSS = GND (pi20)
// 11 ^CS = SPI0.CE0 (pi24=gpio8)
// 12 SCK = SPI0.SCLK (pi23=gpio11)
// 13 SI = SPI0.MOSI (pi19=gpio10)
// 14 SO = SPI0.MISO (pi21=gpio9)
// 15 A0 = GND
// 16 A1 = GND
// 17 A2 = GND
// 18 ^RESET = 3.3V
// 20 INTA = pi input pin (pi18=gpio24)

import (
	"errors"
	"sync"
	"time"

	"periph.io/x/conn/v3/gpio"
	"periph.io/x/conn/v3/physic"
	"periph.io/x/conn/v3/spi"
	"periph.io/x/conn/v3/spi/spireg"
	"periph.io/x/host/v3"
	"periph.io/x/host/v3/rpi"
)

type GpioPort int

const (
	// SPI connection parameters
	spiAddr string           = "/dev/spidev0.0"
	spiMode spi.Mode         = spi.Mode0
	spiBpw  int              = 8
	spiFreq physic.Frequency = 5 * physic.MegaHertz
	address byte             = 0x40 //  0| 1| 0| 0|A2|A1|A0|RW

	// MCP23S17 GPIO ports
	GPA0 GpioPort = 0
	GPA1 GpioPort = 1
	GPA2 GpioPort = 2
	GPA3 GpioPort = 3
	GPA4 GpioPort = 4
	GPA5 GpioPort = 5
	GPA6 GpioPort = 6
	GPA7 GpioPort = 7
	GPB0 GpioPort = 8
	GPB1 GpioPort = 9
	GPB2 GpioPort = 10
	GPB3 GpioPort = 11
	GPB4 GpioPort = 12
	GPB5 GpioPort = 13
	GPB6 GpioPort = 14
	GPB7 GpioPort = 15
)

type Mcp23s17 struct {
	mu           sync.Mutex
	spiPort      spi.PortCloser
	cnx          spi.Conn
	direction    [2]byte
	value        [2]byte
	defaultValue [2]byte
	intPin       gpio.PinIn
	ended        bool
}

func NewMcp23s17(direction, defaultValue [2]byte, intCallback func([2]byte, error)) (mcp *Mcp23s17, err error) {
	mcp = &Mcp23s17{}

	// Pi setup
	if _, err = host.Init(); err != nil {
		return
	}

	// Pi setup: SPI
	mcp.spiPort, err = spireg.Open(spiAddr)
	if err != nil {
		return
	}
	mcp.cnx, err = mcp.spiPort.Connect(spiFreq, spiMode, spiBpw)
	if err != nil {
		return
	}
	if _, ok := mcp.cnx.(spi.Pins); ok {
		//fmt.Printf(" CLK: %s\n", px.CLK())
		//fmt.Printf("MOSI: %s\n", px.MOSI())
		//fmt.Printf("MISO: %s\n", px.MISO())
		//fmt.Printf("  CS: %s\n", px.CS())
	} else {
		err = errors.New("Unable to locate host SPI pins")
		return
	}

	// Pi setup: GPIO
	mcp.intPin = rpi.P1_18 // Interrupt pin (pi18=GPIO24)
	// Previously gpio.PullUp
	if err = mcp.intPin.In(gpio.Float, gpio.RisingEdge); err != nil {
		return
	}

	// MCP23S17 setup
	// IOCON.BANK=0 by default (cf. Table3-5 POR/RST value)
	// But to make sure (otherwise everything is messed-up), writing 'config' to 0x05 [IOCON w/IOCON.BANK=1]
	// Anyways, 0x05 [GPINTENB w/IOCON.BANK=0] is not critical and is overriden afterwards
	mcp.Write([]byte{0x05, 0x00}) // 0x05 IOCON w/IOCON.BANK=1: Resetting configuration to IOCON.BANK=0
	// Now we are sure IOCON.BANK=0
	mcp.Write([]byte{0x0a, 0x42}) // 0x0A IOCON: Resetting configuration:
	//   BANK=0 for sequential access (default)
	//   MIRROR=1 to use a signle interrupt pin
	//   SEQOP=0 to use sequential addresses access (default)
	//   DISSLW=0 to have slew rate enabled (default)
	//   HAEN=0 to disable address pins (default)
	//   ODR=0 to use INTPOL to set INT pins polarity
	//   INTPOL=1 to have interrupts pins as active-high
	//   Unimplemented
	// Since IOCON.SEQOP=0, sequential operation in enabled
	// and with IOCON.BANK=0, addresses are sequential
	// then commands can be consecutive for both A & B register banks
	mcp.direction = direction
	mcp.Write(append([]byte{0x00}, mcp.direction[:]...)) // 0x00 IODIR: Setting pins input/output direction
	mcp.Write([]byte{0x02,
		0x00, 0x00, // 0x02 IPOL: Setting all pins as non-inverted
		0xff, 0xff, // 0x04 GPINTEN: Enabling interrupt on all pins
		0x00, 0x00, // 0x06 DEFVAL: Setting interrupt comparison value to all pins for good measure
		0x00, 0x00, // 0x08 INTCON: Setting all pins to interrupt-on-change (vs previous value)
	})
	mcp.Write([]byte{0x0c, 0xff, 0xff}) // 0x0c GPPU: Setting all pins with 100kR pull-up
	mcp.Write([]byte{0x14, 0x00, 0x00}) // 0x14 OLAT: Disabling output latch for all pins
	mcp.value = defaultValue
	mcp.Write(append([]byte{0x12}, mcp.value[:]...)) // 0x12 GPIO: Setting all pins to default value
	mcp.defaultValue = defaultValue                  // Saving default value to reset before closing

	// Interrupt pin setup: interrupt callback
	mcp.ended = false
	go func() {
		for !mcp.ended {
			if mcp.intPin.WaitForEdge(1 * time.Second) {
				val, err := mcp.GetAll()
				intCallback(val, err)
			}
		}
	}()
	return
}

func (mcp *Mcp23s17) Close() error {
	mcp.ended = true
	mcp.SetAll(mcp.defaultValue)
	err1 := mcp.intPin.Halt()
	err2 := mcp.spiPort.Close()
	if err1 != nil {
		return err1
	}
	return err2
}

func (mcp *Mcp23s17) readwrite(w []byte) ([]byte, error) {
	if len(w) < 3 {
		return nil, errors.New("Write slice lenght must be >= 3 (control, register, value1, [value2, ...]")
	}
	r := make([]byte, len(w))
	mcp.mu.Lock()
	defer mcp.mu.Unlock()
	if err := mcp.cnx.Tx(w, r); err != nil {
		return nil, err
	}
	//fmt.Println("w->", w)
	//fmt.Println("r<-", r)
	return r, nil
}

func (mcp *Mcp23s17) Read(w []byte) ([]byte, error) {
	if len(w) < 2 {
		return nil, errors.New("Write slice lenght must be >= 2 (register, value1, [value2, ...]")
	}
	ctrl := address | 0x01
	ww := append([]byte{ctrl}, w...)
	r, err := mcp.readwrite(ww)
	if err != nil {
		return nil, err
	}
	return r[2:], nil
}

func (mcp *Mcp23s17) Write(w []byte) error {
	if len(w) < 2 {
		return errors.New("Write slice lenght must be >= 2 (register, value1, [value2, ...]")
	}
	ctrl := address | 0x00
	ww := append([]byte{ctrl}, w...)
	_, err := mcp.readwrite(ww)
	return err
}

func (mcp *Mcp23s17) GetAll() ([2]byte, error) {
	val, err := mcp.Read([]byte{0x12, 0x00, 0x00})
	return *(*[2]byte)(val), err
}

func (mcp *Mcp23s17) SetAll(val [2]byte) error {
	w := append([]byte{0x12}, val[:]...)
	if err := mcp.Write(w); err != nil {
		return err
	}
	mcp.value = val
	return nil
}

func (mcp *Mcp23s17) Set(p GpioPort, v int) error {
	if p < 0 || p > 15 {
		return errors.New("Unknown GPIO port. Refer to the package constants")
	}
	if v < 0 || v > 1 {
		return errors.New("Value must be 0 or 1")
	}
	AB := (p - (p % 8)) / 8       // Index within the value array (0 or 1)
	exp := v << (p % 8)           // Value shifted to the right exponent
	mask := 0xff ^ (1 << (p % 8)) // Masking only the exponent we are modifying
	if mcp.direction[AB]&byte(0xff^mask) > 0 {
		// Direction bit is set for given port
		// Writing to it would be transparent,
		// but better warn the user
		return errors.New("Cannot write: given port is an output")
	}
	val := mcp.value
	val[AB] = val[AB]&byte(mask) | byte(exp)
	return mcp.SetAll(val)
}

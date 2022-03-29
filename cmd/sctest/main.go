//-----------------------------------------------------------------------------
/*

Servo Controller Test

*/
//-----------------------------------------------------------------------------

package main

import (
	"log"
	"time"

	"github.com/deadsy/maestro/sc"
	"github.com/tarm/serial"
)

//-----------------------------------------------------------------------------

func sctest() error {

	serialConfig := &serial.Config{
		Name:        "/dev/ttyACM0",
		Baud:        115200,
		ReadTimeout: time.Millisecond * 500,
	}

	port, err := serial.OpenPort(serialConfig)
	if err != nil {
		return err
	}
	defer port.Close()

	scConfig := &sc.Config{
		Port:         port,
		DeviceNumber: 12,
		Compact:      false,
		Crc:          true,
	}

	ctrl, err := sc.NewController(scConfig)
	if err != nil {
		return err
	}

	// get/clear any initial error code
	code, err := ctrl.GetErrors()
	if err != nil {
		return err
	}
	err = sc.GetError(code)
	if err != nil {
		log.Printf("controller error: %s", err)
	}

	s0 := ctrl.NewServo(0)
	s0.SetSpeed(0)
	s0.SetAcceleration(0)

	s0.SetTarget(500 * 4)
	time.Sleep(2 * time.Second)

	s0.SetTarget(2500 * 4)
	time.Sleep(2 * time.Second)

	return nil
}

func main() {
	err := sctest()
	if err != nil {
		log.Fatalf("error: %s", err)
	}
}

//-----------------------------------------------------------------------------

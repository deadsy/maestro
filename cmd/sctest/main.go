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

	scConfig := &sc.Config{
		Port:         port,
		Name:         "servo controller",
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

	s0 := sc.NewServo(ctrl, 0)

	err = s0.SetTarget(500 * 4)
	if err != nil {
		return err
	}
	time.Sleep(2 * time.Second)

	err = s0.SetTarget(2500 * 4)
	if err != nil {
		return err
	}
	time.Sleep(2 * time.Second)

	err = ctrl.Close()
	if err != nil {
		return err
	}

	return nil
}

func main() {
	err := sctest()
	if err != nil {
		log.Fatalf("error: %s", err)
	}
}

//-----------------------------------------------------------------------------

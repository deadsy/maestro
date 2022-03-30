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

	s0, _ := ctrl.NewServo(0)
	s0.SetSpeed(0)
	s0.SetAcceleration(0)

	s1, _ := ctrl.NewServo(1)
	s1.SetSpeed(0)
	s1.SetAcceleration(0)

	s2, _ := ctrl.NewServo(2)
	s2.SetSpeed(0)
	s2.SetAcceleration(0)

	ctrl.SetTargets(0, []uint16{2000, 8000, 2000})
	time.Sleep(2 * time.Second)

	ctrl.SetTargets(0, []uint16{8000, 2000, 8000})
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

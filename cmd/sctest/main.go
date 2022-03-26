//-----------------------------------------------------------------------------
/*

Servo Controller Test

*/
//-----------------------------------------------------------------------------

package main

import (
	"log"

	"github.com/deadsy/maestro/sc"
	"github.com/tarm/serial"
)

//-----------------------------------------------------------------------------

func sctest() error {

	cfg := &serial.Config{
		Name: "/dev/ttyACM0",
		Baud: 115200,
	}

	port, err := serial.OpenPort(cfg)
	if err != nil {
		return err
	}

	ctrl := sc.NewController(port, 0, false)
	s0 := sc.NewServo(ctrl, 0)
	s1 := sc.NewServo(ctrl, 1)
	s2 := sc.NewServo(ctrl, 2)

	err = s0.SetTarget(0)
	if err != nil {
		return err
	}

	err = s1.SetTarget(0)
	if err != nil {
		return err
	}

	err = s2.SetTarget(0)
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

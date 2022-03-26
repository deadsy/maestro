//-----------------------------------------------------------------------------
/*

Maestro Servo Controller

See: https://www.pololu.com/docs/pdf/0J40/maestro.pdf

*/
//-----------------------------------------------------------------------------

package sc

import (
	"errors"
	"strings"

	"github.com/tarm/serial"
)

//-----------------------------------------------------------------------------

const cmdSetTarget = 0x84
const cmdSetSpeed = 0x87
const cmdSetAcceleration = 0x89
const cmdSetPWM = 0x8a
const cmdGetPosition = 0x90
const cmdGetMovingState = 0x93
const cmdSetMultipleTargets = 0x9f
const cmdGetErrors = 0xa1
const cmdGoHome = 0xa2
const cmdStopScript = 0xa4
const cmdRestartScript = 0xa7
const cmdRestartScriptParms = 0xa8
const cmdGetScriptStatus = 0xae

//-----------------------------------------------------------------------------

// GetError converts an error bitmap into a go error object.
func GetError(val uint16) error {
	errorStrings := []string{
		"serial signal error",          // bit 0
		"serial overrun error",         // bit 1
		"serial buffer full",           // bit 2
		"serial crc error",             // bit 3
		"serial protocol error",        // bit 4
		"serial timeout",               // bit 5
		"script stack error",           // bit 6
		"script call stack error",      // bit 7
		"script program counter error", // bit 8
	}
	s := []string{}
	for i, err := range errorStrings {
		if val&(1<<i) != 0 {
			s = append(s, err)
		}
	}
	if len(s) == 0 {
		return nil
	}
	return errors.New(strings.Join(s, ","))
}

//-----------------------------------------------------------------------------
// Controller

// Controller is a servo controller instance.
type Controller struct {
	port    *serial.Port // serial port
	device  uint8        // device number
	compact bool         // use the compact protocol (single device on serial bus)
}

// NewController returns a new servo motor controller.
func NewController(port *serial.Port, device uint8, compact bool) *Controller {
	return &Controller{
		port:    port,
		device:  device,
		compact: compact,
	}
}

func (c *Controller) cmdPreamble(command uint8) []byte {
	if c.compact {
		return []byte{command}
	}
	return []byte{0xaa, c.device, command & 0x7f}
}

// GetMovingState returns true if any servos are moving.
func (c *Controller) GetMovingState() (bool, error) {
	cmd := c.cmdPreamble(cmdGetMovingState)
	_, err := c.port.Write(cmd)
	buf := make([]byte, 1)
	_, err = c.port.Read(buf)
	if err != nil {
		return false, err
	}
	return buf[0] != 0, nil
}

// GetErrors returns the controller error code.
func (c *Controller) GetErrors() (uint16, error) {
	cmd := c.cmdPreamble(cmdGetErrors)
	_, err := c.port.Write(cmd)
	buf := make([]byte, 2)
	_, err = c.port.Read(buf)
	if err != nil {
		return 0, err
	}
	return (uint16(buf[0]) & 0x7f) + (uint16(buf[1])&0x7f)<<8, nil
}

// GoHome sends all servos to their home position.
func (c *Controller) GoHome() error {
	cmd := c.cmdPreamble(cmdGoHome)
	_, err := c.port.Write(cmd)
	return err
}

// StopScript stops the execution of a servo user script.
func (c *Controller) StopScript() error {
	cmd := c.cmdPreamble(cmdStopScript)
	_, err := c.port.Write(cmd)
	return err
}

// RestartScript restarts the servo script at a specified subroutine.
func (c *Controller) RestartScript(subroutine uint8) error {
	cmd := c.cmdPreamble(cmdRestartScript)
	cmd = append(cmd, subroutine)
	_, err := c.port.Write(cmd)
	return err
}

// RestartScriptParms restarts the servo script at a specified subroutine and parameter value.
func (c *Controller) RestartScriptParms(subroutine uint8, val uint16) error {
	cmd := c.cmdPreamble(cmdRestartScriptParms)
	cmd = append(cmd, []byte{subroutine, lo(val), hi(val)}...)
	_, err := c.port.Write(cmd)
	return err
}

// GetScriptStatus returns true if a servo script is running.
func (c *Controller) GetScriptStatus() (bool, error) {
	cmd := c.cmdPreamble(cmdGetScriptStatus)
	_, err := c.port.Write(cmd)
	buf := make([]byte, 1)
	_, err = c.port.Read(buf)
	if err != nil {
		return false, err
	}
	return buf[0] == 0, nil
}

//-----------------------------------------------------------------------------
// Servo

// Servo is a servo motor instance.
type Servo struct {
	ctrl    *Controller
	channel uint8
}

// NewServo returns a new servo motor instance.
func NewServo(ctrl *Controller, channel uint8) *Servo {
	return &Servo{
		ctrl:    ctrl,
		channel: channel,
	}
}

func lo(x uint16) byte {
	return byte(x & 0x7f)
}

func hi(x uint16) byte {
	return byte((x >> 7) & 0x7f)
}

func (s *Servo) cmdPreamble(command uint8) []byte {
	if s.ctrl.compact {
		return []byte{command, s.channel}
	}
	return []byte{0xaa, s.ctrl.device, command & 0x7f, s.channel}
}

// SetTarget sets the servo target value.
func (s *Servo) SetTarget(target uint16) error {
	cmd := s.cmdPreamble(cmdSetTarget)
	cmd = append(cmd, []byte{lo(target), hi(target)}...)
	_, err := s.ctrl.port.Write(cmd)
	return err
}

// SetMultipleTargets sets the target value for multiple servos (starting at the referenced servo).
func (s *Servo) SetMultipleTargets(targets []uint16) error {
	cmd := make([]byte, 0, 5+2*len(targets))
	if s.ctrl.compact {
		cmd = []byte{cmdSetMultipleTargets, byte(len(targets)), s.channel}
	} else {
		cmd = []byte{0xaa, s.ctrl.device, cmdSetMultipleTargets & 0x7f, byte(len(targets)), s.channel}
	}
	for _, v := range targets {
		cmd = append(cmd, []byte{lo(v), hi(v)}...)
	}
	_, err := s.ctrl.port.Write(cmd)
	return err
}

// SetSpeed sets the servo maximum speed (0 is no limit).
func (s *Servo) SetSpeed(speed uint16) error {
	cmd := s.cmdPreamble(cmdSetSpeed)
	cmd = append(cmd, []byte{lo(speed), hi(speed)}...)
	_, err := s.ctrl.port.Write(cmd)
	return err
}

// SetAcceleration sets the servo maximum acceleration (0 is no limit).
func (s *Servo) SetAcceleration(acceleration uint16) error {
	cmd := s.cmdPreamble(cmdSetAcceleration)
	cmd = append(cmd, []byte{lo(acceleration), hi(acceleration)}...)
	_, err := s.ctrl.port.Write(cmd)
	return err
}

// SetPWM sets the ontime and period for a servo control signal.
func (s *Servo) SetPWM(ontime, period uint16) error {
	cmd := s.cmdPreamble(cmdSetPWM)
	cmd = append(cmd, []byte{lo(ontime), hi(ontime), lo(period), hi(period)}...)
	_, err := s.ctrl.port.Write(cmd)
	return err
}

// GetPosition returns the current commanded position of a servo.
func (s *Servo) GetPosition() (uint16, error) {
	cmd := s.cmdPreamble(cmdGetPosition)
	_, err := s.ctrl.port.Write(cmd)
	buf := make([]byte, 2)
	_, err = s.ctrl.port.Read(buf)
	if err != nil {
		return 0, err
	}
	return uint16(buf[0]) + uint16(buf[1])<<8, nil
}

//-----------------------------------------------------------------------------

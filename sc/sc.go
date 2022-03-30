//-----------------------------------------------------------------------------
/*

Maestro Servo Controller

See: https://www.pololu.com/docs/pdf/0J40/maestro.pdf

*/
//-----------------------------------------------------------------------------

package sc

import (
	"errors"
	"fmt"
	"io"
	"strings"
)

//-----------------------------------------------------------------------------

// commands
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
const cmdSetTargetHighResolution = 0xc0       // jrk motor controller
const cmdSetTargetLowResolutionReverse = 0xe0 // jrk motor controller
const cmdSetTargetLowResolutionForward = 0xe1 // jrk motor controller
const cmdMotorOff = 0xff                      // jrk motor controller

// position ticks per uSec of servo control pulse
const uSec = 4

// 14 bits of target position
const maxTarget = 0x3fff

// maximum number of servos per controller
const maxServos = 24

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

// Config is the servo controller configuration.
type Config struct {
	Port         io.ReadWriter // serial port
	DeviceNumber uint8         // device number
	Compact      bool          // use the compact protocol (single device on serial bus)
	Crc          bool          // add a crc byte to outgoing commands
}

// Controller is a servo controller instance.
type Controller struct {
	port    io.ReadWriter     // serial port
	device  uint8             // device number
	compact bool              // use the compact protocol (single device on serial bus)
	crc     bool              // add a crc byte to outgoing commands
	servo   [maxServos]*Servo // child servos
}

// NewController returns a new servo motor controller.
func NewController(cfg *Config) (*Controller, error) {
	c := &Controller{
		port:    cfg.Port,
		device:  cfg.DeviceNumber,
		compact: cfg.Compact,
		crc:     cfg.Crc,
	}
	// send a 0xaa for auto baud detection
	_, err := c.port.Write([]byte{0xaa})
	if err != nil {
		return nil, err
	}
	return c, nil
}

func (c *Controller) cmdPreamble(command uint8) []byte {
	if c.compact {
		return []byte{command}
	}
	return []byte{0xaa, c.device, command & 0x7f}
}

// cmdWrite writes a command to the serial port.
func (c *Controller) cmdWrite(cmd []byte) error {
	if c.crc {
		cmd = append(cmd, crc7(0, cmd)&0x7f)
	}
	_, err := c.port.Write(cmd)
	if err != nil {
		return err
	}
	return nil
}

// rspRead reads a response from the serial port.
func (c *Controller) rspRead(buf []byte) error {
	n, err := c.port.Read(buf)
	if err != nil {
		return err
	}
	if n != len(buf) {
		return errors.New("short read")
	}
	return nil
}

// GetMovingState returns true if the controller has not reached the target value for all servos.
// True implies the servos are moving. False does not imply the servos have stopped moving.
func (c *Controller) GetMovingState() (bool, error) {
	err := c.cmdWrite(c.cmdPreamble(cmdGetMovingState))
	if err != nil {
		return false, err
	}
	var buf [1]byte
	err = c.rspRead(buf[:])
	if err != nil {
		return false, err
	}
	return buf[0] != 0, nil
}

// GetErrors returns the controller error code.
func (c *Controller) GetErrors() (uint16, error) {
	err := c.cmdWrite(c.cmdPreamble(cmdGetErrors))
	if err != nil {
		return 0, err
	}
	var buf [2]byte
	err = c.rspRead(buf[:])
	if err != nil {
		return 0, err
	}
	return (uint16(buf[0]) & 0x7f) + (uint16(buf[1])&0x7f)<<8, nil
}

// GoHome sends all servos to their home position.
func (c *Controller) GoHome() error {
	return c.cmdWrite(c.cmdPreamble(cmdGoHome))
}

// StopScript stops the execution of a servo user script.
func (c *Controller) StopScript() error {
	return c.cmdWrite(c.cmdPreamble(cmdStopScript))
}

// RestartScript restarts the servo script at a specified subroutine.
func (c *Controller) RestartScript(subroutine uint8) error {
	cmd := c.cmdPreamble(cmdRestartScript)
	cmd = append(cmd, subroutine)
	return c.cmdWrite(cmd)
}

// RestartScriptParms restarts the servo script at a specified subroutine and parameter value.
func (c *Controller) RestartScriptParms(subroutine uint8, val uint16) error {
	cmd := c.cmdPreamble(cmdRestartScriptParms)
	cmd = append(cmd, []byte{subroutine, lo(val), hi(val)}...)
	return c.cmdWrite(cmd)
}

// GetScriptStatus returns true if a servo script is running.
func (c *Controller) GetScriptStatus() (bool, error) {
	err := c.cmdWrite(c.cmdPreamble(cmdGetScriptStatus))
	if err != nil {
		return false, err
	}
	var buf [1]byte
	err = c.rspRead(buf[:])
	if err != nil {
		return false, err
	}
	return buf[0] == 0, nil
}

// SetTargets sets the target value for multiple servos (starting at the referenced servo).
func (c *Controller) SetTargets(channel uint8, targets []uint16) error {
	if len(targets) == 0 {
		return nil
	}
	// build the command
	cmd := c.cmdPreamble(cmdSetMultipleTargets)
	cmd = append(cmd, []byte{byte(len(targets)), channel}...)
	// check and append the target values
	for i, v := range targets {
		ch := channel + uint8(i)
		if ch >= maxServos || c.servo[ch] == nil {
			return fmt.Errorf("bad servo channel %d", ch)
		}
		val, err := c.servo[ch].checkTarget(v)
		if err != nil {
			return fmt.Errorf("%s for channel %d", err.Error(), ch)
		}
		cmd = append(cmd, []byte{lo(val), hi(val)}...)
	}
	// send the command
	return c.cmdWrite(cmd)
}

//-----------------------------------------------------------------------------
// Servo

// Servo is a servo motor instance.
type Servo struct {
	ctrl    *Controller // parent controller
	channel uint8       // servo channel number
	min     uint16      // minimum target position
	max     uint16      // maximum target position
	clamp   bool        // clamp out-of-range target values
}

// NewServo returns a new servo motor instance.
func (c *Controller) NewServo(channel uint8) (*Servo, error) {
	if channel >= maxServos {
		return nil, fmt.Errorf("bad servo channel %d", channel)
	}
	s := &Servo{
		ctrl:    c,
		channel: channel,
		min:     500 * uSec,
		max:     2500 * uSec,
		clamp:   false,
	}
	c.servo[channel] = s
	return s, nil
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

// checkTarget clamps/limits the servo target value
func (s *Servo) checkTarget(target uint16) (uint16, error) {
	if s.clamp {
		if target < s.min {
			return s.min, nil
		}
		if target > s.max {
			return s.max, nil
		}
	} else {
		if target < s.min {
			return s.min, errors.New("target too low")
		}
		if target > s.max {
			return s.max, errors.New("target too high")
		}
	}
	return target, nil
}

// SetLimits sets the minimum/maximum values for the servo target position.
func (s *Servo) SetLimits(min, max uint16) error {
	if max > min {
		return errors.New("max > min")
	}
	if min > maxTarget {
		return fmt.Errorf("min > %d", maxTarget)
	}
	if max > maxTarget {
		return fmt.Errorf("max > %d", maxTarget)
	}
	s.min = min
	s.max = max
	return nil
}

// SetTarget sets the servo target value.
func (s *Servo) SetTarget(target uint16) error {
	target, err := s.checkTarget(target)
	if err != nil {
		return err
	}
	cmd := s.cmdPreamble(cmdSetTarget)
	cmd = append(cmd, []byte{lo(target), hi(target)}...)
	return s.ctrl.cmdWrite(cmd)
}

// SetSpeed sets the servo maximum speed (0 is no limit).
func (s *Servo) SetSpeed(speed uint16) error {
	cmd := s.cmdPreamble(cmdSetSpeed)
	cmd = append(cmd, []byte{lo(speed), hi(speed)}...)
	return s.ctrl.cmdWrite(cmd)
}

// SetAcceleration sets the servo maximum acceleration (0 is no limit).
func (s *Servo) SetAcceleration(acceleration uint16) error {
	cmd := s.cmdPreamble(cmdSetAcceleration)
	cmd = append(cmd, []byte{lo(acceleration), hi(acceleration)}...)
	return s.ctrl.cmdWrite(cmd)
}

// SetPWM sets the ontime and period for a servo control signal.
func (s *Servo) SetPWM(ontime, period uint16) error {
	cmd := s.cmdPreamble(cmdSetPWM)
	cmd = append(cmd, []byte{lo(ontime), hi(ontime), lo(period), hi(period)}...)
	return s.ctrl.cmdWrite(cmd)
}

// GetPosition returns the current commanded position for the servo.
func (s *Servo) GetPosition() (uint16, error) {
	err := s.ctrl.cmdWrite(s.cmdPreamble(cmdGetPosition))
	if err != nil {
		return 0, err
	}
	var buf [2]byte
	err = s.ctrl.rspRead(buf[:])
	if err != nil {
		return 0, err
	}
	return uint16(buf[0]) + uint16(buf[1])<<8, nil
}

//-----------------------------------------------------------------------------

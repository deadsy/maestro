//-----------------------------------------------------------------------------
/*

CRC-7 Calculations

See: https://www.pololu.com/docs/0J44/6.7.6

*/
//-----------------------------------------------------------------------------

package sc

//-----------------------------------------------------------------------------

func init() {
	initCrcTable()
}

//-----------------------------------------------------------------------------

const crc7Poly = 0x91

var crcTable [256]uint8

func getCRCForByte(val uint8) uint8 {
	for j := 0; j < 8; j++ {
		if val&1 != 0 {
			val ^= crc7Poly
		}
		val >>= 1
	}
	return val
}

func initCrcTable() {
	for i := 0; i < 256; i++ {
		crcTable[i] = getCRCForByte(byte(i))
	}
}

func crc7(crc uint8, buf []byte) uint8 {
	for _, v := range buf {
		crc = crcTable[crc^v]
	}
	return crc
}

//-----------------------------------------------------------------------------

#include "IRremote.h"
#include "IRremoteInt.h"
#include "ir_Lego_PF_BitStreamEncoder.h"

//==============================================================================
//    L       EEEEEE   EEEE    OOOO
//    L       E       E       O    O
//    L       EEEE    E  EEE  O    O
//    L       E       E    E  O    O    LEGO Power Functions
//    LLLLLL  EEEEEE   EEEE    OOOO     Copyright (c) 2016 Philipp Henkel
//==============================================================================

// Supported Devices
// LEGO® Power Functions IR Receiver 8884

//+=============================================================================
#if SEND_LEGO_PF

#if DEBUG
namespace {
void logFunctionParameters(uint16_t data, bool repeat) {
  DBG_PRINT("sendLegoPowerFunctions(data=");
  DBG_PRINT(data);
  DBG_PRINT(", repeat=");
  DBG_PRINTLN(repeat?"true)" : "false)");
}
} // anonymous namespace
#endif // DEBUG

void IRsend::sendLegoPowerFunctions(uint16_t data, bool repeat)
{
#if DEBUG
  ::logFunctionParameters(data, repeat);
#endif // DEBUG

  enableIROut(38);
  static LegoPfBitStreamEncoder bitStreamEncoder;
  bitStreamEncoder.reset(data, repeat);
  do {
    mark(bitStreamEncoder.getMarkDuration());
    space(bitStreamEncoder.getPauseDuration());
  } while (bitStreamEncoder.next());
}

#endif // SEND_LEGO_PF

//+=============================================================================
#if DECODE_LEGO_PF
bool  IRrecv::decodeLegoPowerFunctions (decode_results *results) {
	static LegoPfBitStreamEncoder bitStreamEncoder;
	uint16_t data   = 0;
	int  offset = 1;

	// Check SIZE
	if (irparams.rawlen < 2 * (bitStreamEncoder.MESSAGE_BITS) - 1)  return false ;  // exclude STOP pause

	// Check START_BIT Mark/Space
	if (!MATCH_MARK (results->rawbuf[offset++], bitStreamEncoder.IR_MARK_DURATION - 58)) return false ;
	if (!MATCH_SPACE(results->rawbuf[offset++], bitStreamEncoder.START_PAUSE_DURATION + 100)) return false ;

	while(offset < irparams.rawlen - 1) {
		if (MATCH_MARK(results->rawbuf[offset], bitStreamEncoder.IR_MARK_DURATION - 58)) {
			offset++ ;
		}
		else {
			return false ;
		}

		// ONE & ZERO
		if (MATCH_SPACE(results->rawbuf[offset], bitStreamEncoder.HIGH_PAUSE_DURATION + 50)) {
			data = (data << 1) | 1 ;
		}
		else if (MATCH_SPACE(results->rawbuf[offset], bitStreamEncoder.LOW_PAUSE_DURATION + 62)) {
			data = (data << 1) | 0 ;
		}
		else {
			break ;  // End of one & zero detected
		}
		offset++;
	}

	results->bits = (offset - 2) / 2;
	if (results->bits < 16) {
		return false ;
	}

	// Check checksum of data found
	if (!bitStreamEncoder.verifyChecksum(data)) {
		return false ;
	}

	results->value       = data;
	results->decode_type = LEGO_PF;
	return true;
}

#endif // DECODE_LEGO_PF

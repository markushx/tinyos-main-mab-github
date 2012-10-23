COMPONENT=PPPSnifferC

# this is necessary, otherwise we will allocate a heap by including
# the lowpan target
CFLAGS += -DNO_IP_MALLOC

# this aligns the 802.15.4 payload on the data field of a message_t.
# very convenient for forwarding, and means we don't have to do a
# memmove.  This doesn't mean that motes running blip also need this--
# they can use IFRAMES or IEEE154FRAMES
#CFLAGS += -DIEEE154FRAMES_ENABLED

CFLAGS += -DRADIO_DEBUG_PPP
# also receive big blip packets....
CFLAGS += -DTOSH_DATA_LENGTH=120

CFLAGS += -DCC2420X_DEF_CHANNEL=26

TOSMAKE_PATH += .
CFLAGS += -I$(TOSDIR)/lib/diagmsg
PFLAGS += -I$(TOSDIR)/lib/ppp
PFLAGS += -I$(TOSDIR)/lib/fragpool

# -Os breaks the PPP or serial stack somehow...
OPTFLAGS += -O2

include $(MAKERULES)

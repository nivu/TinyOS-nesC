#ifndef MOTE_TO_MOTE_H
#define MOTE_TO_MOTE_H

typedef nx_struct MoteToMoteMsg
{
	nx_uint16_t NodeId;
	nx_uint8_t Data;
	nx_uint8_t Data2;


}
	MoteToMoteMsg_t;
	
enum
{
	AM_RADIO = 6
};
#endif /* MOTE_TO_MOTE_H */
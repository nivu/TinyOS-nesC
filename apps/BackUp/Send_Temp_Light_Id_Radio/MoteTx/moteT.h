#ifndef MOTE_T_H
#define MOTE_T_H
typedef nx_struct MoteToMoteMsg
{
	nx_uint16_t NodeId;
	nx_uint8_t Temp;
	nx_uint8_t Hum;
	nx_uint8_t Photo;

}
	MoteToMoteMsg_t;
	
enum
{
	AM_RADIO = 6
};
#endif /* MOTE_T_H */

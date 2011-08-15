#ifndef FRAME_H_
#define FRAME_H_

#include "compiler.h"

#define FRAME_PRIVILEGIED_CMD     0x00
#define FRAME_SENDIDM_CMD         0x10
#define FRAME_SENDIDREF_CMD       0x11
#define FRAME_SENDIQM_CMD         0x12
#define FRAME_SENDIQREF_CMD       0x13
#define FRAME_SENDSPEEDM_CMD      0x14
#define FRAME_SENDSPEEDREF_CMD    0x15
#define FRAME_SENDSPEEDMES_CMD    0x16
#define FRAME_SENDSPEEDEST_CMD    0x17
#define FRAME_SENDTETAM_CMD       0x18
#define FRAME_SENDTETAEST_CMD     0x19

#define FRAME_GETIDREF			  0x20
#define FRAME_GETIQREF			  0x21
#define FRAME_GETVREF			  0x22
#define FRAME_GETKD			  0x23
#define FRAME_GETKDV			  0x24
#define FRAME_GETKITE			  0x25
#define FRAME_GETKIVTE			  0x26

typedef struct{
	unsigned char cmd;
	unsigned char dlc;
	unsigned char data[255];
}frame_message;

#endif /* FRAME_H_ */

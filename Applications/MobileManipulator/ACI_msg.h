#ifndef __ACI_MSG_H__
#define __ACI_MSG_H__

#include <iostream>

#include "ulapi.h"

extern bool ACI_debug;

///////////////////////////////////////////////////////
// define exception classes
class ACI_read_error : public std::exception
{
public:
	ACI_read_error( const char * const & message = "" ) : std::exception( message ) {}
};

class ACI_write_error : public std::exception
{
public:
	ACI_write_error( const char * const & message = "" ) : std::exception( message ) {}
};


class ACI_format_error : public std::exception
{
public:
	ACI_format_error( const char * const & message = "" ) : std::exception( message ) {}
};

class ACI_parameter_error : public std::exception
{
public:
	ACI_parameter_error( const char * const & message = "" ) : std::exception( message ) {}
};

////////////////////////////////////////////////////////
struct ACI_Header
{
	ACI_Header();
	void read( ulapi_integer id );
	void write( ulapi_integer id );
	void print();

	enum Function_Code {
		NORMAL_MESSAGE = 1,
		DISCONNECT_LINK,
		RESERVED,
		HEART_BEAT_POLL,
		HEART_BEAT_ACK,
	};

	Function_Code function_code;
	int header_size;
	int message_size;
};

///////////////////////////////////////////////////////////////
struct ACI_Msg
{
	ACI_Msg( short msg_type, const char* msg_name );
	ACI_Msg( ACI_Header header, short msg_type, const char* msg_name );
	virtual void read( ulapi_integer id );
	virtual void print();
	virtual void write( ulapi_integer id );

	ACI_Header header;
	short msg_type;
	const char* msg_name;
	short num_params;
};

//////////////////////////////////////////////////////////////
struct ACI_Unknown_Msg : public ACI_Msg
{
	ACI_Unknown_Msg( ACI_Header header );
	virtual void read( ulapi_integer id );
	virtual void print();

	unsigned char data[100];
};

/////////////////////////////////////////////////////////////
struct ACI_Order_Status_Msg : public ACI_Msg
{
	ACI_Order_Status_Msg( ACI_Header header );
	virtual void read( ulapi_integer id );
	virtual void print();

	enum Order_Status {
		TS_NOT_VALID = 0,
		NOT_USED,
		PENDING,
		TRANSITORY1,
		TRANSITORY2,
		WAITING_FOR_A_VEHICLE,
		TRANSITORY3,
		MOVE_VEHICLE
	};

	short index;   // 1-999, 0xFFFF if not valid
	char transport_structure;   // 1-255, 0 if not valid
	Order_Status order_status;
	short magic1;
	short magic2;
	char car_no;  // 1-255, 0 if not valid
	char spare; // always 0
	short car_stat;
	short car_stn;
	short magic3;
	short no_of_lp; // 0-31
	short lp[32];   // 0-0x7FFF, 0xFFFF if empty
};

/////////////////////////////////////////////////////////////
struct ACI_Order_Status_Extended_Msg : public ACI_Msg
{
	ACI_Order_Status_Extended_Msg( ACI_Header header );
};

/////////////////////////////////////////////////////////////////
struct ACI_Local_Parameter_Msg : public ACI_Msg
{
	ACI_Local_Parameter_Msg( ACI_Header header );
	ACI_Local_Parameter_Msg();
	virtual void write( ulapi_integer id );

	enum Function {
		SPONTANEOUS_UPDATE,
		REQUESTED_PARAMETER_UPDATE,
		DELETE_PARAMETER,
		READ_PARAMETER_AREA,
		CHANGE_ORDER_PRIORITY,
		CONNECT_ALLOCATED_VEHICLE,
	};

	int index;
	Function function;
	short param_num;
	short pcount; // number of parameters to transfer
	unsigned char par_no;
	short p[5];
	unsigned char prio;
	unsigned char AGVId;
};

struct ACI_Parameter_Area_Contents_Msg : public ACI_Msg
{
	ACI_Parameter_Area_Contents_Msg( ACI_Header header );
	virtual void read( ulapi_integer id );
	virtual void print();

	short index;
	short no_par;
	short p_no[5];
	short p_val[5];

};

struct ACI_Global_Parameter_Command_Msg : public ACI_Msg
{
	ACI_Global_Parameter_Command_Msg( ACI_Header header );
	//virtual void read( ulapi_integer id );

	enum Access_Function {
		READ = 1,
		WRITE };

	short magic;
	int par_num;
	int par_ix;
	short par[16];
};

struct ACI_Global_Parameter_Status_Msg : public ACI_Msg
{
	ACI_Global_Parameter_Status_Msg( ACI_Header header );
	//virtual voidread( ulapi_integer id );
};

struct ACI_OM_to_PLC_Msg : public ACI_Msg
{
	ACI_OM_to_PLC_Msg( ACI_Header header );
	ACI_OM_to_PLC_Msg();
	virtual void write( ulapi_integer id );

	enum Code {
		NONE = 0,
		READ = 1,
		WRITE,
		MULTI_READ = 5,
		MULTI_WRITE,
	};

	unsigned char res;
	unsigned char carid;
	short magic;
	Code c1;
	unsigned char s1;
	unsigned char om1;
	unsigned char plc1;
	short par1;
	Code c2;
	unsigned char s2;
	unsigned char om2;
	unsigned char plc2;
	short par2;
	unsigned char par_count;
	unsigned char par[240];
};

struct ACI_PLC_to_OM_Msg : public ACI_Msg
{
	ACI_PLC_to_OM_Msg( ACI_Header header );
	virtual void read( ulapi_integer id );
	virtual void print();

	enum Code {
		READ_ACK = 1,
		WRITE_ACK,
		READ_NAK,
		WRITE_NAK,
		MULTI_READ_ACK = 5,
		MULTI_WRITE_ACK,
		MULTI_READ_NAK,
		MULTI_WRITE_NAK,
	};

	unsigned char res;
	unsigned char carid;
	short magic;
	Code c1;
	unsigned char s1;
	unsigned char om1;
	unsigned char plc1;
	short par1;
	Code c2;
	unsigned char s2;
	unsigned char om2;
	unsigned char plc2;
	short par2;
	unsigned char par[240];
	short plc;
	unsigned char seq;
	unsigned char last;
	int num;

private:
	void print( Code c );
};

//////////////////////////////////////////////////////////////////////
struct ACI_Delete_Order_Msg : public ACI_Msg
{
	ACI_Delete_Order_Msg();
	ACI_Delete_Order_Msg( ACI_Header header );
	void write( ulapi_integer id );

	short index;
	unsigned char carno;
};

///////////////////////////////////////////////////////////////////
struct ACI_Order_Status_Request_Msg : public ACI_Msg
{
	ACI_Order_Status_Request_Msg();
	ACI_Order_Status_Request_Msg( ACI_Header header );
	void write( ulapi_integer id );

	short index;
	unsigned char carno;
};

////////////////////////////////////////////////////////////////////
struct ACI_Order_Initiate_Msg : public ACI_Msg
{
	ACI_Order_Initiate_Msg();
	ACI_Order_Initiate_Msg( ACI_Header header);
	void write( ulapi_integer id );
	
	enum Code {   // values are OR'd together
		DEFAULT = 0,
		IKEY = 0x01,
		TRACE = 0x02,
		DEBUG = 0x04 
	};

	unsigned char trp_str;
	unsigned char pri;
	short code;
	short ikey;
	int p_count;
	short p[5];
};

////////////////////////////////////////////////////////////////////
struct ACI_Order_Acknowledge_Msg : public ACI_Msg
{
	ACI_Order_Acknowledge_Msg( ACI_Header header );
	virtual void read( ulapi_integer id );
	virtual void print();
	const char* status2str( unsigned char status );

	short index;
	unsigned char trp_str;
	unsigned char status;
	unsigned char par_no;
	unsigned char spare;
	short ikey;
};

/////////////////////////////////////////////////////////////////////
struct ACI_Heartbeat_Msg : public ACI_Msg
{
	ACI_Heartbeat_Msg();
	ACI_Heartbeat_Msg( ACI_Header header );
	void read( ulapi_integer id );
	void write( ulapi_integer id );
	void print();
};

/////////////////////////////////////////////////////////////////////
struct ACI_Parameter_Request_Msg : public ACI_Msg
{	
	ACI_Parameter_Request_Msg( ACI_Header header );
};

//////////////////////////////////////////////////////////////////////
ACI_Msg *ACI_read_message( ulapi_integer id );

#endif

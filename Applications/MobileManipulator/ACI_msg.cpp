#include "ACI_msg.h"

#include <iostream>
#include <iomanip>

bool ACI_debug = false;

using namespace std;

///////////////////////////////////////////////////////////////
unsigned char ACI_read_byte( ulapi_integer id )
{
	unsigned char value;
	int count = ulapi_socket_read( id, (char *)&value, 1 );
	if( count != 1 )
		throw ACI_read_error();
	return value;
}

/////////////////////////////////////////////////////////////////////
short ACI_read_short( ulapi_integer id )
{
	unsigned char buffer[2];
	int count = ulapi_socket_read( id, (char *)buffer, 2 );
	if( count != 2 )
		throw ACI_read_error();
	return( buffer[0]<<8 | buffer[1] );
}

/////////////////////////////////////////////////////////////////////
void ACI_write_byte( ulapi_integer id,
	unsigned char c )
{
	ulapi_integer count = ulapi_socket_write( id, (char *) &c, 1 );
	if( count != 1 )
		throw ACI_write_error();
}

/////////////////////////////////////////////////////////////////////
void ACI_write_short( ulapi_integer id,
	short s )
{
	ulapi_integer count = 0;

	char *ptr = (char *) &s;
	count += ulapi_socket_write( id, ptr+1, 1 ); 
	count += ulapi_socket_write( id, ptr, 1 );

	if( count != 2 )
		throw ACI_write_error();
}

////////////////////////////////////////////////////////////////
ACI_Header::ACI_Header()
	: function_code(NORMAL_MESSAGE),
	header_size(8),
	message_size(0)
{
}

/////////
void ACI_Header::read( 
	ulapi_integer id )
{
	bool ok = true;
	
	// search till we find the header key
	while( ok )
	{
		while( ACI_read_byte( id ) != (unsigned char)0x87 ) {}

		if( ACI_read_byte( id ) == (unsigned char)0xCD )
		{
			break;
		}
	}

	header_size = ACI_read_short( id );
	message_size = ACI_read_short( id );
	function_code = (Function_Code) ACI_read_short( id );

	// check values
	if( header_size != 8 ) throw ACI_format_error("Incorrect header size");
	if( message_size<0 || message_size>128 ) throw ACI_format_error("Invalid message size");
	if( (int)function_code<1 || (int)function_code>5 ) throw ACI_format_error("Invalid function code size");
}

//////////
void ACI_Header::write( ulapi_integer id )
{
	ACI_write_short( id, (short)0x87CD );
	ACI_write_short( id, header_size );
	ACI_write_short( id, message_size );
	ACI_write_short( id, function_code );
}

//////////
void ACI_Header::print()
{
	cout << "header_size = " << header_size << ", message_size = " << message_size << ", function_code = ";
	switch( function_code )
	{
	case NORMAL_MESSAGE:   cout << "NORMAL_MESSAGE"; break;
	case DISCONNECT_LINK:  cout << "DISCONNECT_LINK"; break;
	case RESERVED:         cout << "RESERVED"; break;
	case HEART_BEAT_POLL:  cout << "HEART_BEAT_POLL"; break;
	case HEART_BEAT_ACK:   cout << "HEART_BEAT_ACK"; break;
	default:               cout << "Unknown"; break;
	}
}
/////////////////////////////////////////////////////////////////
ACI_Msg::ACI_Msg( short _msg_type, 
	const char* _name )
	: msg_type(_msg_type ),
	msg_name( _name ),
	num_params(0)
{
}

ACI_Msg::ACI_Msg( ACI_Header _header,
	short _msg_type,
	const char* name )
	: header( _header ),
	msg_type(_msg_type ),
	msg_name( name ),
	num_params(0)
{
}

/////////
void ACI_Msg::read( 
	ulapi_integer id )
{
	cout << "read() not implemented for " << msg_name << endl;
}

////////
void ACI_Msg::print()
{
	cout << "print() not implemented for " << msg_name << endl;
}

////////
void ACI_Msg::write( ulapi_integer id )
{
	cout << "write() not implemented for " << msg_name << endl;	
}

////////////////////////////////////////////////////////////////////////////
ACI_Unknown_Msg::ACI_Unknown_Msg( 
	ACI_Header header )
	: ACI_Msg( header, '?', "Unknown" )
{
}

/////////
void ACI_Unknown_Msg::read( 
	ulapi_integer id )
{
	num_params = ACI_read_short( id );
	if( num_params != header.message_size - 4 )
		throw ACI_format_error( "Message size and parameter number do not agree" );

	for( int i=0; i<num_params; ++i )
		data[i] = ACI_read_byte( id );
}

////////
void ACI_Unknown_Msg::print()
{
	cout << "Unknown Message Type: ";
	header.print();
	cout << ", msg_type = " << msg_type << " '" << (char)msg_type << "', num_params = " << num_params
		<< ", data = ";

	cout << setbase(16) << setfill('0');
	for( int i=0; i<num_params; ++i )
		cout << " " << setw(2) << (int)data[i];
	cout << setbase(10) << setfill(' ');

	cout << endl;
}

////////////////////////////////////////////////////////////////////////////
ACI_Order_Status_Msg::ACI_Order_Status_Msg( 
	ACI_Header header )
	: ACI_Msg( header, 's', "Order Status" ),
	index(0),
	transport_structure(0),
	order_status(TS_NOT_VALID),
	magic1(0),
	magic2(0),
	car_no(0),
	spare(0),
	car_stat(0),
	car_stn(0),
	magic3(0),
	no_of_lp(0)
{
	for( int i=0; i<32; ++i )
		lp[i] = 0;
}

///////////
void ACI_Order_Status_Msg::read( 
	ulapi_integer id )
{
	num_params = ACI_read_short( id );
	if( num_params != header.message_size - 4 )
		throw ACI_format_error( "Message size and parameter number do not agree" );

	index = ACI_read_short( id );
	if( !( (0 <= index && index <= 999 ) || index == 0xffff ) )
		throw ACI_format_error( "Invalid index" );
	transport_structure = ACI_read_byte( id );
	order_status = (Order_Status) ACI_read_byte( id);
	if( !(1 <= (int)order_status && (int)order_status <= 7) )
		throw ACI_format_error( "Invalid order status" );
	magic1 = ACI_read_short( id );
	if( header.message_size > 10 )
	{
		magic2 = ACI_read_short( id );
		car_no = ACI_read_byte( id );
		spare = ACI_read_byte( id );
		car_stat = ACI_read_short( id );
	}
	if( header.message_size > 16 )
	{
		car_stn = ACI_read_short( id );
		magic3 = ACI_read_short( id );
	}
	if( header.message_size > 20 )
	{
		no_of_lp = ACI_read_short( id );
		if( no_of_lp > 31 )
			throw ACI_format_error( "Number of local parameters is too large" );
		for( int i=0; i<no_of_lp; ++i )
		{
			lp[i] = ACI_read_short( id );
		}
	}
}

//////////
void ACI_Order_Status_Msg::print()
{
	cout << "Order Status Message: ";
	header.print();
	cout << ", msg_type = " << msg_type << " '" << (char)msg_type << "'";
	cout << ", num_params = " << num_params << ", index = " << index << ", transport_structure = " 
		<< (int)transport_structure << ", order_status = " << (int)order_status << ", magic1 = " << magic1;

	if( header.message_size > 10 )
	{
		cout << ", magic2 = " << magic2 << ", car_no = " << (int)car_no << ", spare = " << (int)spare 
			<< ", car_stat = " << car_stat;
	}

	if( header.message_size > 16 )
	{
		cout << ", car_stn = " << car_stn << ", magic3 = " << magic3;
	}

	if( header.message_size > 20 )
	{
		cout << ", no_of_lp = " << no_of_lp;
		for( int i=0; i<no_of_lp; ++i )
		{
			cout << ", lp" << i << " = " << lp[i];
		}
	}

	cout << endl;
}

////////////////////////////////////////////////////////////////
ACI_Order_Status_Extended_Msg::ACI_Order_Status_Extended_Msg( ACI_Header header )
	: ACI_Msg( header, 'o', "Order Status Extended" )
{
}

/////////////////////////////////////////////////////////////////
ACI_Local_Parameter_Msg::ACI_Local_Parameter_Msg( ACI_Header header )
	: ACI_Msg( 'm', "Local Parameter" ),
	index(0),
	function(READ_PARAMETER_AREA),
	param_num(0),
	pcount(0),
	AGVId(0)
{
	for( int i=0; i<5; ++i )
		p[i] = 0;
}

///////////
ACI_Local_Parameter_Msg::ACI_Local_Parameter_Msg()
	: ACI_Msg( 'm', "Local Parameter" ),
	index(0),
	function(READ_PARAMETER_AREA),
	param_num(0),
	pcount(0),
	AGVId(0)
{
	for( int i=0; i<5; ++i )
		p[i] = 0;
}

///////////
void ACI_Local_Parameter_Msg::write( 
	ulapi_integer id )
{
	switch( function )
	{
	case READ_PARAMETER_AREA:
		// check for valid parameter values
		if( pcount < 1 || pcount > 5 )
			throw ACI_parameter_error( "Invalid number of parameters" );
		if( index < 0 || index >999 )
			throw ACI_parameter_error( "Invalid order queue index" );

		// compute message size parameters
		num_params = 3 + pcount;
		header.message_size = 4 + num_params;

		// write out the data
		header.write( id );
		ACI_write_short( id, msg_type );
		ACI_write_short( id, num_params );

		ACI_write_short( id, index );
		ACI_write_byte( id, function );
		for( int i=0; i<pcount; i++ )
		{
			if( p[i] < 0 || p[i] > 31 )
				throw ACI_parameter_error( "Invalid parameter number" );
			ACI_write_byte( id, (unsigned char)p[i] );
		}
		break;

	case SPONTANEOUS_UPDATE:
	case REQUESTED_PARAMETER_UPDATE:
		// insert local parameter

		// validate parameter values
		if( pcount < 1 || pcount > 5 )
			throw ACI_parameter_error( "Invalid number of parameters" );
		if( index < 0 || index >999 )
			throw ACI_parameter_error( "Invalid order queue index" );
		if( par_no < 0 || par_no > 31 )
			throw ACI_parameter_error( "Invalid parameter number" );

		// compute message size parameters
		num_params = 4 + 2*pcount;
		header.message_size = 4 + num_params;

		// write out the data
		header.write( id );
		ACI_write_short( id, msg_type );
		ACI_write_short( id, num_params );

		ACI_write_short( id, index );
		ACI_write_byte( id, function );
		ACI_write_byte( id, par_no );
		for( int i=0; i<pcount; ++i )
			ACI_write_short( id, p[i] );
		break;

	case DELETE_PARAMETER:
		// validate parameter values
		if( index < 0 || index >999 )
			throw ACI_parameter_error( "Invalid order queue index" );
		if( par_no < 0 || par_no > 31 )
			throw ACI_parameter_error( "Invalid parameter number" );

		// compute message size parameters
		num_params = 4;
		header.message_size = 4 + num_params;

		// write out the data
		header.write( id );
		ACI_write_short( id, msg_type );
		ACI_write_short( id, num_params );
		ACI_write_short( id, index );
		ACI_write_byte( id, (unsigned char)function );
		ACI_write_byte( id, par_no );
		break;

	case CHANGE_ORDER_PRIORITY:
		// validate parameter values
		if( index < 0 || index >999 )
			throw ACI_parameter_error( "Invalid order queue index" );
		if( prio < 0 || prio > 99 )
			throw ACI_parameter_error( "Invalid priority value" );

		// compute message size parameters
		num_params = 4;
		header.message_size = 4 + num_params;

		// write out the data
		header.write( id );
		ACI_write_short( id, msg_type );
		ACI_write_short( id, num_params );

		ACI_write_short( id, index );
		ACI_write_byte( id, function );
		ACI_write_byte( id, prio );
		break;

	case CONNECT_ALLOCATED_VEHICLE:
		// validate parameter values
		if( index < 0 || index >999 )
			throw ACI_parameter_error( "Invalid order queue index" );

		// compute message size parameters
		num_params = 4;
		header.message_size = 4 + num_params;

		ACI_write_short( id, index );
		ACI_write_byte( id, function );
		ACI_write_byte( id, AGVId );
		break;

	default: 
		throw ACI_parameter_error( "Invalid function value" );
		break;
	}
}

//////////////////////////////////////////////////////////////////
ACI_Parameter_Area_Contents_Msg::ACI_Parameter_Area_Contents_Msg( ACI_Header header )
	: ACI_Msg( header, 'w', "Area Contents" )
{
}

/////////
void ACI_Parameter_Area_Contents_Msg::read( ulapi_integer id )
{
	num_params = ACI_read_short( id );
	if( num_params != header.message_size - 4 )
		throw ACI_format_error( "Message size and parameter number do not agree" );
	if( num_params != 18 )
		throw ACI_format_error( "Parameter count is invalid" );

	index = ACI_read_short( id );
	if( !( (0 <= index && index <= 999 ) || index == 0xffff ) )
		throw ACI_format_error( "Invalid index" );
	no_par = ACI_read_byte( id );
	if( no_par != 5 )
		throw ACI_format_error( "Parameter number is not 5" );

	for( int i=0; i<5; ++i )
		p_no[i] = ACI_read_byte( id );
	for( int i=0; i<5; ++i )
		p_val[i] = ACI_read_short( id );
}

////////
void ACI_Parameter_Area_Contents_Msg::print()
{
	cout << "Parameter Area Contents: ";
	header.print();
	cout << ", msg_type = " << msg_type << " '" << (char)msg_type << "'";
	cout << ", num_params = " << num_params << ", index = " << index << ", no_par = " << no_par;
	for( int i=0; i<5; ++i )
	{
		if( p_no[i] != (short)0xff )
		{
			cout << ", p[" << p_no[i] << "] = " << p_val[i];
		}
	}

	cout << endl;
}

//////////////////////////////////////////////////////////////////
ACI_Global_Parameter_Command_Msg::ACI_Global_Parameter_Command_Msg( ACI_Header header )
	: ACI_Msg( header, 'g', "Global Parameter_Command" )
{
}

//////////////////////////////////////////////////////////////////
ACI_Global_Parameter_Status_Msg::ACI_Global_Parameter_Status_Msg
	( ACI_Header header )
	: ACI_Msg( header, 'p', "Global Parameter Status" )
{
}

/////////////////////////////////////////////////////////////////

ACI_OM_to_PLC_Msg::ACI_OM_to_PLC_Msg( ACI_Header header )
	: ACI_Msg( header, '>', "OM to PLC" ),
	res(0),
	carid(1),
	magic(0),
	c1(READ),
	s1(0),
	om1(0),
	plc1(0),
	c2(NONE),
	s2(0),
	om2(0),
	plc2(0), 
	par_count(0)
{
}

////////
ACI_OM_to_PLC_Msg::ACI_OM_to_PLC_Msg()
	: ACI_Msg( '>', "OM to PLC" ),
	res(0),
	carid(1),
	magic(0),
	c1(READ),
	s1(0),
	om1(0),
	plc1(0),
	par1(0),
	c2(NONE),
	s2(0),
	om2(0),
	plc2(0),
	par2(0),
	par_count(0)
{
}

/////////
void ACI_OM_to_PLC_Msg::write( ulapi_integer id )
{
	// compute correct num_params
	switch( c1 )
	{
	case READ:
	case WRITE:
		switch( c2 )
		{
		case READ:
		case WRITE:
			num_params = 16;
			header.message_size = 4 + num_params;

			// write out the data
			header.write( id );
			ACI_write_short( id, msg_type );
			ACI_write_short( id, num_params );

			ACI_write_byte( id, res );
			ACI_write_byte( id, carid );
			ACI_write_short( id, magic );
			ACI_write_byte( id, c1 );
			ACI_write_byte( id, s1 );
			ACI_write_byte( id, om1 );
			ACI_write_byte( id, plc1 );
			ACI_write_short( id, par1 );
			ACI_write_byte( id, c2 );
			ACI_write_byte( id, s2 );
			ACI_write_byte( id, om2 );
			ACI_write_byte( id, plc2 );
			ACI_write_short( id, par2 );
			break;
		case NONE:
			num_params = 10;
			header.message_size = 4 + num_params;

			// write out the data
			header.write( id );
			ACI_write_short( id, msg_type );
			ACI_write_short( id, num_params );

			ACI_write_byte( id, res );
			ACI_write_byte( id, carid );
			ACI_write_short( id, magic );
			ACI_write_byte( id, c1 );
			ACI_write_byte( id, s1 );
			ACI_write_byte( id, om1 );
			ACI_write_byte( id, plc1 ); // ord offset
			ACI_write_short( id, par1 );
			break;
		default:
			throw ACI_parameter_error( "Invalid c2" );
		}
		break;

	case MULTI_READ:
		num_params = 10;
		header.message_size = 4 + num_params;

		// write out the data
		header.write( id );
		ACI_write_short( id, msg_type );
		ACI_write_short( id, num_params );

		ACI_write_byte( id, res );
		ACI_write_byte( id, carid );
		ACI_write_short( id, magic );
		ACI_write_byte( id, c1 );
		ACI_write_byte( id, s1 );
		ACI_write_short( id, plc1 ); // byte offset
		ACI_write_byte( id, par_count ); // data byte count
		ACI_write_byte( id, 0 );  // spare
		break;
	case MULTI_WRITE:
		if( par_count < 0 || par_count > 80 )
			throw ACI_parameter_error( "Invalid parameter count" );
		num_params = 10 + 2*par_count;
		header.message_size = 4 + num_params;

		// write out the data
		header.write( id );
		ACI_write_short( id, msg_type );
		ACI_write_short( id, num_params );

		ACI_write_byte( id, res );
		ACI_write_byte( id, carid );
		ACI_write_short( id, magic );
		ACI_write_byte( id, c1 );
		ACI_write_byte( id, s1 );
		ACI_write_short( id, plc1 );
		ACI_write_byte( id, 2*par_count ); // data byte count
		ACI_write_byte( id, 0 );  // spare
		for( int i=0; i<par_count; ++i )
			ACI_write_byte( id, par[i] );
		break;
	default:
		throw ACI_parameter_error( "Invalid c1" );
	}
}

/////////////////////////////////////////////////////////////

ACI_PLC_to_OM_Msg::ACI_PLC_to_OM_Msg( ACI_Header header )
	: ACI_Msg( header, '<', "PLC to OM" ),
	res(0),
	carid(1),
	magic(0),
	c1(READ_ACK),
	s1(0),
	om1(0),
	plc1(0),
	par1(0),
	c2(READ_ACK),
	s2(0),
	om2(0),
	plc2(0),
	par2(0),
	plc(0),
	seq(0),
	last(0),
	num(0)
{
	for( int i=0; i<240; ++i )
		par[i] = 0;
}

//////
void ACI_PLC_to_OM_Msg::read( ulapi_integer id )
{
	num_params = ACI_read_short( id );
	if( num_params != header.message_size - 4 )
		throw ACI_format_error( "Message size and parameter number do not agree" );
	
	res = ACI_read_byte( id );
	carid = ACI_read_byte( id );
	magic = ACI_read_short( id );
	c1 = (Code)ACI_read_byte( id );
	s1 = ACI_read_byte( id );
	switch( c1 )
	{
	case READ_ACK:
	case WRITE_ACK:
	case READ_NAK:
	case WRITE_NAK:
		om1 = ACI_read_byte( id );
		plc1 = ACI_read_byte( id );
		par1 = ACI_read_short( id );

		if( num_params > 10 )
		{
			c2 = (Code)ACI_read_byte( id );
			s2 = ACI_read_byte( id );
			om2 = ACI_read_byte( id );
			plc2 = ACI_read_byte( id );
			par2 = ACI_read_short( id );
		}
		break;

	case MULTI_READ_ACK:
	case MULTI_WRITE_ACK:
	case MULTI_READ_NAK:
	case MULTI_WRITE_NAK:
		plc = ACI_read_short( id );
		seq = ACI_read_byte( id );
		last = ACI_read_byte( id );
		num = num_params - 10;

		if( num >= 240 )
			throw ACI_format_error( "num is too large" );
		for( int i=0; i<num; ++i )
			par[i] = ACI_read_byte( id );

		break;
	}

}

//////
void ACI_PLC_to_OM_Msg::print( Code c )
{
	switch( c1 )
	{ 
	case READ_ACK:         cout << "READ_ACK";  break;
	case WRITE_ACK:        cout << "WRITE_ACK";  break;
	case READ_NAK:         cout << "READ_NAK";  break;
	case WRITE_NAK:        cout << "WRITE_NAK";  break;
	case MULTI_READ_ACK:   cout << "MULTI_READ_ACK";  break;
	case MULTI_WRITE_ACK:  cout << "MULTI_WRITE_ACK";  break;
	case MULTI_READ_NAK:   cout << "MULTI_READ_NAK";  break;
	case MULTI_WRITE_NAK:  cout << "MULIT_WRITE_NAK";  break;
	}
}

//////
void ACI_PLC_to_OM_Msg::print()
{
	cout << "PLC to OM Message: ";
	header.print();
	cout << ", msg_type = " << msg_type << " '" << (char)msg_type << "'";
	cout << ", num_params = " << num_params << ", res = " << (int)res << ", carid = " 
		<< ", magic = " << magic << ", c1 = ";
	print( c1 );
	cout << ", s1 = " << s1;
	switch( c1 )
	{
	case READ_ACK: 
	case WRITE_ACK:
	case READ_NAK: 
	case WRITE_NAK:
		cout << ", om1 = " << om1 << ", plc1 = " << plc1 << ", par1 = " << par1;
		cout << ", c2 = ";
		print( c2 );
		cout << ", s2 = " << s2 << ", om2 = " << om2 << ", plc2 = " << plc2 << ", par2 = " << par2;
		break;

	case MULTI_READ_ACK: 
	case MULTI_WRITE_ACK:
	case MULTI_READ_NAK:
	case MULTI_WRITE_NAK:
		cout << ", plc = " << plc << ", seq = " << seq << ", last = " << last << endl;
		cout << "data = ";
		for( int i=0; i<num; ++i )
			cout << (int) par[i] << " ";
		break;
	}

	cout << endl;
}

//////////////////////////////////////////////////////////////////////////////////
ACI_Delete_Order_Msg::ACI_Delete_Order_Msg()
	: ACI_Msg( 'n', "Delete Order" ),
	index(0),
	carno(0)
{
}

/////
ACI_Delete_Order_Msg::ACI_Delete_Order_Msg( ACI_Header header)
	: ACI_Msg( header, 'n', "Delete Order" ),
	index(0),
	carno(0)
{
}

/////////
void ACI_Delete_Order_Msg::write( ulapi_integer id )
{
	if( carno == 0 )
	{
		// use short form
		num_params = 2;
	}
	else
	{
		// use long form
		num_params = 3;
	}
	header.message_size = num_params + 4;

	header.write( id );
	ACI_write_short( id, msg_type );
	ACI_write_short( id, num_params );

	ACI_write_short( id, index );
	if( carno != 0 )
		ACI_write_byte( id, carno );
}

/////////////////////////////////////////////////////////////////
ACI_Order_Status_Request_Msg::ACI_Order_Status_Request_Msg()
	: ACI_Msg( 'j', "Order Status" ),
	index(0),
	carno(0)
{
}

////////
ACI_Order_Status_Request_Msg::ACI_Order_Status_Request_Msg( ACI_Header header )
	: ACI_Msg( header, 'j', "Order Status" ),
	index(0),
	carno(0)
{
}

///////
void ACI_Order_Status_Request_Msg::write( ulapi_integer id )
{
	if( carno == 0 )
	{
		// short format
		num_params = 2;
	}
	else
	{
		// long format
		num_params = 3;
	}
	header.message_size = num_params + 4;

	header.write( id );
	ACI_write_short( id, msg_type );
	ACI_write_short( id, num_params );

	ACI_write_short( id, index );
	if( carno != 0 )
		ACI_write_byte( id, carno );
}

///////////////////////////////////////////////////////////
ACI_Order_Initiate_Msg::ACI_Order_Initiate_Msg()
	: ACI_Msg( 'q', "Order Intiate" ),
	trp_str(1),
	code(0),
	ikey(0),
	pri(0),
	p_count(0)
{
	for( int i=0; i<5; ++i )
		p[i] = 0;
}

/////////
ACI_Order_Initiate_Msg::ACI_Order_Initiate_Msg( ACI_Header _header )
	: ACI_Msg( header, 'q', "Order Intiate" ),
	trp_str(1),
	code(0),
	ikey(0),
	pri(0),
	p_count(0)

{
}

/////////
void ACI_Order_Initiate_Msg::write( ulapi_integer id )
{
	// validate parameters
	if( p_count < 0  ||  p_count > 4 )
		throw ACI_parameter_error( "p_count out of bounds" );

	if( code || ikey )
	{
		// long form
		num_params = 6 + 2*p_count;
	}
	else
	{
		// short form
		num_params = 2 + 2*p_count;
	}
	header.message_size = num_params + 4;

	header.write( id );
	ACI_write_short( id, msg_type );
	ACI_write_short( id, num_params );

	ACI_write_byte( id, trp_str );
	ACI_write_byte( id, pri );

	if( code || ikey )
	{
		ACI_write_short( id, code );
		ACI_write_short( id, ikey );
	}

	for( int i=0; i<p_count; ++i )
		ACI_write_short( id, p[i] );
}

//////////////////////////////////////////////////////////////////////
ACI_Order_Acknowledge_Msg::ACI_Order_Acknowledge_Msg( ACI_Header header )
	: ACI_Msg( header ,'b', "Order Acknowledge" ),
	index(0),
	trp_str(0),
	status(0),
	par_no(0),
	spare(0),
	ikey(0)
{
}

/////////
void ACI_Order_Acknowledge_Msg::read( ulapi_integer id )
{
	num_params = ACI_read_short( id );
	if( num_params != 5  &&  num_params != 8 )
		throw ACI_format_error( "Invalid number of parameters" );

	index = ACI_read_short( id );
	if( !( (0 <= index && index <= 999 ) || index == 0xffff ) )
		throw ACI_format_error( "Invalid index" );
	trp_str = ACI_read_byte( id );
	status = ACI_read_byte( id );
	par_no = ACI_read_byte( id );
	
	if( num_params == 8 )
	{
		spare =	ACI_read_byte( id );
		ikey = ACI_read_short( id );
	}
}

/////////
void ACI_Order_Acknowledge_Msg::print()
{
	cout << "Order Acknowledge Message: ";
	header.print();
	cout << ", msg_type = " << msg_type << " '" << (char)msg_type << "'";
	cout << ", num_params = " << num_params << ", index = " << index << ", transport_structure = " 
		<< (int)trp_str << ", status = " << (int)status << " = '" << status2str(status) << "', par_no = " << (int)par_no;

	if( num_params > 5 )
	{
		cout << ", ikey = " << ikey;
	}

	cout << endl;
}

/////////
const char* ACI_Order_Acknowledge_Msg::status2str( unsigned char status )
{
	switch( status )
	{
	case 0: return "order rejected";
	case 1: return "order acknowledge, input in queue";
	case 2: return "allocation of carrier";
	case 3: return "order finished";
	case 7: return "parameter acknowledge";
	case 8: return "invalid number of parameters";
	case 9: return "priority error";
	case 10: return "invalid sturcture";
	case 11: return "order buffer full";
	case 14: return "not activated index";
	case 15: return "parameter number too high for this";
	case 16: return "not allowed to update";
	case 17: return "fatal error, order execution stopped";
	case 18: return "parameter deleted";
	case 19: return "parameter value accepted";
	case 20: return "parameter not accepted";
	case 21: return "order has lost the carrier";
	case 22: return "carrier number error";
	case 23: return "imput release";
	case 24: return "invalid index number";
	case 25: return "cancel acknowledge";
	case 26: return "missing parameters";
	case 27: return "duplicated IKEY";
	case 28: return "invalid format code";
	case 35: return "change order instance priority acknowledge";
	case 37: return "connectoion of carrier-order";
	case 39: return "CONNECT OK, poffs=AGV id";
	case 40: return "CONNECT failed, poffs=AGV id";

	default: return "unknown status";
	}
}

////////////////////////////////////////////////////////////////////

ACI_Heartbeat_Msg::ACI_Heartbeat_Msg()
	: ACI_Msg( 0, "Heartbeat" )
{
}

///////
ACI_Heartbeat_Msg::ACI_Heartbeat_Msg( ACI_Header header )
	: ACI_Msg( header, 0, "Heartbeat" )
{
}

//////
void ACI_Heartbeat_Msg::read( ulapi_integer id )
{
}

//////
void ACI_Heartbeat_Msg::write( ulapi_integer id )
{
	header.write( id );
}

//////
void ACI_Heartbeat_Msg::print()
{
	switch( header.function_code )
	{
	case ACI_Header::DISCONNECT_LINK:
		cout << "Disconnect Link Message:";
		break;
	case ACI_Header::HEART_BEAT_POLL:
		cout << "Heartbeat Poll Message: ";
		break;
	case ACI_Header::HEART_BEAT_ACK:
		cout << "Heartbeat Ack Message: ";
		break;
	default:
		throw ACI_parameter_error( "Invalid function code" );
		break;
	}
	header.print();
}

///////////////////////////////////////////////////////////////////
ACI_Parameter_Request_Msg::ACI_Parameter_Request_Msg( ACI_Header header )
	: ACI_Msg( header, 'o', "Parameter Request" )
{
}

////////////////////////////////////////////////////////////////////
ACI_Msg* ACI_read_message( ulapi_integer id )
{
	// parse message
	ACI_Header header;
	int byte_count = 0;
	short msg_type;

	ACI_Msg *msg_ptr = 0;

	header.read( id );

	switch( header.function_code )
	{
	case ACI_Header::NORMAL_MESSAGE:
		msg_type = ACI_read_short( id );

		// validate message type value
		if( msg_type < 0x20  ||  0x7F < msg_type ) throw ACI_format_error( "Invalid message type" );

		if ( ACI_debug )
			cout << "Normal message, type = " << msg_type << " '" << (char)msg_type << "'" << endl;
		switch( msg_type )
		{
		case 'q':
			if( ACI_debug )
				cout << "Order Initiate Message" << endl;
			msg_ptr = new ACI_Order_Initiate_Msg( header );
			break;
		case 'b':
			if( ACI_debug )
				cout << "Order Acknowledge Message" << endl;
			msg_ptr = new ACI_Order_Acknowledge_Msg( header );
			break;
		case 'j':
			if( ACI_debug )
				cout << "Order Status Request Message" << endl;
			msg_ptr = new ACI_Order_Status_Request_Msg( header );
			break;
		case 's':
			if( ACI_debug )
				cout << "Order Status Message" << endl;
			msg_ptr = new ACI_Order_Status_Msg( header );
			break;
		case 'o':
			if( ACI_debug )
				cout << "Order Status Extended Message" << endl;
			msg_ptr = new ACI_Order_Status_Extended_Msg( header );
			break;
		case 'n':
			if( ACI_debug )
				cout << "Delete Order Message" << endl;
			msg_ptr = new ACI_Delete_Order_Msg( header );
			break;
		case 'm':
			if( ACI_debug )
				cout << "Local Parameter Message" << endl;
			msg_ptr = new ACI_Local_Parameter_Msg( header );
			break;
		case 'w':
			if( ACI_debug )
				cout << "Parameter Area Contents Message" << endl;
			msg_ptr = new ACI_Parameter_Area_Contents_Msg( header );
			break;
		case 'r':
			if( ACI_debug )
				cout << "Parameter Request Message" << endl;
			msg_ptr = new ACI_Parameter_Request_Msg( header );
			break;
		case 'g':
			if( ACI_debug )
				cout << "Global Parameter Command" << endl;
			msg_ptr = new ACI_Global_Parameter_Command_Msg( header );
			break;
		case 'p':
			if( ACI_debug )
				cout << "Global Parameter Status" << endl;
			msg_ptr = new ACI_Global_Parameter_Status_Msg( header );
			break;
		case '>':
			if( ACI_debug )
				cout << "OM to PLC Message" << endl;
			msg_ptr = new ACI_OM_to_PLC_Msg( header );
			break;
		case '<':
			if( ACI_debug )
				cout << "PLC to OM Message" << endl;
			msg_ptr = new ACI_PLC_to_OM_Msg( header );
			break;
		default:
			if( ACI_debug )
				cout << "Unrecognized message type = " << msg_type << " '" << (char)msg_type << 
				"', message_size = " << header.message_size << endl;
			//throw ACI_format_error( "Unknown message type" );
			msg_ptr = new ACI_Unknown_Msg( header );
			msg_ptr->msg_type = msg_type;
			break;
		}
		break;
	case ACI_Header::DISCONNECT_LINK:
	case ACI_Header::RESERVED:
	case ACI_Header::HEART_BEAT_POLL:
	case ACI_Header::HEART_BEAT_ACK:
		if( ACI_debug )
			cout << "Disconnect Link message" << endl;
		new ACI_Heartbeat_Msg( header );
		break;
	default:
		if( ACI_debug )
			cout << "Unrecognized function code = " << header.function_code << endl;
		throw ACI_format_error( "Illegal function code" );
		break;
	}

	if( msg_ptr == 0 )
	{
		if( ACI_debug )
			cout << "Message type not currently handled" << endl;
		throw ACI_format_error( "Message type not currently supported" );
	}

	msg_ptr->read( id );

	return msg_ptr;
}

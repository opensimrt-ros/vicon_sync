

#include "opensimrt_msgs/SetFileNameSrvRequest.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/service_client.h"
#include <iostream>
#include <sstream>
#include <istream>
#include <streambuf>

#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#pragma warning( push )
#pragma warning( disable : 4242 )
#include <boost/asio.hpp>
#pragma warning( pop )

#include <ros/ros.h>
#include <signal.h>
#include <opensimrt_msgs/SetFileNameSrv.h>
#include <std_srvs/Empty.h>
#include <regex>

void sig_handler(int signum){

	//Return type of the handler function should be void
	printf("\nInside handler function\n");
	kill(getpid(), SIGTERM);
}

struct MemBuf : std::streambuf
{
	MemBuf( char * Begin, char * End ) 
	{
		this->setg( Begin, Begin, End );
	}
};

std::string clear_path(std::string a)
{
	std::string exp = R"(\\)";
	std::regex re(exp);
	std::regex re2(R"([a-zA-Z]:)");
	a = std::regex_replace(a, re2, "");
	a = std::regex_replace(a, re, "/");
	return "/tmp/"+a;
}

int main( int argc, char **argv )
{
	//ros deals this the arguments itself, so we are going to need params to set the port
	//
	//
	ros::init(argc, argv, "vicon_broadcast_monitor");
	ros::NodeHandle nh("~");
	ros::NodeHandle n; //global nodehandle
	signal(SIGINT,sig_handler); // Register signal handler

	unsigned short Port = 1020;
	int port_port;

	nh.param<int>("port", port_port, 1030);

	bool play_sound_after_calibration = false;
	nh.param("play_sound_after_calibration",play_sound_after_calibration,false);

	bool save_insoles = false;
	nh.param("save_insoles",save_insoles,false);

	Port = (unsigned short)port_port;
	ROS_INFO_STREAM("Using port:" << Port);
	// The name and path are mandatory for both the start and stop signals.

	boost::asio::io_service Service;  
	boost::asio::ip::udp::socket Socket( Service );
	boost::system::error_code Error;

	if( Socket.open( boost::asio::ip::udp::v4(), Error ) ) 
	{
		std::cerr << "Failed to open: " << Error.message() << std::endl;
		return 2;
	}

	if( Socket.set_option( boost::asio::ip::udp::socket::reuse_address( true ), Error ) ) 
	{
		std::cerr << "Failed to set option: " << Error.message() << std::endl;
		return 3;
	}

	boost::asio::ip::udp::endpoint Endpoint( boost::asio::ip::address_v4::any(), Port );


	if( Socket.bind( Endpoint, Error ) ) 
	{
		std::cerr << "Failed to bind: " << Error.message() << std::endl;
		return 4;
	}

	char Buffer[ 1601 ];
	std::set< unsigned int > PacketIDs;


	//create all the service clients we want to call
	ros::ServiceClient calibrate_imus 	= n.serviceClient<std_srvs::Empty>("calibrate_imus");
	ros::ServiceClient calibrate_imus_play_sound 	= n.serviceClient<std_srvs::Empty>("play");
	ros::ServiceClient start_imu_logger	= n.serviceClient<std_srvs::Empty>("start");
	ros::ServiceClient stop_imu_logger	= n.serviceClient<std_srvs::Empty>("stop");
	ros::ServiceClient save_imu_sto		= n.serviceClient<std_srvs::Empty>("save_sto");
	ros::ServiceClient set_imu_name_and_path	= n.serviceClient<opensimrt_msgs::SetFileNameSrv>("set_name_and_path");
	ros::ServiceClient clear_imu_loggers	= n.serviceClient<std_srvs::Empty>("clear");

	ros::ServiceClient start_insole_logger			= n.serviceClient<std_srvs::Empty>("insole/start");
	ros::ServiceClient stop_insole_logger			= n.serviceClient<std_srvs::Empty>("insole/stop");
	ros::ServiceClient save_insole_sto			= n.serviceClient<std_srvs::Empty>("insole/save");
	ros::ServiceClient set_insole_name_and_path		= n.serviceClient<opensimrt_msgs::SetFileNameSrv>("insole/set_name_and_path");
	ros::ServiceClient clear_insole_loggers			= n.serviceClient<std_srvs::Empty>("insole/clear");

	std_srvs::Empty common_trigger;
	while( ros::ok() )
	{
		ROS_INFO_STREAM("spinOnce");
		ros::spinOnce(); //untested pattern here, maybe this will behave weirdly. I need this here because of the continues. aösp maybe rate limiting this node
		ROS_INFO_STREAM(" after spinOnce");

		std::size_t Read = Socket.receive( boost::asio::buffer( &Buffer, sizeof( Buffer ) - 1), 0 , Error );
		ROS_INFO_STREAM("received something");

		if( 0 == Read )
		{
			std::cerr << "Failed to receive: " << Error.message() << std::endl;
			return 4;
		}

		//parse part

		MemBuf MemoryBuffer( Buffer, Buffer + Read );

		std::istream Stream( &MemoryBuffer );

		boost::property_tree::ptree PropertyTree;

		try
		{
			boost::property_tree::xml_parser::read_xml( Stream, PropertyTree ); 
		}
		catch( boost::property_tree::ptree_error & i_rError )
		{
			std::cout << "XML Error " << i_rError.what() << std::endl;
			Buffer[ Read ] = 0;
			std::cout << Buffer << std::endl;
			continue;
		}

		auto Items = PropertyTree.get_child("");
		if( Items.begin() == Items.end() )
		{
			std::cout << "No Data" << std::endl;
			Buffer[ Read ] = 0;
			std::cout << Buffer << std::endl;
			continue;
		}

		const std::string Type = Items.begin()->first;

		const unsigned int PacketID = PropertyTree.get<unsigned int>( Type + ".PacketID.<xmlattr>.VALUE" );

		if( PacketIDs.find( PacketID ) != PacketIDs.end() )
		{
			std::cout << "Duplicate" << std::endl;
			continue;
		}

		PacketIDs.insert( PacketID );

		std::cout << "Type " << Type << std::endl;


		//		std::cout << "Name " << PropertyTree.get< std::string >( Type + ".Name.<xmlattr>.VALUE" ) << std::endl;
		std::string name_str =  PropertyTree.get< std::string >( Type + ".Name.<xmlattr>.VALUE" );
		ROS_INFO_STREAM("Name " << name_str);

		std::string path_str = PropertyTree.get< std::string >( Type + ".DatabasePath.<xmlattr>.VALUE" ) ;
		//		std::cout << "Path " << PropertyTree.get< std::string >( Type + ".DatabasePath.<xmlattr>.VALUE" ) << std::endl;

		path_str = clear_path(path_str);
		ROS_INFO_STREAM("Path (already reshaped): " << path_str);


		auto Timecode = PropertyTree.get_optional< std::string >( Type + ".TimeCode.<xmlattr>.VALUE" );
		if( Timecode ) 
		{
			std::cout << "Timecode " << Timecode << std::endl;
		}

		auto Description = PropertyTree.get_optional< std::string >( Type + ".Description.<xmlattr>.VALUE" );
		if( Description ) 
		{
			std::cout << "Description " << Description << std::endl;
		}

		auto Notes = PropertyTree.get_optional< std::string >( Type + ".Notes.<xmlattr>.VALUE" );
		if( Notes ) 
		{
			std::cout << "Notes " << Notes << std::endl;
		}

		std::cout << std::endl;


		//now we need to check if type is equal to start or stop
		//
		if (Type.compare("CaptureStart")==0)
		{
			ROS_INFO_STREAM("Detected start. like, start the imu loggers and whatever else");
			calibrate_imus.call(common_trigger);
			//need to set name and path as well with a service call.
			opensimrt_msgs::SetFileNameSrv my_name_and_path;
			ROS_INFO_STREAM("Name " << name_str);
			ROS_INFO_STREAM("path " << path_str);
			my_name_and_path.request.name = name_str;
			my_name_and_path.request.path = path_str;
			set_imu_name_and_path.call(my_name_and_path);
			if (save_insoles)
				set_insole_name_and_path.call(my_name_and_path);
			start_imu_logger.call(common_trigger);
			if (save_insoles)
				start_insole_logger.call(common_trigger);

			if (play_sound_after_calibration)
				calibrate_imus_play_sound.call(common_trigger);
			ROS_INFO_STREAM("can start recording!");

		}
		if (Type.compare("CaptureStop")==0)
		{
			ROS_INFO_STREAM("Detected stop. like, stop the imu loggers, save and clear the buffer");
			stop_imu_logger.call(common_trigger);
			if (save_insoles)
				stop_insole_logger.call(common_trigger);
			save_imu_sto.call(common_trigger);
			if (save_insoles)
				save_insole_sto.call(common_trigger);
			clear_imu_loggers.call(common_trigger);
			if (save_insoles)
				clear_insole_loggers.call(common_trigger);
		}

		if (Type.compare("CaptureComplete")==0)

		{
			ROS_INFO_STREAM("Detected complete. i probably should do something with info.");
		}


		auto Result = PropertyTree.get_optional< std::string >( Type + ".<xmlattr>.RESULT" );
		if( Result ) 
		{
			std::cout << "Result " << Result << std::endl;
		}



	}

}


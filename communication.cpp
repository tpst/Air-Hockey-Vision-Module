#include "visModule.h"

#include <iostream>
#include <string>

string puckTracker::constructMessage(Puck puck)
{
	ostringstream puckMessage;

	puck.x_err = 0;
	puck.y_err = 0;
	puck.v_err = 0;

	if(puck.velocity < 0.015) puck.velocity = 0, puck.flag = 0;

	puckMessage << 0 << " "; 
	puckMessage << (int)(puck.position.x/mmratio) << " ";
	puckMessage << (int)(puck.position.y/mmratio) << " ";
	puckMessage << (int)puck.velocity << " ";
	puckMessage << (int)puck.x_err << " ";
	puckMessage << (int)puck.y_err << " ";
	puckMessage << (int)puck.v_err << " ";
	puckMessage << (int)puck.flag << endl;



	string puckMsg = puckMessage.str();

	//cout << puckMsg << endl;
	return puckMsg;
}

string puckTracker::constructMessage(Prediction p)
{
	ostringstream predictMessage;
	
	predictMessage << 1 << " ";
	predictMessage << (int)(p.distance/mmratio) << " ";
	predictMessage << (int)p.angle << " ";
	predictMessage << (int)p.eta << " ";
	predictMessage << (int)p.velocity << " ";
	predictMessage << (int)p.d_err << " ";
	predictMessage << (int)p.a_err << " ";
	predictMessage << (int)p.t_err << " ";
	predictMessage << (int)p.v_err << endl;

	string predictMsg = predictMessage.str();
	
	cout << predictMsg << endl;

	return predictMsg;
}





//#include <ctime>
//#include <iostream>
//#include <string>
//#include <boost/asio.hpp>
//#include <opencv/cv.h>
//
//using boost::asio::ip::tcp;
//using namespace std;
//using namespace cv;
//
//#define VISION_PORT 5003
//
//std::string make_daytime_string()
//{
//  std::time_t now = std::time(0);
//  return std::ctime(&now);
//}
//
//int main()
//{
//  try
//  {
//
//	Point position = Point(128, 160);
//	double velocity = 2.5;
//	double x_err = 0, y_err = 0, v_err = 0;
//	bool flag = 1; 
//		
//	ostringstream message;
//
//	message << 0 << " ";
//	message << position.x << " ";
//	message << position.y << " ";
//	message << velocity << " ";
//	message << x_err << " ";
//	message << y_err << " ";
//	message << v_err << " ";
//	message << flag << " " << endl;
//
//	cout << message.str() << endl;
//
//	string msg = message.str();
//	// Any program that uses asio need to have at least one io_service object
//    boost::asio::io_service io_service;
//
//    // acceptor object needs to be created to listen for new connections
//    tcp::acceptor acceptor(io_service, tcp::endpoint(tcp::v4(), VISION_PORT)); // port 5003
//
//    for (;;)
//    {
//      // creates a socket
//      tcp::socket socket(io_service);
//
//      // wait and listen
//      acceptor.accept(socket);
//
//      // prepare message to send back to client
//      //std::string message = make_daytime_string();
//
//      boost::system::error_code ignored_error;
//
//      // writing the message for current time
//      boost::asio::write(socket, boost::asio::buffer(msg), ignored_error);
//    }
//  }
//  catch (std::exception& e)
//  {
//    std::cerr << e.what() << std::endl;
//  }
//
//  return 0;
//}
#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <string>
#include <thread>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>

using PointType = pcl::PointXYZRGB;

namespace beast = boost::beast;
namespace http = beast::http;
namespace websocket = beast::websocket;
namespace net = boost::asio;
using tcp = boost::asio::ip::tcp;

std::string buffer_to_string(beast::flat_buffer buffer) {
	return std::string(boost::asio::buffer_cast<char const *>(beast::buffers_front(buffer.data())), boost::asio::buffer_size(buffer.data()));
}

std::string PointCloud2Message(pcl::PointCloud<PointType>::Ptr cloud) {
	//FORMAT: {"type": "points", "points": [[0.0, 0.0, 0.0], [1.0,1.0,1.0]]}";
	std::ostringstream oss;
	oss << "{\"type\":\"points\",\"points\":[";
	bool isFirst = true;
	for (const auto &point : *cloud)
	{
		if (!isFirst) {
			oss << ",";
		} else {
			isFirst = false;
		}
		oss << "[" << point.x
				<< "," << point.y
				<< "," << point.z << "]";
	}
	oss << "]}";

	return oss.str();
}

void do_session(tcp::socket socket) {
	try {
		websocket::stream<tcp::socket> ws{std::move(socket)};

		ws.accept();

		pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);

		const std::string filename = "../data.pcd";
		if (pcl::io::loadPCDFile<PointType>(filename, *cloud) == -1)
		{
			const std::string error_message = "Couldn't read file " + filename + "\n";
			PCL_ERROR(error_message.c_str());
		}

		pcl::PointCloud<PointType>::Ptr cloud_send(new pcl::PointCloud<PointType>);
		std::vector<int> idx;
		pcl::removeNaNFromPointCloud(*cloud, *cloud_send, idx);
		std::cout << "SEND SIZE: " << cloud_send->size() << std::endl;

		while (true) {
			beast::flat_buffer buffer;
			ws.read(buffer);
			ws.text(ws.got_text());
			ws.write(net::buffer(PointCloud2Message(cloud_send)));
		}
	}
	catch (beast::system_error const &se)
	{
		if (se.code() != websocket::error::closed) {
			std::cerr << "Error: " << se.code().message() << std::endl;
		}
	}
	catch (std::exception const &e)
	{
		std::cerr << "Error: " << e.what() << std::endl;
	}
}

int main(int argc, char* argv[]) {
	try
	{
		if (argc != 3) {
			std::cerr << "Usage: server <address> <port>\n"
								<< "Example:\n"
								<< "./server 0.0.0.0 8080" << std::endl;
			return EXIT_FAILURE;
		}

		const auto address = net::ip::make_address(argv[1]);
		const auto port = static_cast<unsigned short>(std::atoi(argv[2]));

		net::io_context ioc{1};
		tcp::acceptor acceptor{ioc, {address, port}};
		while (true) {
			tcp::socket socket{ioc};
			acceptor.accept(socket);

			std::thread(&do_session, std::move(socket)).detach();
		}
	}
	catch (const std::exception &e)
	{
		std::cerr << "Error: " << e.what() << std::endl;
		return EXIT_FAILURE;
	}
}
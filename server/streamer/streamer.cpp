#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>

#include "../utils/streamer.hpp"

using PointType = pcl::PointXYZRGB;

namespace beast = boost::beast;
namespace http = beast::http;
namespace websocket = beast::websocket;
namespace net = boost::asio;
namespace fs = boost::filesystem;
using tcp = boost::asio::ip::tcp;

std::vector<boost::filesystem::path> get_files(boost::filesystem::path path) {
	std::vector<boost::filesystem::path> files;
	try
	{
		if (exists(path)) {
			if (is_regular_file(path)) {
				std::cout << path << " size is " << file_size(path) << '\n';
			} else if (is_directory(path)) {
				std::copy(fs::directory_iterator(path), fs::directory_iterator(), std::back_inserter(files));
			}
		}
	}
	catch (const fs::filesystem_error& ex) {
		std::cout << ex.what() << '\n';
	}

	std::sort(files.begin(), files.end());
	return files;
}

void do_session(tcp::socket socket) {
	try {
		Streamer<PointType> streamer(std::move(socket));

		pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);

		const fs::path path("../pcds");
		std::vector<fs::path> files = get_files(path);

		for (const auto& filename: files) {
			std::cout << filename << std::endl;

			if (pcl::io::loadPCDFile<PointType>(filename.string(), *cloud) == -1)
			{
				const std::string error_message = "Couldn't read file " + filename.string() + "\n";
				PCL_ERROR(error_message.c_str());
			}

			pcl::PointCloud<PointType>::Ptr cloud_send(new pcl::PointCloud<PointType>);
			std::vector<int> idx;
			pcl::removeNaNFromPointCloud(*cloud, *cloud_send, idx);

			streamer.setPointCloud(*cloud_send);
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
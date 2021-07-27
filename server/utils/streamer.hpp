#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

template <class PointT>
class Streamer {
	private:
		boost::beast::websocket::stream<boost::asio::ip::tcp::socket> ws_;
		typename pcl::PointCloud<PointT> cloud_;
		bool auto_send_ = true;
		std::string message_;

		void PointCloud2Message(pcl::PointCloud<PointT> cloud) {
			//FORMAT: {"type": "points", "points": [[0.0, 0.0, 0.0], [1.0,1.0,1.0]]}";
			std::ostringstream oss;
			oss << "{\"type\":\"points\",\"points\":[";
			bool isFirst = true;
			for (const auto &point : cloud)
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

			message_ = oss.str();
		}

	public:
		Streamer() = default;
		Streamer(boost::asio::ip::tcp::socket socket): ws_{std::move(socket)} {
			ws_.accept();
		}

		void setPointCloud(pcl::PointCloud<PointT> input_cloud) {
			cloud_ = input_cloud;
			PointCloud2Message(cloud_);

			if (auto_send_) {
				sendMessage();
			}
		}

		void sendMessage() {
			ws_.write(boost::asio::buffer(message_));
		}

		void setAutoSend(bool state) {
			auto_send_ = state;
		}
};

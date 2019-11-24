#include "rosbridge_ws_client.hpp"
#include <vector>

RosbridgeWsClient ros_client("ws://tegra-ubuntu:5800");

void advertiseServiceCallback(client* c, websocketpp::connection_hdl hdl, client::message_ptr msg)
{
	std::cout << "advertiseServiceCallback(): Message Received: " << msg->get_payload() << std::endl;

	nlohmann::json j = nlohmann::json::parse(msg->get_payload());
	j["success"] = j["args"]["data"].get<bool>();
	j["message"] = "from advertiseServiceCallback";

	ros_client.ServiceResponse(j["service"], j["id"], true, j);
}

void callServiceCallback(client* c, websocketpp::connection_hdl hdl, client::message_ptr msg) {
	std::cout << "serviceResponseCallback(): Message Received: " << msg->get_payload() << std::endl;
	c->close(hdl, websocketpp::close::status::normal, "Service Over");
}

void publisherThread(RosbridgeWsClient& rbc, const std::future<void>& futureObj)
{
    // Header
    int seq = 0;
	
	double x = 0.0;
	double y = 0.0;
	double z = 0.0;
	
	double value1 = 0.0;
	double value2 = 0.1;
	
	const int sz = 36;
	const double covarianceVal = 1e-10;
	std::vector<double> covariance (sz);
	std::fill(covariance.begin(), covariance.end(), covarianceVal);
	
	while (futureObj.wait_for(std::chrono::milliseconds(1)) == std::future_status::timeout)
	{
        using namespace std::chrono;
        
        auto now = system_clock::now();
        // Get current time with precision of milliseconds
        auto now_ns = time_point_cast<nanoseconds>(now);
        // sys_nanoseconds is type time_point<system_clock, nanoseconds>
        //using sys_nanoseconds = decltype(now);
        // Convert time_point to signed integral type
        //auto integral_duration = now.time_since_epoch().count();
        // Convert signed integral type to time_point
        //sys_nanoseconds dt{nanoseconds{integral_duration}};
        
		nlohmann::json j = {
			{"linear", {
				{"x", x},
				{"y", y},
				{"z", z}
			}},
			{"angular", {
				{"x", -x},
				{"y", -y},
				{"z", -z}
			}}
		};
		
		nlohmann::json odom = {
			{"header", {
			    {"seq", seq},
			    {"stamp", {
			        {"secs", 123},
			        {"nsecs", 456}
			    }},
			    {"frame_id", "odom_asdf"}
			}},
			{"child_frame_id", "base_link_asdf"},
			{"pose", {
			    {"pose", {
			        {"position", {
			            {"x", value1},
			            {"y", 1.2},
			            {"z", 3.4}
			        }},
			        {"orientation", {
			            {"x", value1},
			            {"y", 5.6},
			            {"z", 7.8},
			            {"w", 9.0}
			        }}
			    }},
			    {"covariance", covariance}
			}},
			{"twist", {
			    {"twist", {
			        {"linear", {
			            {"x", value2},
			            {"y", 3.1},
			            {"z", 4.5}
			        }},
			        {"angular", {
			            {"x", value2},
			            {"y", 5.9},
			            {"z", 2.6}
			        }}
			    }},
			    {"covariance", covariance}
			}}
		};
		
		/*
        auto asdf = odom.get<std::unordered_map<std::string, json>>();
        
        for (auto i : asdf) {
         std::cout << i.first << ": " << i.second << '\n';
        }
        */
        
		rbc.Publish("/frc_diff_drive_controller/odom", odom);
		// rbc.Publish("/frc_diff_drive_controller/cmd_vel", j);
		
		seq++;
		value1 = value1 + 0.1;
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	}

	std::cout << "publisherThread stops()" << std::endl;
}

void subscriberCallback(client* c, websocketpp::connection_hdl hdl, client::message_ptr msg)
{
	std::cout << "subscriberCallback(): Message Received: " << msg->get_payload() << std::endl;
}

int main() {
	//ros_client.AdvertiseService("service_advertiser", "/zservice", "std_srvs/SetBool", advertiseServiceCallback);
	//ros_client.CallService("/zService", callServiceCallback);
	ros_client.Advertise("roborio", "/frc_diff_drive_controller/cmd_vel", "geometry_msgs/Twist");
	ros_client.Subscribe("client_sub_test", "/frc_diff_drive_controller/cmd_vel", subscriberCallback);
	ros_client.Advertise("roborio", "/frc_diff_drive_controller/odom", "nav_msgs/Odometry");
	//std::this_thread::sleep_for(std::chrono::seconds(100));

	{
		// Create a std::promise object
		std::promise<void> exitSignal;

		// Fetch std::future object associated with promise
		std::future<void> futureObj = exitSignal.get_future();

		// Starting Thread & move the future object in lambda function by reference
		std::thread th(&publisherThread, std::ref(ros_client), std::cref(futureObj));

		// Wait for 10 sec
		std::this_thread::sleep_for(std::chrono::seconds(1000000));

		std::cout << "Asking publisherThread to Stop" << std::endl;

		// Set the value in promise
		exitSignal.set_value();

		// Wait for thread to join
		th.join();
	}
	return 0;
}


#define ASIO_STANDALONE
#define _WEBSOCKETPP_CPP11_MEMORY_
#define _WEBSOCKETPP_CPP11_TYPE_TRAITS_
#define _WEBSOCKETPP_CPP11_RANDOM_DEVICE_

#include <websocketpp/config/asio_no_tls_client.hpp>
#include <websocketpp/client.hpp>

#include <websocketpp/common/thread.hpp>
#include <websocketpp/common/memory.hpp>

#include <cstdlib>
#include <iostream>
#include <map>
#include <string>
#include <sstream>
#include <json/json.hpp>

typedef websocketpp::client<websocketpp::config::asio_client> client;

class connection_metadata {
public:
	typedef websocketpp::lib::shared_ptr<connection_metadata> ptr;

	connection_metadata(std::string id, websocketpp::connection_hdl hdl, std::string uri)
		: m_id(id)
		, m_hdl(hdl)
		, m_status("Connecting")
		, m_uri(uri)
		, m_server("N/A")
	{}

	void on_open(client* c, websocketpp::connection_hdl hdl) {
		m_status = "Open";

		client::connection_ptr con = c->get_con_from_hdl(hdl);
		m_server = con->get_response_header("Server");
	}

	void on_fail(client* c, websocketpp::connection_hdl hdl) {
		m_status = "Failed";

		client::connection_ptr con = c->get_con_from_hdl(hdl);
		m_server = con->get_response_header("Server");
		m_error_reason = con->get_ec().message();
	}

	void on_close(client* c, websocketpp::connection_hdl hdl) {
		m_status = "Closed";
		client::connection_ptr con = c->get_con_from_hdl(hdl);
		std::stringstream s;
		s << "close code: " << con->get_remote_close_code() << " ("
			<< websocketpp::close::status::get_string(con->get_remote_close_code())
			<< "), close reason: " << con->get_remote_close_reason();
		m_error_reason = s.str();
	}

	void on_message(websocketpp::connection_hdl, client::message_ptr msg) {
		if (msg->get_opcode() == websocketpp::frame::opcode::text) {
			m_messages.push_back(msg->get_payload());
		}
		else {
			m_messages.push_back(websocketpp::utility::to_hex(msg->get_payload()));
		}
	}

	websocketpp::connection_hdl get_hdl() const {
		return m_hdl;
	}

	std::string get_id() const {
		return m_id;
	}

	std::string get_status() const {
		return m_status;
	}

	std::vector<std::string> getMessages() {
		return m_messages;
	}

private:
	std::string m_id;
	websocketpp::connection_hdl m_hdl;
	std::string m_status;
	std::string m_uri;
	std::string m_server;
	std::string m_error_reason;
	std::vector<std::string> m_messages;
};

class websocket_endpoint {
public:
	websocket_endpoint() {
		m_endpoint.clear_access_channels(websocketpp::log::alevel::all);
		m_endpoint.clear_error_channels(websocketpp::log::elevel::all);

		m_endpoint.init_asio();
		m_endpoint.start_perpetual();

		m_thread = websocketpp::lib::make_shared<websocketpp::lib::thread>(&client::run, &m_endpoint);
	}

	~websocket_endpoint() {
		m_endpoint.stop_perpetual();

		for (con_list::const_iterator it = m_connection_list.begin(); it != m_connection_list.end(); ++it) {
			if (it->second->get_status() != "Open") {
				// Only close open connections
				continue;
			}

			std::cout << "Closing connection " << it->second->get_id() << std::endl;

			websocketpp::lib::error_code ec;
			m_endpoint.close(it->second->get_hdl(), websocketpp::close::status::going_away, "", ec);
			if (ec) {
				std::cout << "Error closing connection " << it->second->get_id() << ": "
					<< ec.message() << std::endl;
			}
		}

		m_thread->join();
	}

	void connect(std::string const& uri, 
				 std::string id,
				 const std::function<void(client * c, websocketpp::connection_hdl hdl)>& open_handler,
				 const std::function<void(client * c, websocketpp::connection_hdl hdl)>& fail_handler,
				 const std::function<void(client * c, websocketpp::connection_hdl hdl)>& close_handler,
				 const std::function<void(client * c, websocketpp::connection_hdl hdl, client::message_ptr msg)>& msg_handler
				 ) {
		websocketpp::lib::error_code ec;

		client::connection_ptr con = m_endpoint.get_connection(uri, ec);

		if (ec) {
			std::cout << "Connect initialization error: " << ec.message() << std::endl;
		}

		connection_metadata::ptr metadata_ptr = websocketpp::lib::make_shared<connection_metadata>(id, con->get_handle(), uri);
		m_connection_list[id] = metadata_ptr;

		if (open_handler != nullptr) {
			con->set_open_handler(std::bind(
				open_handler,
				&m_endpoint,
				std::placeholders::_1
			));
		}
		else {
			con->set_open_handler(websocketpp::lib::bind(
				&connection_metadata::on_open,
				metadata_ptr,
				&m_endpoint,
				websocketpp::lib::placeholders::_1
			));
		}

		if (fail_handler != nullptr) {
			con->set_fail_handler(std::bind(
				fail_handler,
				&m_endpoint,
				std::placeholders::_1
			));
		}
		else {
			con->set_fail_handler(websocketpp::lib::bind(
				&connection_metadata::on_fail,
				metadata_ptr,
				&m_endpoint,
				websocketpp::lib::placeholders::_1
			));
		}

		if (close_handler != nullptr) {
			con->set_close_handler(std::bind(
				close_handler,
				&m_endpoint,
				std::placeholders::_1
			));
		}
		else {
			con->set_close_handler(websocketpp::lib::bind(
				&connection_metadata::on_close,
				metadata_ptr,
				&m_endpoint,
				websocketpp::lib::placeholders::_1
			));
		}

		if (msg_handler != nullptr) {
			con->set_message_handler(std::bind(
				msg_handler,
				&m_endpoint,
				std::placeholders::_1,
				std::placeholders::_2
			));
		}
		else {
			con->set_message_handler(websocketpp::lib::bind(
				&connection_metadata::on_message,
				metadata_ptr,
				websocketpp::lib::placeholders::_1,
				websocketpp::lib::placeholders::_2
			));
		}

		m_endpoint.connect(con);
	}

	void close(std::string id, websocketpp::close::status::value code, std::string reason) {
		websocketpp::lib::error_code ec;

		con_list::iterator metadata_it = m_connection_list.find(id);
		if (metadata_it == m_connection_list.end()) {
			std::cout << "No connection found with id " << id << std::endl;
			return;
		}

		m_endpoint.close(metadata_it->second->get_hdl(), code, reason, ec);
		if (ec) {
			std::cout << "Error initiating close: " << ec.message() << std::endl;
		}
	}

	void send(std::string id, std::string message) {
		websocketpp::lib::error_code ec;

		con_list::iterator metadata_it = m_connection_list.find(id);
		if (metadata_it == m_connection_list.end()) {
			std::cout << "No connection found with id " << id << std::endl;
			return;
		}

		m_endpoint.send(metadata_it->second->get_hdl(), message, websocketpp::frame::opcode::text, ec);
		if (ec) {
			std::cout << "Error sending message: " << ec.message() << std::endl;
			return;
		}
	}

	connection_metadata::ptr get_metadata(std::string id) const {
		con_list::const_iterator metadata_it = m_connection_list.find(id);
		if (metadata_it == m_connection_list.end()) {
			return connection_metadata::ptr();
		}
		else {
			return metadata_it->second;
		}
	}
private:
	typedef std::map<std::string, connection_metadata::ptr> con_list;

	client m_endpoint;
	websocketpp::lib::shared_ptr<websocketpp::lib::thread> m_thread;

	con_list m_connection_list;
};
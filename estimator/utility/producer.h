#pragma once

#include <iostream>
#include <exception>
#include <string>
#include <sstream>
#include <thread>
#include <mutex>
#include <queue>
#include <chrono>

#include <json/json.h>
#include <opencv2/opencv.hpp>
#include <rabbitmq-c/amqp.h>
#include <amqp_tcp_socket.h>

static std::exception_ptr ex_ptr = nullptr;

class RabbitMQProducer
{
public:
	RabbitMQProducer() 
	{
	}

	RabbitMQProducer(std::string producer_name, std::string hostname = "localhost", int port = 5672, std::string queue_name = "my_queue", std::string exchange_name = "my_exchange", std::string exchange_type = "direct", std::string routing_key = "my_key")
	{
		this->name = producer_name;
		this->hostname = hostname;
		this->port = port;
		this->queue_name = queue_name;
		this->exchange_name = exchange_name;
		this->exchange_type = exchange_type;
		this->routing_key = routing_key;
	}

	RabbitMQProducer &operator=(const RabbitMQProducer &other)
	{
		this->name = other.name;
		this->hostname = other.hostname;
		this->port = other.port;
		this->queue_name = other.queue_name;
		this->exchange_name = other.exchange_name;
		this->exchange_type = other.exchange_type;
		this->routing_key = other.routing_key;
		return *this;
	}

	void credential(std::string username, std::string password)
	{
		this->username = username;
		this->password = password;
	}

	void run()
	{
		producer_thread = std::thread([this]() {
			try
			{
				std::cout << " [" << name.c_str() << "] Connecting to RabbitMQ..." << std::endl;
				amqp_connection_state_t connection = amqp_new_connection();
				amqp_socket_t* socket = amqp_tcp_socket_new(connection);
				if (!socket) throw std::exception("Creating TCP socket");

				handle_response(amqp_socket_open(socket, hostname.c_str(), port), "Opening TCP socket");
				handle_amqp_response(amqp_login(connection, "/", 0, 131072, 0, AMQP_SASL_METHOD_PLAIN, username.c_str(), password.c_str()), "Logging in");

				amqp_channel_open(connection, 1);
				handle_amqp_response(amqp_get_rpc_reply(connection), "Opening channel");

				amqp_bytes_t amqp_queue_name = amqp_cstring_bytes(queue_name.c_str());
				amqp_queue_declare_ok_t* queue_status = amqp_queue_declare(connection, 1, amqp_queue_name, 0, 0, 0, 1, amqp_empty_table);
				handle_amqp_response(amqp_get_rpc_reply(connection), "Declaring queue");

				amqp_bytes_t amqp_exchange_name = amqp_cstring_bytes(exchange_name.c_str());
				amqp_bytes_t amqp_exchange_type = amqp_cstring_bytes(exchange_type.c_str());
				amqp_exchange_declare_ok_t* exchange_status = amqp_exchange_declare(connection, 1, amqp_exchange_name, amqp_exchange_type, 0, 0, 0, 0, amqp_empty_table);
				handle_amqp_response(amqp_get_rpc_reply(connection), "Declaring exchange");

				amqp_bytes_t amqp_routing_key = amqp_cstring_bytes(routing_key.c_str());
				amqp_queue_bind(connection, 1, amqp_queue_name, amqp_exchange_name, amqp_routing_key, amqp_empty_table);
				handle_amqp_response(amqp_get_rpc_reply(connection), "Binding queue");

				std::vector<unsigned char> msg;
				while (true)
				{
					{
						std::unique_lock<std::mutex> lock(producer_mutex);

						while (msg_queue.size() == 0 && !do_exit)
						{
							producer_cv.wait(lock);
						}

						if (do_exit)
						{
							std::cout << " [" << name.c_str() << "] Received exit flag" << std::endl;
							break;
						}

						msg = msg_queue.front();
						msg_queue.pop();

						lock.unlock();
					}

					amqp_bytes_t message_bytes;
					message_bytes.len = msg.size();
					message_bytes.bytes = &(msg[0]);

					int rc = amqp_basic_publish(connection, 1, amqp_exchange_name, amqp_routing_key, 1, 0, NULL, message_bytes);
					handle_response(rc, "Sending Message");
				}

				std::cout << " [" << name.c_str() << "] Closing down connections..." << std::endl;
				handle_amqp_response(amqp_channel_close(connection, 1, AMQP_REPLY_SUCCESS), "Closing channel");
				handle_amqp_response(amqp_connection_close(connection, AMQP_REPLY_SUCCESS), "Closing connection");
				handle_response(amqp_destroy_connection(connection), "Ending connection");
			}
			catch (const std::exception& x)
			{
				std::cout << " [" << name.c_str() << "] A major exception has occurred! " << x.what() << std::endl;
				ex_ptr = std::current_exception();
			}
			catch (...)
			{
				std::cout << " [" << name.c_str() << "] A major exception has occurred! " << std::endl;
				ex_ptr = std::current_exception();
			}

			std::cout << " [" << name.c_str() << "] Finished" << std::endl;
		});
	}

	void send(const std::vector<unsigned char>& msg)
	{
		using namespace std;
		if (ex_ptr) rethrow_exception(ex_ptr);

		cout << " [" << name.c_str() << "] Queuing a binary message of size " << msg.size() << endl;
		{
			lock_guard<mutex> guard(producer_mutex);
			msg_queue.push(msg);
		}

		producer_cv.notify_one();
	}

	void send(const std::string& msg)
	{
		using namespace std;
		if (ex_ptr) rethrow_exception(ex_ptr);

		vector<unsigned char> binary_msg;
		copy(msg.begin(), msg.end(), back_inserter(binary_msg));

		cout << " [" << name.c_str() << "] Queuing a string message of size " << msg.length() << " : " << msg.c_str() << endl;
		{
			lock_guard<mutex> guard(producer_mutex);
			msg_queue.push(binary_msg);
		}

		producer_cv.notify_one();
	}

	void send(const cv::Mat& img, std::string encoding_format = ".png")
	{
		using namespace std;
		if (ex_ptr) rethrow_exception(ex_ptr);
		if (img.empty()) throw exception("Error loading image!");

		vector<unsigned char> img_buffer;
		bool ok = cv::imencode(encoding_format, img, img_buffer);
		if (!ok) throw exception("Error encoding image!");

		cout << " [" << name.c_str() << "] Queuing an image of size: " << img.cols << "x" << img.rows << endl;

		{
			lock_guard<mutex> guard(producer_mutex);
			msg_queue.push(img_buffer);
		}

		producer_cv.notify_one();
	}

	void send_EOT()
	{
		using namespace std;
		if (ex_ptr) rethrow_exception(ex_ptr);

		cout << " [" << name.c_str() << "] Queuing EOT" << endl;

		{
			lock_guard<mutex> guard(producer_mutex);
			vector<uchar> msg = { 4 };
			msg_queue.push(msg);
		}

		producer_cv.notify_one();
	}

	std::thread& exit()
	{
		using namespace std;
		if (ex_ptr) rethrow_exception(ex_ptr);

		cout << " [" << name.c_str() << "] Setting exit flag & notifying" << endl;

		{
			lock_guard<mutex> guard(producer_mutex);
			do_exit = true;
		}

		producer_cv.notify_all();
		return producer_thread;
	}

	void flush()
	{
		using namespace std;
		if (ex_ptr) rethrow_exception(ex_ptr);

		cout << " [" << name.c_str() << "] Flushing queue" << endl;

		bool queue_is_full = true;
		while (queue_is_full)
		{
			{
				lock_guard<mutex> guard(producer_mutex);
				if (msg_queue.size() == 0)
					queue_is_full = false;
			}
			if (queue_is_full)
				this_thread::sleep_for(100ms);

			if (ex_ptr)
				rethrow_exception(ex_ptr);
		}
	}

	std::thread& flush_and_exit()
	{
		flush();
		return exit();
	}
private:

	void handle_amqp_response(amqp_rpc_reply_t x, std::string context)
	{
		using namespace std;

		string err;

		switch (x.reply_type) {
		case AMQP_RESPONSE_NORMAL:
			return;

		case AMQP_RESPONSE_NONE:
			err = context + ": missing RPC reply type!";
			break;

		case AMQP_RESPONSE_LIBRARY_EXCEPTION:
			err = context + ": " + amqp_error_string2(x.library_error);
			break;

		case AMQP_RESPONSE_SERVER_EXCEPTION:
			switch (x.reply.id) {
			case AMQP_CONNECTION_CLOSE_METHOD: {
				amqp_connection_close_t* m = (amqp_connection_close_t*)x.reply.decoded;
				stringstream strm;
				strm << context << ": server connection error " << m->reply_code << ", message: ";
				for (int k = 0; k < m->reply_text.len; ++k)
					strm << static_cast<unsigned char*>(m->reply_text.bytes)[k];
				err = strm.str();
				break;
			}
			case AMQP_CHANNEL_CLOSE_METHOD: {
				amqp_channel_close_t* m = (amqp_channel_close_t*)x.reply.decoded;
				stringstream strm;
				strm << context << ": server channel error " << m->reply_code << ", message: ";
				for (int k = 0; k < m->reply_text.len; ++k)
					strm << static_cast<unsigned char*>(m->reply_text.bytes)[k];
				err = strm.str();
				break;
			}
			default:
				err = context + ": unknown server error, method id " + to_string(static_cast<int>(x.reply.id));
				break;
			}
			break;
		}

		throw exception(err.c_str());
	}

	void handle_response(int rc, std::string context)
	{
		if (rc < 0)
			throw std::exception(context.c_str());
	}

	int port;
	std::string name;
	std::string username;
	std::string password;
	std::string hostname;
	std::string queue_name;
	std::string exchange_name;
	std::string exchange_type;
	std::string routing_key;

	std::queue<std::vector<unsigned char>> msg_queue;
	std::thread producer_thread;
	std::mutex producer_mutex;
	std::condition_variable producer_cv;
	bool do_exit = false;
};
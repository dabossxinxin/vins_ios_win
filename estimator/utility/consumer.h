#pragma once

#include <iostream>
#include <exception>
#include <string>
#include <sstream>
#include <thread>
#include <mutex>
#include <queue>
#include <time.h>

#include <json/json.h>
#include <opencv2/opencv.hpp>
#include <rabbitmq-c/amqp.h>
#include <amqp_tcp_socket.h>

static std::exception_ptr ex_ptr = nullptr;

typedef struct timeval {
	long tv_sec;
	long tv_usec;
} TIMEVAL, *PTIMEVAL, *LPTIMEVAL;

class EOT : public std::exception
{
public:
	EOT() : exception("EOT") {}
};

class RabbitMQConsumer
{
public:

	RabbitMQConsumer()
	{
	}

	RabbitMQConsumer(std::string producer_name, std::string hostname = "localhost", int port = 5672, std::string queue_name = "my_queue", std::string exchange_name = "my_exchange", std::string exchange_type = "direct", std::string routing_key = "my_key")
	{
		this->name = producer_name;
		this->hostname = hostname;
		this->port = port;
		this->queue_name = queue_name;
		this->exchange_name = exchange_name;
		this->exchange_type = exchange_type;
		this->routing_key = routing_key;
	}

	RabbitMQConsumer &operator=(const RabbitMQConsumer &other)
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
		using namespace std;

		consumer_thread = thread([this]() {
			try
			{
				cout << " [" << name.c_str() << "] Connecting to RabbitMQ..." << endl;
				amqp_connection_state_t connection = amqp_new_connection();
				amqp_socket_t* socket = amqp_tcp_socket_new(connection);
				if (!socket) throw exception("Creating TCP socket");

				handle_response(amqp_socket_open(socket, hostname.c_str(), port), "Opening TCP socket");
				handle_amqp_response(amqp_login(connection, "/", 0, 131072, 0, AMQP_SASL_METHOD_PLAIN, username, password), "Logging in");

				amqp_channel_open(connection, 1);
				handle_amqp_response(amqp_get_rpc_reply(connection), "Opening channel");

				amqp_bytes_t amqp_queue_name = amqp_cstring_bytes(queue_name.c_str());
				amqp_queue_declare_ok_t* r = amqp_queue_declare(connection, 1, amqp_queue_name, 0, 0, 0, 1, amqp_empty_table);
				handle_amqp_response(amqp_get_rpc_reply(connection), "Declaring queue");

				amqp_basic_consume(connection, 1, amqp_queue_name, amqp_empty_bytes, 0, 1, 0, amqp_empty_table);
				handle_amqp_response(amqp_get_rpc_reply(connection), "Consuming");

				struct timeval timeout;
				timeout.tv_sec = 1;
				timeout.tv_usec = 0;

				cout << " [" << name.c_str() << "] Waiting for messages..." << endl;
				while (true)
				{
					amqp_maybe_release_buffers(connection);
					amqp_envelope_t envelope;
					amqp_rpc_reply_t res = amqp_consume_message(connection, &envelope, &timeout, 0);

					if (res.reply_type != AMQP_RESPONSE_NORMAL &&
						!(res.reply_type == AMQP_RESPONSE_LIBRARY_EXCEPTION && res.library_error == AMQP_STATUS_TIMEOUT))
						handle_amqp_response(res, "Consuming messages");

					{
						lock_guard<mutex> guard(consumer_mutex);

						if (do_exit)
						{
							cout << " [" << name.c_str() << "] Received exit flag" << endl;
							break;
						}
					}

					if (res.reply_type == AMQP_RESPONSE_NORMAL)
					{
						// 打印被消费的内容
						if (0) {
							cout << " [" << name.c_str() << "] Received message #" << static_cast<unsigned>(envelope.delivery_tag)
								<< " of length " << envelope.message.body.len;
							if (envelope.message.properties._flags & AMQP_BASIC_CONTENT_TYPE_FLAG)
							{
								cout << " Content-type: ";
								for (size_t k = 0; k < envelope.message.properties.content_type.len; ++k)
									cout << (static_cast<uchar*>(envelope.message.properties.content_type.bytes))[k];
							}
							cout << endl;
						}

						vector<uchar> msg(envelope.message.body.len);
						for (size_t k = 0; k < envelope.message.body.len; ++k)
							msg[k] = (static_cast<uchar*>(envelope.message.body.bytes))[k];

						amqp_destroy_envelope(&envelope);

						{
							lock_guard<mutex> guard(consumer_mutex);
							msg_queue.push(msg);
						}

						consumer_cv.notify_one();
					}
				}

				cout << " [" << name.c_str() << "] Closing down connections..." << endl;
				handle_amqp_response(amqp_channel_close(connection, 1, AMQP_REPLY_SUCCESS), "Closing channel");
				handle_amqp_response(amqp_connection_close(connection, AMQP_REPLY_SUCCESS), "Closing connection");
				handle_response(amqp_destroy_connection(connection), "Ending connection");
			}
			catch (const exception& x)
			{
				cout << " [" << name.c_str() << "] A major exception has occurred! " << x.what() << endl;
				ex_ptr = current_exception();
			}
			catch (...)
			{
				cout << " [" << name.c_str() << "] A major exception has occurred! " << endl;
				ex_ptr = current_exception();
			}

			cout << " [" << name.c_str() << "] Finished" << endl;
		});
	}


	void receive(std::vector<uchar>& msg)
	{
		using namespace std;
		if (ex_ptr) rethrow_exception(ex_ptr);

		{
			unique_lock<mutex> lock(consumer_mutex);

			while (msg_queue.size() == 0 && !do_exit)
			{
				consumer_cv.wait(lock);
			}

			msg = msg_queue.front();
			msg_queue.pop();

			lock.unlock();
		}

		if (msg.size() == 1 && msg[0] == 4)
		{
			cout << " [" << name.c_str() << "] Received EOT" << endl;
			throw EOT();
		}
	}

	std::string receive()
	{
		using namespace std;
		vector<uchar> msg;
		receive(msg);

		string str(msg.begin(), msg.end());
		return str;
	}


	void receive(cv::Mat& img)
	{
		using namespace std;
		vector<uchar> msg;
		receive(msg);

		img = cv::imdecode(cv::Mat(msg), cv::IMREAD_UNCHANGED);
	}


	std::thread& exit()
	{
		using namespace std;
		if (ex_ptr) rethrow_exception(ex_ptr);

		cout << " [" << name.c_str() << "] Setting exit flag & notifying" << endl;

		{
			lock_guard<mutex> guard(consumer_mutex);
			do_exit = true;
		}

		consumer_cv.notify_all();
		return consumer_thread;
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
	std::string hostname;
	std::string username;
	std::string password;
	std::string queue_name;
	std::string exchange_name;
	std::string exchange_type;
	std::string routing_key;

	std::queue<std::vector<uchar>> msg_queue;
	std::thread consumer_thread;
	std::mutex consumer_mutex;
	std::condition_variable consumer_cv;
	bool do_exit = false;
};
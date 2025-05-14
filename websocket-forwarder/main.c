#include <libwebsockets.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define eprintf(...) fprintf(stderr, __VA_ARGS__)

#define KGRN "\033[0;32;32m"
#define KCYN "\033[0;36m"
#define KRED "\033[0;32;31m"
#define KYEL "\033[1;33m"
#define KMAG "\033[0;35m"
#define KBLU "\033[0;32;34m"
#define KCYN_L "\033[1;36m"
#define RESET "\033[0m"

static int destroy_flag = 0;

static void INT_HANDLER(int signo) { destroy_flag = 1; }

static int ws_service_callback(
	struct lws *wsi,
	enum lws_callback_reasons reason,
	void *user,
	void *in,
	size_t len
) {
	switch (reason) {
	case LWS_CALLBACK_ESTABLISHED:
		eprintf(KYEL "[Main Service] Connection established\n" RESET);
		break;

	//* If receive a data from client*/
	case LWS_CALLBACK_RECEIVE:
		eprintf(
			KCYN_L "[Main Service] Server recvived:%.*s\n" RESET,
			(int)len,
			(char *)in
		);

		//* echo back to client*/
		printf("%.*s", (int)len, (char *)in);
		break;
	case LWS_CALLBACK_CLOSED:
		eprintf(KYEL "[Main Service] Client close.\n" RESET);
		break;

	default: break;
	}

	return 0;
}

struct per_session_data {
	int fd;
};

int main(void) {
	// server url will usd port 5000
	int port = 5000;
	const char *interface = NULL;
	struct lws_context_creation_info info;
	struct lws_protocols protocol;
	struct lws_context *context;
	// Not using ssl
	const char *cert_path = NULL;
	const char *key_path = NULL;
	// no special options
	int opts = 0;

	//* register the signal SIGINT handler */
	struct sigaction act;
	act.sa_handler = INT_HANDLER;
	act.sa_flags = 0;
	sigemptyset(&act.sa_mask);
	sigaction(SIGINT, &act, 0);

	//* setup websocket protocol */
	protocol.name = "xyz.jackfaller.dissertation-handler";
	protocol.callback = ws_service_callback;
	protocol.per_session_data_size = sizeof(struct per_session_data);
	protocol.rx_buffer_size = 0;

	//* setup websocket context info*/
	memset(&info, 0, sizeof info);
	info.port = port;

	info.iface = interface;
	info.protocols = &protocol;
	/* info.extensions = lws_get_internal_extensions(); */
	info.ssl_cert_filepath = cert_path;
	info.ssl_private_key_filepath = key_path;
	info.gid = -1;
	info.uid = -1;
	info.options = opts;

	//* create libwebsocket context. */
	context = lws_create_context(&info);
	if (context == NULL) {
		eprintf(KRED "[Main] Websocket context create error.\n" RESET);
		return -1;
	}

	eprintf(KGRN "[Main] Websocket context create success.\n" RESET);

	//* websocket service */
	while (!destroy_flag) {
		lws_service(context, 50);
	}
	usleep(10);
	lws_context_destroy(context);

	return 0;
}

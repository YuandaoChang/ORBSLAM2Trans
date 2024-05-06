#include "WebSocket.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>

WebSocket websocket;

unsigned char *WebSocket::data = nullptr;
int WebSocket::length = 0;
int WebSocket::count = 0;
int WebSocket::keyPointsSize = 0;
bool WebSocket::isReceived = false;
std::vector<cv::Mat> WebSocket::keyFrame;
std::vector<CovisibilityGraph> WebSocket::covGraph;
std::vector<SpanningTree> WebSocket::spaTree;
std::vector<Loops> WebSocket::loops;
std::vector<cv::Mat> WebSocket::mapPoints;
std::vector<cv::Mat> WebSocket::refPoints;

WebSocket::WebSocket() : context(nullptr), port(0)
{
    // Initialize member variables
    protocols[0] = {"ws", callback, 0, MAX_PAYLOAD_SIZE};
    protocols[1] = {NULL, NULL, 0, 0};
}

void WebSocket::init(int port)
{
    this->port = port;

    struct lws_context_creation_info info;
    memset(&info, 0, sizeof(info));
    info.port = port;
    info.protocols = protocols;

    this->context = lws_create_context(&info);

    if (!context)
    {
        printf("No create contest\n");
    }

    printf("Create contest on port %d\n", this->port);
}

void WebSocket::stopServer()
{
    if (this->context)
    {
        lws_context_destroy(this->context);
        this->context = nullptr;
    }
}

void WebSocket::waitForSignal()
{
    if (this->context)
    {
        lws_service(this->context, 50);
    }
}

void WebSocket::sendToClient()
{
    if (this->context)
    {
        lws_callback_on_writable_all_protocol(this->context, &protocols[0]);
        lws_service(this->context, 50);
        // Implement sending data to connected WebSocket clients
        // You can use lws_write function to send messages
    }
}

int WebSocket::callback(struct lws *wsi, enum lws_callback_reasons reason, void *user, void *in, size_t len)
{

    // WebSocket callback handling code
    // Implement your handling logic here
    switch (reason)
    {
    case LWS_CALLBACK_ESTABLISHED:
    {
        printf("WebSocket connection established\n");
        websocket.isReceived = true;
    }
    break;
    case LWS_CALLBACK_RECEIVE:
        printf("Received data: %s\n", (char *)in);
        break;
    case LWS_CALLBACK_CLOSED:
        printf("WebSocket connection closed\n");
        break;
    case LWS_CALLBACK_SERVER_WRITEABLE:
    {
        int n;
        n = lws_write(wsi, WebSocket::data + LWS_PRE, WebSocket::length, LWS_WRITE_BINARY);
        n = lws_callback_on_writable(wsi);
    }
    break;
    default:
        break;
    }
    return 0;
}

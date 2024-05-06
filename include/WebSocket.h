#ifndef WEBSOCKET_H
#define WEBSOCKET_H

#define MAX_PAYLOAD_SIZE 30 * 1024
#include <libwebsockets.h>
#include <string>
#include <pangolin/pangolin.h>
#include <opencv2/opencv.hpp>
#include <vector>
struct CovisibilityGraph {
    cv::Mat Ow;
    cv::Mat Ow2;
};

struct SpanningTree {
    cv::Mat Ow;
    cv::Mat Owp;
};
struct Loops {
    cv::Mat Ow;
    cv::Mat Owl;
};

class WebSocket
{
public:
    WebSocket();

    void init(int port);
    void stopServer();
    void sendToClient();
    void waitForSignal();
    static unsigned char *data;
    static int length;
    static int count;
    static int keyPointsSize;
    static bool isReceived;
    static std::vector<cv::Mat> keyFrame;
    static std::vector<CovisibilityGraph> covGraph;
    static std::vector<SpanningTree> spaTree;
    static std::vector<Loops> loops;
    static std::vector<cv::Mat> mapPoints;
    static std::vector<cv::Mat> refPoints;
private:
    struct lws_context *context;
    struct lws_protocols protocols[2];

    int port;

    static int callback(
        struct lws *wsi,
        enum lws_callback_reasons reason,
        void *user,
        void *in,
        size_t len);
};

#endif

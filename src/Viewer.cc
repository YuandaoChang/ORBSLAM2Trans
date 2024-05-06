/**
 * This file is part of ORB-SLAM2.
 *
 * Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
 * For more information see <https://github.com/raulmur/ORB_SLAM2>
 *
 * ORB-SLAM2 is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ORB-SLAM2 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
 */

#include "Viewer.h"
#include <pangolin/pangolin.h>
#include <mutex>
#include "WebSocket.h"
#include <chrono>
#include <iomanip>
#include <ctime>
extern WebSocket websocket;
namespace ORB_SLAM2
{

    Viewer::Viewer(System *pSystem, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Tracking *pTracking, const string &strSettingPath) : mpSystem(pSystem), mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer), mpTracker(pTracking),
                                                                                                                                           mbFinishRequested(false), mbFinished(true), mbStopped(true), mbStopRequested(false)
    {
        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

        float fps = fSettings["Camera.fps"];
        if (fps < 1)
            fps = 30;
        mT = 1e3 / fps;

        mImageWidth = fSettings["Camera.width"];
        mImageHeight = fSettings["Camera.height"];
        if (mImageWidth < 1 || mImageHeight < 1)
        {
            mImageWidth = 640;
            mImageHeight = 480;
        }

        mViewpointX = fSettings["Viewer.ViewpointX"];
        mViewpointY = fSettings["Viewer.ViewpointY"];
        mViewpointZ = fSettings["Viewer.ViewpointZ"];
        mViewpointF = fSettings["Viewer.ViewpointF"];
    }

    void Viewer::Run()
    {
        mbFinished = false;
        mbStopped = false;

        pangolin::CreateWindowAndBind("ORB-SLAM2: Map Viewer", 1024, 768);

        // 3D Mouse handler requires depth testing to be enabled
        glEnable(GL_DEPTH_TEST);

        // Issue specific OpenGl we might need
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(175));
        pangolin::Var<bool> menuFollowCamera("menu.Follow Camera", true, true);
        pangolin::Var<bool> menuShowPoints("menu.Show Points", true, true);
        pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames", true, true);
        pangolin::Var<bool> menuShowGraph("menu.Show Graph", true, true);
        pangolin::Var<bool> menuLocalizationMode("menu.Localization Mode", false, true);
        pangolin::Var<bool> menuReset("menu.Reset", false, false);

        // Define Camera Render Object (for view / scene browsing)
        pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, mViewpointF, mViewpointF, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(mViewpointX, mViewpointY, mViewpointZ, 0, 0, 0, 0.0, -1.0, 0.0));

        // Add named OpenGL viewport to window and provide 3D Handler
        pangolin::View &d_cam = pangolin::CreateDisplay()
                                    .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
                                    .SetHandler(new pangolin::Handler3D(s_cam));

        pangolin::OpenGlMatrix Twc;
        Twc.SetIdentity();

        cv::namedWindow("ORB-SLAM2: Current Frame");

        bool bFollow = true;
        bool bLocalizationMode = false;
        // 创建计时器
        std::chrono::steady_clock::time_point start_time;
        std::chrono::steady_clock::time_point end_time;
        while (1)
        {
            // 开始计时
            start_time = std::chrono::steady_clock::now();
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            mpMapDrawer->GetCurrentOpenGLCameraMatrix(Twc);
            if (menuFollowCamera && bFollow)
            {
                s_cam.Follow(Twc);
            }
            else if (menuFollowCamera && !bFollow)
            {
                s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(mViewpointX, mViewpointY, mViewpointZ, 0, 0, 0, 0.0, -1.0, 0.0));
                s_cam.Follow(Twc);
                bFollow = true;
            }
            else if (!menuFollowCamera && bFollow)
            {
                bFollow = false;
            }

            if (menuLocalizationMode && !bLocalizationMode)
            {
                mpSystem->ActivateLocalizationMode();
                bLocalizationMode = true;
            }
            else if (!menuLocalizationMode && bLocalizationMode)
            {
                mpSystem->DeactivateLocalizationMode();
                bLocalizationMode = false;
            }

            d_cam.Activate(s_cam);
            glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

            mpMapDrawer->DrawCurrentCamera(Twc);
            if (menuShowKeyFrames || menuShowGraph)
                mpMapDrawer->DrawKeyFrames(menuShowKeyFrames, menuShowGraph);
            if (menuShowPoints)
                mpMapDrawer->DrawMapPoints();

            pangolin::FinishFrame();

            cv::Mat im = mpFrameDrawer->DrawFrame();
            cv::imshow("ORB-SLAM2: Current Frame", im);

            cout << "第" << ++websocket.count << "循环" << endl;

            // send cameraPose
            int len = sizeof(pangolin::OpenGlMatrix);
            unsigned char *out = (unsigned char *)malloc(sizeof(unsigned char) * (LWS_PRE + 8 + len));
            out[LWS_PRE] = 'A'; //
            memcpy(out + LWS_PRE + 8, Twc.m, len);
            websocket.data = out;
            websocket.length = len + 8;
            websocket.sendToClient();
            free(out);

            // send keyFrame
            int keyFrameSize = websocket.keyFrame.size() * sizeof(float) * 16;
            out = (unsigned char *)malloc(sizeof(unsigned char) * (LWS_PRE + 8 + keyFrameSize));
            out[LWS_PRE] = 'B';
            unsigned char *currentkeyFrame = out + LWS_PRE + 8;
            for (const auto &obj : websocket.keyFrame)
            {
                memcpy(currentkeyFrame, obj.data, sizeof(float) * 16);
                currentkeyFrame += sizeof(float) * 16;
            }
            websocket.data = out;
            websocket.length = keyFrameSize + 8;
            websocket.sendToClient();
            // cout << "keyFrame数据量为:" << websocket.keyFrame.size() << endl;
            websocket.keyFrame.clear();
            free(out);

            // send covGraph
            int covGraphSize = websocket.covGraph.size() * sizeof(float) * 6;
            out = (unsigned char *)malloc(sizeof(unsigned char) * (LWS_PRE + 8 + covGraphSize));
            out[LWS_PRE] = 'C';
            unsigned char *currentcovGraph = out + LWS_PRE + 8;
            for (const auto &obj : websocket.covGraph)
            {
                memcpy(currentcovGraph, obj.Ow.data, sizeof(float) * 3);
                currentcovGraph += sizeof(float) * 3;
                memcpy(currentcovGraph, obj.Ow2.data, sizeof(float) * 3);
                currentcovGraph += sizeof(float) * 3;
            }
            websocket.data = out;
            websocket.length = covGraphSize + 8;
            websocket.sendToClient();
            // cout << "covGraph数据量为:" << websocket.covGraph.size() << endl;
            websocket.covGraph.clear();
            free(out);

            // send spaTree
            int spaTreeSize = websocket.spaTree.size() * sizeof(float) * 6;
            out = (unsigned char *)malloc(sizeof(unsigned char) * (LWS_PRE + 8 + spaTreeSize));
            out[LWS_PRE] = 'D';
            unsigned char *currentspaTree = out + LWS_PRE + 8;
            for (const auto &obj : websocket.spaTree)
            {
                memcpy(currentspaTree, obj.Ow.data, sizeof(float) * 3);
                currentspaTree += sizeof(float) * 3;
                memcpy(currentspaTree, obj.Owp.data, sizeof(float) * 3);
                currentspaTree += sizeof(float) * 3;
            }
            websocket.data = out;
            websocket.length = spaTreeSize + 8;
            websocket.sendToClient();
            websocket.spaTree.clear();
            free(out);

            // send loops
            int loopsSize = websocket.loops.size() * sizeof(float) * 6;
            out = (unsigned char *)malloc(sizeof(unsigned char) * (LWS_PRE + 8 + loopsSize));
            out[LWS_PRE] = 'E';
            unsigned char *currentLoops = out + LWS_PRE + 8;
            for (const auto &obj : websocket.loops)
            {
                memcpy(currentLoops, obj.Ow.data, sizeof(float) * 3);
                currentLoops += sizeof(float) * 3;
                memcpy(currentLoops, obj.Owl.data, sizeof(float) * 3);
                currentLoops += sizeof(float) * 3;
            }
            websocket.data = out;
            websocket.length = loopsSize + 8;
            websocket.sendToClient();
            // cout << "loops数据量为:" << websocket.loops.size() << endl;
            websocket.loops.clear();
            free(out);

            // send mapPoints
            int mapPointsSize = websocket.mapPoints.size() * sizeof(float) * 3;
            out = (unsigned char *)malloc(sizeof(unsigned char) * (LWS_PRE + 8 + mapPointsSize));
            out[LWS_PRE] = 'F';
            unsigned char *currentmapPoints = out + LWS_PRE + 8;
            for (const auto &obj : websocket.mapPoints)
            {
                memcpy(currentmapPoints, obj.data, sizeof(float) * 3);
                currentmapPoints += sizeof(float) * 3;
            }
            websocket.data = out;
            websocket.length = mapPointsSize + 8;
            websocket.sendToClient();
            // cout << "mapPoints数据量为:" << websocket.mapPoints.size() << endl;
            websocket.mapPoints.clear();
            free(out);

            // send refPoints
            int refPointsSize = websocket.refPoints.size() * sizeof(float) * 3;
            out = (unsigned char *)malloc(sizeof(unsigned char) * (LWS_PRE + 8 + refPointsSize));
            out[LWS_PRE] = 'G';
            unsigned char *currentrefPoints = out + LWS_PRE + 8;
            for (const auto &obj : websocket.refPoints)
            {
                memcpy(currentrefPoints, obj.data, sizeof(float) * 3);
                currentrefPoints += sizeof(float) * 3;
            }
            websocket.data = out;
            websocket.length = refPointsSize + 8;
            websocket.sendToClient();
            // cout << "refPoints数据量为:" << websocket.refPoints.size() << endl;
            websocket.refPoints.clear();
            free(out);

            // send keyPointsSize
            int keyPointsSize = websocket.keyPointsSize;
            out = (unsigned char *)malloc(sizeof(unsigned char) * (LWS_PRE + 8 + sizeof(int)));
            out[LWS_PRE] = 'K';
            memcpy(out + LWS_PRE + 8, &keyPointsSize, sizeof(int));
            websocket.data = out;
            websocket.length = sizeof(int) + 8;
            websocket.sendToClient();
            free(out);

            len = 8;
            out = (unsigned char *)malloc(sizeof(unsigned char) * len);
            out[LWS_PRE] = 'Z';
            websocket.data = out;
            websocket.length = len;
            websocket.sendToClient();
            // cout << "Z has been sent" << endl;
            free(out); // 在每次发送消息后释放内存
            cv::waitKey(mT);

            // 结束计时
            end_time = std::chrono::steady_clock::now();

            // 计算循环运行时间
            auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

            // 打印循环运行时间
            std::cout << "循环运行时间: " << elapsed_time << " 毫秒" << std::endl;
            if (menuReset)
            {
                menuShowGraph = true;
                menuShowKeyFrames = true;
                menuShowPoints = true;
                menuLocalizationMode = false;
                if (bLocalizationMode)
                    mpSystem->DeactivateLocalizationMode();
                bLocalizationMode = false;
                bFollow = true;
                menuFollowCamera = true;
                mpSystem->Reset();
                menuReset = false;
            }

            if (Stop())
            {
                while (isStopped())
                {
                    usleep(3000);
                }
            }

            if (CheckFinish())
                break;
        }

        SetFinish();
        // 获取当前时间点
        auto now = std::chrono::system_clock::now();

        // 转换为time_t以便我们可以使用 ctime
        std::time_t now_c = std::chrono::system_clock::to_time_t(now);

        // 转换为本地时间
        std::tm now_tm = *std::localtime(&now_c);

        // 获取毫秒部分
        auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

        // 打印时间，精确到毫秒
        std::cout << std::put_time(&now_tm, "%Y-%m-%d %H:%M:%S");
        std::cout << '.' << std::setfill('0') << std::setw(3) << milliseconds.count() << std::endl;
        websocket.stopServer();
    }

    void Viewer::RequestFinish()
    {
        unique_lock<mutex> lock(mMutexFinish);
        mbFinishRequested = true;
    }

    bool Viewer::CheckFinish()
    {
        unique_lock<mutex> lock(mMutexFinish);
        return mbFinishRequested;
    }

    void Viewer::SetFinish()
    {
        unique_lock<mutex> lock(mMutexFinish);
        mbFinished = true;
    }

    bool Viewer::isFinished()
    {
        unique_lock<mutex> lock(mMutexFinish);
        return mbFinished;
    }

    void Viewer::RequestStop()
    {
        unique_lock<mutex> lock(mMutexStop);
        if (!mbStopped)
            mbStopRequested = true;
    }

    bool Viewer::isStopped()
    {
        unique_lock<mutex> lock(mMutexStop);
        return mbStopped;
    }

    bool Viewer::Stop()
    {
        unique_lock<mutex> lock(mMutexStop);
        unique_lock<mutex> lock2(mMutexFinish);

        if (mbFinishRequested)
            return false;
        else if (mbStopRequested)
        {
            mbStopped = true;
            mbStopRequested = false;
            return true;
        }

        return false;
    }

    void Viewer::Release()
    {
        unique_lock<mutex> lock(mMutexStop);
        mbStopped = false;
    }

}

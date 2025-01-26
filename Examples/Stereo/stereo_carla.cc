#include <iostream>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <fstream>
#include <opencv2/core/core.hpp>

#include <System.h>
#include <nlohmann/json.hpp>
#include <unordered_map>

using namespace std;
using json = nlohmann::json;

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft, vector<string> &vstrImageLeftSemantic, vector<string> &vstrImageLeftSemanticMeta,
                vector<string> &vstrImageRight, vector<double> &vTimestamps);

int main(int argc, char **argv) {
        if (argc != 5) {
                cerr << endl
                     << "Usage: ./carla path_to_vocabulary path_to_settings path_to_sequence results_path" << endl;
                return 1;
        }

        vector<string> vstrImageLeft;
        vector<string> vstrImageLeftSemantic;
        vector<string> vstrImageLeftSemanticMeta;
        vector<string> vstrImageRight;
        vector<double> vTimestamps;

        LoadImages(string(argv[3]), vstrImageLeft, vstrImageLeftSemantic, vstrImageLeftSemanticMeta, vstrImageRight, vTimestamps);
        const int nImages = vstrImageLeft.size();

        ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::STEREO, true, 0, std::string());
        float imageScale = SLAM.GetImageScale();

        // Vector for tracking time statistics
        vector<float> vTimesTrack;
        vTimesTrack.resize(nImages);

        cout << endl
             << "-------" << endl;
        cout << "Start processing sequence ..." << endl;
        cout << "Images in the sequence: " << nImages << endl
             << endl;

        double t_track = 0.f;
        double t_resize = 0.f;

        // Main loop
        cv::Mat imLeft, imLeftSem, imRight;
        for (int ni = 0; ni < nImages; ni++)
        {
                // Read left and right images from file
                imLeft = cv::imread(vstrImageLeft[ni], cv::IMREAD_UNCHANGED);
                imRight = cv::imread(vstrImageRight[ni], cv::IMREAD_UNCHANGED);
                // Read semantic information for left image (segmentation image and json with moving objects)
                imLeftSem = cv::imread(vstrImageLeftSemantic[ni], cv::IMREAD_UNCHANGED);
                ifstream f(vstrImageLeftSemanticMeta[ni]);
                json seg_meta_json = json::parse(f);
                unordered_map<int, bool> seg_meta;
                for (json::iterator it = seg_meta_json.begin(); it != seg_meta_json.end(); ++it)
                {
                        seg_meta[(*it)["object_id"]] = (*it)["is_moving"];
                }
                double tframe = vTimestamps[ni];

                if (imLeft.empty())
                {
                        cerr << endl
                             << "Failed to load image at: "
                             << string(vstrImageLeft[ni]) << endl;
                        return 1;
                }

                if (imageScale != 1.f)
                {
#ifdef REGISTER_TIMES
#ifdef COMPILEDWITHC11
                        std::chrono::steady_clock::time_point t_Start_Resize = std::chrono::steady_clock::now();
#else
                        std::chrono::steady_clock::time_point t_Start_Resize = std::chrono::steady_clock::now();
#endif
#endif
                        int width = imLeft.cols * imageScale;
                        int height = imLeft.rows * imageScale;
                        cv::resize(imLeft, imLeft, cv::Size(width, height));
                        cv::resize(imRight, imRight, cv::Size(width, height));
                        cv::resize(imLeftSem, imLeftSem, cv::Size(width, height));
#ifdef REGISTER_TIMES
#ifdef COMPILEDWITHC11
                        std::chrono::steady_clock::time_point t_End_Resize = std::chrono::steady_clock::now();
#else
                        std::chrono::steady_clock::time_point t_End_Resize = std::chrono::steady_clock::now();
#endif
                        t_resize = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(t_End_Resize - t_Start_Resize).count();
                        SLAM.InsertResizeTime(t_resize);
#endif
                }

#ifdef COMPILEDWITHC11
                std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
                std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#endif

                // Pass the images to the SLAM system
                SLAM.TrackStereoSemantic(imLeft, imRight, imLeftSem, seg_meta, tframe);

#ifdef COMPILEDWITHC11
                std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
                std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#endif

#ifdef REGISTER_TIMES
                t_track = t_resize + std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(t2 - t1).count();
                SLAM.InsertTrackTime(t_track);
#endif

                double ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();

                vTimesTrack[ni] = ttrack;

                // Wait to load the next frame
                double T = 0;
                if (ni < nImages - 1)
                        T = vTimestamps[ni + 1] - tframe;
                else if (ni > 0)
                        T = tframe - vTimestamps[ni - 1];

                // if (ttrack < T)
                //         usleep((T - ttrack) * 1e6);
        }

        // Stop all threads
        SLAM.Shutdown();

        // Tracking time statistics
        sort(vTimesTrack.begin(), vTimesTrack.end());
        float totaltime = 0;
        for (int ni = 0; ni < nImages; ni++)
        {
                totaltime += vTimesTrack[ni];
        }
        cout << "-------" << endl
             << endl;
        cout << "median tracking time: " << vTimesTrack[nImages / 2] << endl;
        cout << "mean tracking time: " << totaltime / nImages << endl;

        // Save camera trajectory
        SLAM.SaveTrajectoryKITTI(string(argv[4]));

        return 0;
}

void LoadImages(const string &strPathToSequence,
                vector<string> &vstrImageLeft,
                vector<string> &vstrImageLeftSemantic,
                vector<string> &vstrImageLeftSemanticMeta,
                vector<string> &vstrImageRight,
                vector<double> &vTimestamps)
{
        ifstream fTimes;
        string strPathTimeFile = strPathToSequence + "/times.txt";
        fTimes.open(strPathTimeFile.c_str());
        vector<int> Frames;
        while (!fTimes.eof())
        {
                string s;
                getline(fTimes, s);
                if (!s.empty())
                {
                        vector<string> result;
                        stringstream lineStream(s);
                        string cell;
                        while (std::getline(lineStream, cell, ' '))
                        {
                                result.push_back(cell);
                        }
                        stringstream ss;
                        ss << result[0];
                        double t;
                        ss >> t;
                        Frames.push_back(t);

                        stringstream ss1;
                        ss1 << result[1];
                        double t1;
                        ss1 >> t1;
                        vTimestamps.push_back(t1);
                }
        }

        string strPrefixLeft = strPathToSequence + "/img_left/";
        string strPrefixRight = strPathToSequence + "/img_right/";
        string strPrefixLeftSeg = strPathToSequence + "/img_left_seg/";
        string strPrefixSemanticMeta = strPathToSequence + "/actors_json/";

        const int nTimes = vTimestamps.size();
        vstrImageLeft.resize(nTimes);
        vstrImageRight.resize(nTimes);
        vstrImageLeftSemantic.resize(nTimes);
        vstrImageLeftSemanticMeta.resize(nTimes);

        for (int i = 0; i < nTimes; i++)
        {
                string frame_str = to_string(Frames[i]);
                vstrImageLeft[i] = strPrefixLeft + frame_str + ".png";
                vstrImageRight[i] = strPrefixRight + frame_str + ".png";
                vstrImageLeftSemantic[i] = strPrefixLeftSeg + frame_str + ".png";
                vstrImageLeftSemanticMeta[i] = strPrefixSemanticMeta + frame_str + ".json";
        }
}

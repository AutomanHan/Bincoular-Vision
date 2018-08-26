
#ifdef VERBOSE
#  include <iostream>
#  define VERBOSE_LOG std::cout << ">> " << __func__ << std::endl
#else
#  define VERBOSE_LOG
#endif

#include <opencv2/calib3d/calib3d.hpp>
// #include <opencv2/imgproc/imgproc.hpp>

#include "camera.h"
#include "plugin.h"

namespace mynteye {

// Depth Map from Stereo Images
//   http://docs.opencv.org/master/dd/d53/tutorial_py_depthmap.html

class MYNTEYE_API OCVBMPlugin : public Plugin {
public:
    OCVBMPlugin();
    virtual ~OCVBMPlugin();

    virtual void OnCreate(Camera *camera) override;

    virtual void OnOpen(const ErrorCode &code) override;

    virtual void OnGrab(const ErrorCode &code) override;

    virtual void OnRetrieveImage(cv::Mat &mat, const View &view, const ErrorCode &code) override;

    virtual void OnRetrieveIMUData(std::vector<IMUData> &imudatas, const ErrorCode &code) override;

    virtual void OnClose() override;

    virtual void OnProcessGrab(cv::Mat &left_raw, cv::Mat &right_raw) override;

    virtual bool OnProcessRecify(
            const cv::Mat &left_raw, const cv::Mat &right_raw,
            cv::Mat &left_rectified, cv::Mat &right_rectified) override;

    virtual bool OnProcessDepthMap(
            const cv::Mat &left_rectified, const cv::Mat &right_rectified,
            cv::Mat &depthmap) override;

    virtual bool OnProcessPointCloud(
            const cv::Mat &depthmap, cv::Mat &pointcloud) override;

private:
    cv::Ptr<cv::StereoBM> bm_;
};

OCVBMPlugin::OCVBMPlugin() {
    VERBOSE_LOG;
#ifdef USE_OPENCV2
    bm_ = cv::Ptr<cv::StereoBM>(
        new cv::StereoBM(cv::StereoBM::BASIC_PRESET, 64, 21));
#else
    bm_ = cv::StereoBM::create(64, 21);
#endif
}

OCVBMPlugin::~OCVBMPlugin() {
    VERBOSE_LOG;
}

void OCVBMPlugin::OnCreate(Camera *camera) {
    VERBOSE_LOG;
    Plugin::OnCreate(camera);
}

void OCVBMPlugin::OnOpen(const ErrorCode &code) {
    VERBOSE_LOG;
}

void OCVBMPlugin::OnGrab(const ErrorCode &code) {
    VERBOSE_LOG;
}

void OCVBMPlugin::OnRetrieveImage(cv::Mat &mat, const View &view, const ErrorCode &code) {
    VERBOSE_LOG;
}

void OCVBMPlugin::OnRetrieveIMUData(std::vector<IMUData> &imudatas, const ErrorCode &code) {
    VERBOSE_LOG;
}

void OCVBMPlugin::OnClose() {
    VERBOSE_LOG;
}

void OCVBMPlugin::OnProcessGrab(cv::Mat &left_raw, cv::Mat &right_raw) {
    // Scale the grabbed images.
    // cv::resize(left_raw, left_raw, cv::Size(), 0.5, 0.5, cv::INTER_LINEAR);
    // cv::resize(right_raw, right_raw, cv::Size(), 0.5, 0.5, cv::INTER_LINEAR);
}

bool OCVBMPlugin::OnProcessRecify(
        const cv::Mat &left_raw, const cv::Mat &right_raw,
        cv::Mat &left_rectified, cv::Mat &right_rectified) {
    VERBOSE_LOG;
    return false;
}

bool OCVBMPlugin::OnProcessDepthMap(
        const cv::Mat &left_rectified, const cv::Mat &right_rectified,
        cv::Mat &depthmap) {
    VERBOSE_LOG;
#ifdef USE_OPENCV2
    (*bm_)(left_rectified, right_rectified, depthmap);
#else
    bm_->compute(left_rectified, right_rectified, depthmap);
#endif
    return true;
}

bool OCVBMPlugin::OnProcessPointCloud(
        const cv::Mat &depthmap, cv::Mat &pointcloud) {
    VERBOSE_LOG;
    return false;
}

}  // namespace mynteye

extern "C" {

MYNTEYE_API mynteye::Plugin *plugin_create() {
    return new mynteye::OCVBMPlugin();
}

MYNTEYE_API void plugin_destroy(mynteye::Plugin *plugin) {
    delete plugin;
}

}

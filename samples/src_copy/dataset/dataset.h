#ifndef MYNTEYE_SDK_SAMPLES_DATASET_H_
#define MYNTEYE_SDK_SAMPLES_DATASET_H_
#pragma once

#include <chrono>
#include <cstdint>
#include <fstream>

#include <opencv2/core/core.hpp>

class Dataset {
public:
#ifdef CXX11
    using clock = std::chrono::steady_clock;
#else
    using clock = std::chrono::monotonic_clock;
#endif

    explicit Dataset(const std::string &outdir)
        : outdir_(outdir), count_(0) {}

    virtual ~Dataset() {}

    virtual void Save(const cv::Mat &left, const cv::Mat &right) = 0;

    std::uint64_t GetCount() { return count_; }

protected:
    std::string outdir_;
    std::uint64_t count_;
};

class KittiDataset : public Dataset {
public:
    explicit KittiDataset(const std::string &outdir);
    virtual ~KittiDataset();

    virtual void Save(const cv::Mat &left, const cv::Mat &right) override;

private:
    std::string dir_img0_;
    std::string dir_img1_;
    std::string file_times_;

    clock::time_point time_start_;
    std::ofstream ofs_times_;
};

#endif  // MYNTEYE_SDK_SAMPLES_DATASET_H_

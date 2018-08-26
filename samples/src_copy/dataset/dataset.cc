#include "dataset.h"

#include <iomanip>
#include <iostream>
#include <string>

#ifdef USE_OPENCV2
#include <opencv2/highgui/highgui.hpp>
#else
#include <opencv2/imgcodecs/imgcodecs.hpp>
#endif

#if defined(_WIN32)
#  define OS_WIN
#  if defined(__MINGW32__) || defined(__MINGW64__)
#    define OS_MINGW
#  endif
#else
#  define OS_UNIX
#endif

#if defined(OS_WIN) && !defined(OS_MINGW)
#  include <direct.h>
#  define PATH_SEPARATOR "\\"
#else
#  include <sys/stat.h>
#  define PATH_SEPARATOR "/"
#endif

namespace {

// Boost.Filesystem is better here.

void MakeDir(const std::string &dir) {
#if defined(OS_MINGW)
    const int status = mkdir(dir.c_str());
#elif defined(OS_WIN)
    const int status = _mkdir(dir.c_str());
#else // OS_UNIX
    const int status = mkdir(dir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
#endif
    if (status != 0 && errno != EEXIST) {
        std::cerr << "Error: Create directory failed." << std::endl;
        std::cerr << "  Status: " << status << std::endl
                  << "  Path: " << dir << std::endl;
        exit(2);
    }
    if (errno != EEXIST) {
        std::cout << "Create directory: " << dir << std::endl;
    }
}

}  // namespace

KittiDataset::KittiDataset(const std::string &outdir)
    : Dataset(outdir),
      dir_img0_(outdir + PATH_SEPARATOR"image_0"),
      dir_img1_(outdir + PATH_SEPARATOR"image_1"),
      file_times_(outdir + PATH_SEPARATOR"times.txt") {
    MakeDir(outdir);
    MakeDir(dir_img0_);
    MakeDir(dir_img1_);

    ofs_times_.open(file_times_, std::ofstream::out);
    ofs_times_ << std::scientific;
}

KittiDataset::~KittiDataset() {
    ofs_times_.flush();
    ofs_times_.close();
}

void KittiDataset::Save(const cv::Mat &left, const cv::Mat &right) {
    if (count_ == 0) {
        time_start_ = clock::now();
    }

    std::stringstream ss;
    ss << std::dec << std::setw(6) << std::setfill('0') << count_++ << ".png";
    std::string filename = ss.str();

    std::string path_img_left = dir_img0_ + PATH_SEPARATOR + filename;
    std::string path_img_right = dir_img1_ + PATH_SEPARATOR + filename;

    cv::imwrite(path_img_left, left);
    cv::imwrite(path_img_right, right);

    double elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(
        clock::now() - time_start_).count();
    ofs_times_ << elapsed << std::endl;

    std::cout << "\rCapture " << count_ << " frames" << std::flush;
}

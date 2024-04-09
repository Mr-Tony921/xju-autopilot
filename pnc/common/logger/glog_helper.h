/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <glog/raw_logging.h>
#include <unistd.h>

#include <iostream>
#include <string>

namespace xju {
namespace pnc {
  // void SignalHandle(const char* data, int size) {
  //   std::string str = std::string(data, size);
  //   LOG(ERROR) << str;
  // }
class GlogHelper {
 public:
  explicit GlogHelper(const char* program) {
    int ret = 0;
    ret = access((std::string(FLAGS_log_dir)).c_str(), R_OK | W_OK);
    if (ret != 0) {
      std::cout << FLAGS_log_dir << std::endl;
      std::string str_mkdir =
          "mkdir " + static_cast<std::string>(FLAGS_log_dir);
      system(str_mkdir.c_str());
    }

    FLAGS_logbufsecs = 0;
    FLAGS_stderrthreshold = google::ERROR;
    FLAGS_max_log_size = 100;
    FLAGS_stop_logging_if_full_disk = true;
    google::InitGoogleLogging(program);
    google::InstallFailureSignalHandler();
    // google::InstallFailureWriter(&SignalHandle);
  }
  ~GlogHelper() { google::ShutdownGoogleLogging(); }
};

}  // namespace pnc
}  // namespace xju

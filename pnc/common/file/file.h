/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <string>

#include "google/protobuf/io/zero_copy_stream_impl.h"
#include "google/protobuf/text_format.h"

namespace xju {
namespace pnc {

class File {
 public:
  template <typename T>
  static bool GetProtoConfig(const std::string& file_name, T* config) {
    static const std::string kBinExt = ".bin";
    if (std::equal(kBinExt.rbegin(), kBinExt.rend(), file_name.rbegin())) {
      return GetProtoFromBinaryFile(file_name, config) || 
             GetProtoASCIIFromFile(file_name, config);
    }

    return GetProtoASCIIFromFile(file_name, config) || 
           GetProtoFromBinaryFile(file_name, config);
  }

  static bool GetGflagConfig(const std::string& file_name);

 private:
  static bool GetProtoASCIIFromFile(const std::string& file_name,
                                    google::protobuf::Message* message);

  static bool GetProtoFromBinaryFile(const std::string& file_name,
                                     google::protobuf::Message* message);
};

} // namespace pnc
} // namespace xju

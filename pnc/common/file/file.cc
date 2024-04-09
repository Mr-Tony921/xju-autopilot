/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "common/file/file.h"

#include <fstream>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>

#include "gflags/gflags.h"

#include <iostream>

namespace xju {
namespace pnc {

bool File::GetGflagConfig(const std::string& file_name) {
  google::SetCommandLineOption("flagfile", file_name.c_str());
  return true;
}

bool File::GetProtoASCIIFromFile(
    const std::string& file_name, google::protobuf::Message* message) {
  using google::protobuf::TextFormat;
  using google::protobuf::io::FileInputStream;
  using google::protobuf::io::ZeroCopyInputStream;

  int file_descriptor = open(file_name.c_str(), O_RDONLY);

  if (file_descriptor < 0) {
    return false;
  }

  ZeroCopyInputStream *input = new FileInputStream(file_descriptor);
  bool success = TextFormat::Parse(input, message);
  if (!success) {
    return false;
  }
  delete input;
  close(file_descriptor);
  return true;
}

bool File::GetProtoFromBinaryFile(
    const std::string& file_name, google::protobuf::Message* message) {
  std::fstream input(file_name, std::ios::in | std::ios::binary);
  if (!input.good()) {
    return false;
  }
  if (!message->ParseFromIstream(&input)) {
    return false;
  }
  return true;
}

} // namespace pnc
} // namespace xju
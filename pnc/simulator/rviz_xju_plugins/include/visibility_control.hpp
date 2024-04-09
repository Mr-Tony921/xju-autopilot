/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/
#pragma once

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define RVIZ_XJU_PLUGINS_EXPORT __attribute__ ((dllexport))
    #define RVIZ_XJU_PLUGINS_IMPORT __attribute__ ((dllimport))
  #else
    #define RVIZ_XJU_PLUGINS_EXPORT __declspec(dllexport)
    #define RVIZ_XJU_PLUGINS_IMPORT __declspec(dllimport)
  #endif
  #ifdef RVIZ_XJU_PLUGINS_BUILDING_LIBRARY
    #define RVIZ_XJU_PLUGINS_PUBLIC RVIZ_XJU_PLUGINS_EXPORT
  #else
    #define RVIZ_XJU_PLUGINS_PUBLIC RVIZ_XJU_PLUGINS_IMPORT
  #endif
  #define RVIZ_XJU_PLUGINS_PUBLIC_TYPE RVIZ_XJU_PLUGINS_PUBLIC
  #define RVIZ_XJU_PLUGINS_LOCAL
#else
  #define RVIZ_XJU_PLUGINS_EXPORT __attribute__ ((visibility("default")))
  #define RVIZ_XJU_PLUGINS_IMPORT
  #if __GNUC__ >= 4
    #define RVIZ_XJU_PLUGINS_PUBLIC __attribute__ ((visibility("default")))
    #define RVIZ_XJU_PLUGINS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define RVIZ_XJU_PLUGINS_PUBLIC
    #define RVIZ_XJU_PLUGINS_LOCAL
  #endif
  #define RVIZ_XJU_PLUGINS_PUBLIC_TYPE
#endif

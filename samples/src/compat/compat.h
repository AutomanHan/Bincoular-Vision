#ifndef MYNTEYE_SDK_SAMPLES_COMPAT_H_
#define MYNTEYE_SDK_SAMPLES_COMPAT_H_
#pragma once

#if defined(_WIN32)
#  define OS_WIN
#  if defined(__MINGW32__) || defined(__MINGW64__)
#    define OS_MINGW
#  endif
#else
#  define OS_UNIX
#endif

#if defined(OS_WIN)

#include <conio.h>

#else

extern int _kbhit();

#endif

#endif  // MYNTEYE_SDK_SAMPLES_COMPAT_H_

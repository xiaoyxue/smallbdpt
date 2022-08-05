#pragma once

#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS
#endif

// Platforms

// Windows
#if defined(_WIN64)
#define GYT_PLATFORM_WINDOWS
#endif

#if defined(_WIN32) && !defined(_WIN64)
static_assert(false, "32-bit Windows systems are not supported")
#endif

// Compilers
// MSVC
#if defined(_MSC_VER)
#define GYT_COMPILER_MSVC
#endif

# pragma warning(disable : 4305)

// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

/**
 * @file platform_type.h
 * Defines supported target platform types.
 */
#if defined(__amd64__) || defined(_M_AMD64)
      #define X64
#elif defined(__X86__) || defined(_M_IX86)
      #define X86
#elif defined(__arm__) || defined(_M_ARM)
      #define ARM
#elif defined(__aarch64__) || defined(_M_ARM64)
      #define ARM64
#endif

#if defined(_WIN32) || defined(_WIN64)
      #define WINDOWS
#elif defined(__unix) || defined(__unix__)
      #define LINUX
#endif

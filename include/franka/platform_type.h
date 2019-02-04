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
#endif

#if defined(_WIN32) || defined(_WIN64)
      #define WINDOWS
#elif defined(__unix) || defined(__unix__)
      #define LINUX
#endif

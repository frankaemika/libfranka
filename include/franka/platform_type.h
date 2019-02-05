// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

/**
 * @file platform_type.h
 * Defines supported target platform types.
 */

#undef LIBFRANKA_X64
#undef LIBFRANKA_X86

#if defined(__amd64__) || defined(_M_AMD64)
#define LIBFRANKA_X64
#elif defined(__X86__) || defined(_M_IX86)
#define LIBFRANKA_X86
#endif

#undef LIBFRANKA_WINDOWS
#undef LIBFRANKA_LINUX

#if defined(_WIN32) || defined(_WIN64)
#define LIBFRANKA_WINDOWS
#elif defined(__unix) || defined(__unix__)
#define LIBFRANKA_LINUX
#endif

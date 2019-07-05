// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#undef LIBFRANKA_X64
#undef LIBFRANKA_X86
#undef LIBFRANKA_ARM64
#undef LIBFRANKA_ARM

#if defined(__amd64__) || defined(_M_AMD64)
#define LIBFRANKA_X64
#elif defined(__X86__) || defined(_M_IX86)
#define LIBFRANKA_X86
#elif defined(__aarch64__) || defined(_M_ARM64)
#define LIBFRANKA_ARM64
#elif defined(__arm__) || defined(_M_ARM)
#define LIBFRANKA_ARM
#endif

#undef LIBFRANKA_WINDOWS
#undef LIBFRANKA_LINUX

#if defined(_WIN32) || defined(_WIN64)
#define LIBFRANKA_WINDOWS
#elif defined(__unix) || defined(__unix__)
#define LIBFRANKA_LINUX
#endif

// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

extern "C" {

void Ji_J_J1(double b_Ji_J_J1[42]);
void Ji_J_J2(const double q[7], double b_Ji_J_J2[42]);
void Ji_J_J3(const double q[7], double b_Ji_J_J3[42]);
void Ji_J_J4(const double q[7], double b_Ji_J_J4[42]);
void Ji_J_J5(const double q[7], double b_Ji_J_J5[42]);
void Ji_J_J6(const double q[7], double b_Ji_J_J6[42]);
void Ji_J_J7(const double q[7], double b_Ji_J_J7[42]);
void Ji_J_J8(const double q[7], double b_Ji_J_J8[42]);
void Ji_J_J9(const double q[7], const double F_T_EE[16], double b_Ji_J_J9[42]);

void O_J_J1(double b_O_J_J1[42]);
void O_J_J2(const double q[7], double b_O_J_J2[42]);
void O_J_J3(const double q[7], double b_O_J_J3[42]);
void O_J_J4(const double q[7], double b_O_J_J4[42]);
void O_J_J5(const double q[7], double b_O_J_J5[42]);
void O_J_J6(const double q[7], double b_O_J_J6[42]);
void O_J_J7(const double q[7], double b_O_J_J7[42]);
void O_J_J8(const double q[7], double b_O_J_J8[42]);
void O_J_J9(const double q[7], const double F_T_EE[16], double b_O_J_J9[42]);

void O_T_J1(const double q[7], double b_O_T_J1[16]);
void O_T_J2(const double q[7], double b_O_T_J2[16]);
void O_T_J3(const double q[7], double b_O_T_J3[16]);
void O_T_J4(const double q[7], double b_O_T_J4[16]);
void O_T_J5(const double q[7], double b_O_T_J5[16]);
void O_T_J6(const double q[7], double b_O_T_J6[16]);
void O_T_J7(const double q[7], double b_O_T_J7[16]);
void O_T_J8(const double q[7], double b_O_T_J8[16]);
void O_T_J9(const double q[7], const double F_T_EE[16], double b_O_T_J9[16]);
}

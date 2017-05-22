#pragma once

extern "C" {

void M_NE_file(const double q[7],  // NOLINT
               const double I_load[9],
               double m_load,
               const double F_x_Cload[16],
               double M_NE[49]);
void O_T_J1_file(const double q[7],  // NOLINT
                 const double F_T_EE[16],
                 double O_T_J1[16]);
void O_T_J2_file(const double q[7],  // NOLINT
                 const double F_T_EE[16],
                 double O_T_J2[16]);
void O_T_J3_file(const double q[7],  // NOLINT
                 const double F_T_EE[16],
                 double O_T_J3[16]);
void O_T_J4_file(const double q[7],  // NOLINT
                 const double F_T_EE[16],
                 double O_T_J4[16]);
void O_T_J5_file(const double q[7],  // NOLINT
                 const double F_T_EE[16],
                 double O_T_J5[16]);
void O_T_J6_file(const double q[7],  // NOLINT
                 const double F_T_EE[16],
                 double O_T_J6[16]);
void O_T_J7_file(const double q[7],  // NOLINT
                 const double F_T_EE[16],
                 double O_T_J7[16]);
void O_T_J8_file(const double q[7],  // NOLINT
                 const double F_T_EE[16],
                 double O_T_J8[16]);
void O_T_J9_file(const double q[7],  // NOLINT
                 const double F_T_EE[16],
                 double O_T_J9[16]);
void c_NE_file(const double q[7],  // NOLINT
               const double dq[7],
               const double I_load[9],
               double m_load,
               const double F_x_Cload[16],
               double c_NE[7]);
void g_NE_file(const double q[7],  // NOLINT
               const double g_earth[3],
               double m_load,
               const double F_x_Cload[16],
               double g_NE[7]);
void libfcimodels_initialize(void);  // NOLINT

}

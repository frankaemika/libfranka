#include <libfcimodels.h>

#include "model_library_interface.h"

ModelLibraryInterface* model_library_interface;

void M_NE_file(const double q[7],
               const double I_load[9],
               double m_load,
               const double F_x_Cload[3],
               double M_NE[49]) {
  if (model_library_interface) {
    model_library_interface->M_NE(q, I_load, m_load, F_x_Cload, M_NE);
  }
}

void O_T_J1_file(const double q[7], const double F_T_EE[16], double O_T_J1[16]) {
  if (model_library_interface) {
    model_library_interface->O_T_J1(q, F_T_EE, O_T_J1);
  }
}

void O_T_J2_file(const double q[7], const double F_T_EE[16], double O_T_J2[16]) {
  if (model_library_interface) {
    model_library_interface->O_T_J2(q, F_T_EE, O_T_J2);
  }
}

void O_T_J3_file(const double q[7], const double F_T_EE[16], double O_T_J3[16]) {
  if (model_library_interface) {
    model_library_interface->O_T_J3(q, F_T_EE, O_T_J3);
  }
}

void O_T_J4_file(const double q[7], const double F_T_EE[16], double O_T_J4[16]) {
  if (model_library_interface) {
    model_library_interface->O_T_J4(q, F_T_EE, O_T_J4);
  }
}

void O_T_J5_file(const double q[7], const double F_T_EE[16], double O_T_J5[16]) {
  if (model_library_interface) {
    model_library_interface->O_T_J5(q, F_T_EE, O_T_J5);
  }
}

void O_T_J6_file(const double q[7], const double F_T_EE[16], double O_T_J6[16]) {
  if (model_library_interface) {
    model_library_interface->O_T_J6(q, F_T_EE, O_T_J6);
  }
}

void O_T_J7_file(const double q[7], const double F_T_EE[16], double O_T_J7[16]) {
  if (model_library_interface) {
    model_library_interface->O_T_J7(q, F_T_EE, O_T_J7);
  }
}

void O_T_J8_file(const double q[7], const double F_T_EE[16], double O_T_J8[16]) {
  if (model_library_interface) {
    model_library_interface->O_T_J8(q, F_T_EE, O_T_J8);
  }
}

void O_T_J9_file(const double q[7], const double F_T_EE[16], double O_T_J9[16]) {
  if (model_library_interface) {
    model_library_interface->O_T_J9(q, F_T_EE, O_T_J9);
  }
}

void c_NE_file(const double q[7],
               const double dq[7],
               const double I_load[9],
               double m_load,
               const double F_x_Cload[3],
               double c_NE[7]) {
  if (model_library_interface) {
    model_library_interface->c_NE(q, dq, I_load, m_load, F_x_Cload, c_NE);
  }
}

void g_NE_file(const double q[7],
               const double g_earth[3],
               double m_load,
               const double F_x_Cload[3],
               double g_NE[7]) {
  if (model_library_interface) {
    model_library_interface->g_NE(q, g_earth, m_load, F_x_Cload, g_NE);
  }
}

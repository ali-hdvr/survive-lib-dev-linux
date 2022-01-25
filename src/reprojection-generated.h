#pragma once

#include "redist/linmath.h"
#include <float.h>
#include <math.h>
#include <stdbool.h>
#include "objects.h"

#define GEN_FLT FLT

// Jacobian of reproject_gen2 wrt [obj_px, obj_py, obj_pz, obj_qi, obj_qj, obj_qk]
static inline void gen_reproject_gen2_jac_obj_p_axis_angle(FLT *out, const LinmathAxisAnglePose *obj_p,
                                                           const FLT *sensor_pt, const LinmathAxisAnglePose *lh_p,
                                                           const int lh)
{
  // printf("executing %s\n", __FUNCTION__);
  const GEN_FLT obj_px = (*obj_p).Pos[0];
  const GEN_FLT obj_py = (*obj_p).Pos[1];
  const GEN_FLT obj_pz = (*obj_p).Pos[2];
  const GEN_FLT obj_qi = (*obj_p).AxisAngleRot[0];
  const GEN_FLT obj_qj = (*obj_p).AxisAngleRot[1];
  const GEN_FLT obj_qk = (*obj_p).AxisAngleRot[2];
  const GEN_FLT sensor_x = sensor_pt[0];
  const GEN_FLT sensor_y = sensor_pt[1];
  const GEN_FLT sensor_z = sensor_pt[2];
  const GEN_FLT lh_px = (*lh_p).Pos[0];
  const GEN_FLT lh_py = (*lh_p).Pos[1];
  const GEN_FLT lh_pz = (*lh_p).Pos[2];
  const GEN_FLT lh_qi = (*lh_p).AxisAngleRot[0];
  const GEN_FLT lh_qj = (*lh_p).AxisAngleRot[1];
  const GEN_FLT lh_qk = (*lh_p).AxisAngleRot[2];
  const GEN_FLT phase_0 = lighthouses[lh].ootx_handler.fcal_phase[0];
  const GEN_FLT tilt_0 = lighthouses[lh].ootx_handler.fcal_tilt[0];
  const GEN_FLT curve_0 = lighthouses[lh].ootx_handler.fcal_curve[0];
  const GEN_FLT gibPhase_0 = lighthouses[lh].ootx_handler.fcal_gibphase[0];
  const GEN_FLT gibMag_0 = lighthouses[lh].ootx_handler.fcal_gibmag[0];
  const GEN_FLT ogeeMag_0 = lighthouses[lh].ootx_handler.fcal_ogeemag[0];
  const GEN_FLT ogeePhase_0 = lighthouses[lh].ootx_handler.fcal_ogeephase[0];
  const GEN_FLT phase_1 = lighthouses[lh].ootx_handler.fcal_phase[1];
  const GEN_FLT tilt_1 = lighthouses[lh].ootx_handler.fcal_tilt[1];
  const GEN_FLT curve_1 = lighthouses[lh].ootx_handler.fcal_curve[1];
  const GEN_FLT gibPhase_1 = lighthouses[lh].ootx_handler.fcal_gibphase[1];
  const GEN_FLT gibMag_1 = lighthouses[lh].ootx_handler.fcal_gibmag[1];
  const GEN_FLT ogeeMag_1 = lighthouses[lh].ootx_handler.fcal_ogeemag[1];
  const GEN_FLT ogeePhase_1 = lighthouses[lh].ootx_handler.fcal_ogeephase[1];
  const GEN_FLT x0 = lh_qk * lh_qk;
  const GEN_FLT x1 = lh_qj * lh_qj;
  const GEN_FLT x2 = lh_qi * lh_qi;
  const GEN_FLT x3 = 1e-10 + x2 + x1 + x0;
  const GEN_FLT x4 = sqrt(x3);
  const GEN_FLT x5 = (1. / x4) * sin(x4);
  const GEN_FLT x6 = x5 * lh_qj;
  const GEN_FLT x7 = cos(x4);
  const GEN_FLT x8 = (1. / x3) * (1 + (-1 * x7));
  const GEN_FLT x9 = x8 * lh_qk * lh_qi;
  const GEN_FLT x10 = x9 + (-1 * x6);
  const GEN_FLT x11 = x9 + x6;
  const GEN_FLT x12 = obj_qk * obj_qk;
  const GEN_FLT x13 = obj_qj * obj_qj;
  const GEN_FLT x14 = obj_qi * obj_qi;
  const GEN_FLT x15 = 1e-10 + x14 + x13 + x12;
  const GEN_FLT x16 = 1. / x15;
  const GEN_FLT x17 = sqrt(x15);
  const GEN_FLT x18 = cos(x17);
  const GEN_FLT x19 = 1 + (-1 * x18);
  const GEN_FLT x20 = x19 * x16;
  const GEN_FLT x21 = sin(x17);
  const GEN_FLT x22 = x21 * (1. / x17);
  const GEN_FLT x23 = x22 * obj_qi;
  const GEN_FLT x24 = x20 * obj_qj;
  const GEN_FLT x25 = x24 * obj_qk;
  const GEN_FLT x26 = x22 * obj_qj;
  const GEN_FLT x27 = -1 * x26;
  const GEN_FLT x28 = x20 * obj_qi;
  const GEN_FLT x29 = x28 * obj_qk;
  const GEN_FLT x30 = ((x29 + x27) * sensor_x) + ((x25 + x23) * sensor_y) + ((x18 + (x20 * x12)) * sensor_z) + obj_pz;
  const GEN_FLT x31 = -1 * x23;
  const GEN_FLT x32 = x22 * obj_qk;
  const GEN_FLT x33 = x24 * obj_qi;
  const GEN_FLT x34 = ((x33 + x32) * sensor_x) + ((x25 + x31) * sensor_z) + ((x18 + (x20 * x13)) * sensor_y) + obj_py;
  const GEN_FLT x35 = x5 * lh_qk;
  const GEN_FLT x36 = x8 * lh_qj;
  const GEN_FLT x37 = x36 * lh_qi;
  const GEN_FLT x38 = x37 + (-1 * x35);
  const GEN_FLT x39 = -1 * x32;
  const GEN_FLT x40 = ((x18 + (x20 * x14)) * sensor_x) + ((x33 + x39) * sensor_y) + ((x29 + x26) * sensor_z) + obj_px;
  const GEN_FLT x41 = x7 + (x2 * x8);
  const GEN_FLT x42 = (x40 * x41) + (x30 * x11) + (x34 * x38) + lh_px;
  const GEN_FLT x43 = 1. / x42;
  const GEN_FLT x44 = x7 + (x0 * x8);
  const GEN_FLT x45 = x5 * lh_qi;
  const GEN_FLT x46 = x36 * lh_qk;
  const GEN_FLT x47 = x46 + x45;
  const GEN_FLT x48 = (x40 * x10) + (x47 * x34) + (x44 * x30) + lh_pz;
  const GEN_FLT x49 = x42 * x42;
  const GEN_FLT x50 = x48 * (1. / x49);
  const GEN_FLT x51 = x49 + (x48 * x48);
  const GEN_FLT x52 = 1. / x51;
  const GEN_FLT x53 = x52 * x49;
  const GEN_FLT x54 = ((x50 * x41) + (-1 * x43 * x10)) * x53;
  const GEN_FLT x55 = x46 + (-1 * x45);
  const GEN_FLT x56 = x7 + (x1 * x8);
  const GEN_FLT x57 = x37 + x35;
  const GEN_FLT x58 = (x57 * x40) + lh_py + (x56 * x34) + (x55 * x30);
  const GEN_FLT x59 = x58 * x58;
  const GEN_FLT x60 = x51 + x59;
  const GEN_FLT x61 = 1. / sqrt(x60);
  const GEN_FLT x62 = 0.523598775598299 + tilt_0;
  const GEN_FLT x63 = cos(x62);
  const GEN_FLT x64 = 1. / x63;
  const GEN_FLT x65 = x64 * x61;
  const GEN_FLT x66 = asin(x65 * x58);
  const GEN_FLT x67 = 8.0108022e-06 * x66;
  const GEN_FLT x68 = -8.0108022e-06 + (-1 * x67);
  const GEN_FLT x69 = 0.0028679863 + (x68 * x66);
  const GEN_FLT x70 = 5.3685255e-06 + (x66 * x69);
  const GEN_FLT x71 = 0.0076069798 + (x70 * x66);
  const GEN_FLT x72 = x71 * x66;
  const GEN_FLT x73 = -8.0108022e-06 + (-1.60216044e-05 * x66);
  const GEN_FLT x74 = x69 + (x73 * x66);
  const GEN_FLT x75 = x70 + (x74 * x66);
  const GEN_FLT x76 = x71 + (x75 * x66);
  const GEN_FLT x77 = (x76 * x66) + x72;
  const GEN_FLT x78 = 1. / sqrt(x51);
  const GEN_FLT x79 = tan(x62);
  const GEN_FLT x80 = x79 * x78;
  const GEN_FLT x81 = x80 * x58;
  const GEN_FLT x82 = atan2(-1 * x48, x42);
  const GEN_FLT x83 = x82 + (-1 * asin(x81)) + ogeeMag_0;
  const GEN_FLT x84 = (sin(x83) * ogeePhase_0) + curve_0;
  const GEN_FLT x85 = sin(x62);
  const GEN_FLT x86 = x84 * x85;
  const GEN_FLT x87 = x63 + (-1 * x86 * x77);
  const GEN_FLT x88 = 1. / x87;
  const GEN_FLT x89 = x66 * x66;
  const GEN_FLT x90 = x89 * x84;
  const GEN_FLT x91 = x88 * x90;
  const GEN_FLT x92 = x81 + (x71 * x91);
  const GEN_FLT x93 = 1. / sqrt(1 + (-1 * (x92 * x92)));
  const GEN_FLT x94 = (1. / x60) * x59;
  const GEN_FLT x95 = 1. / sqrt(1 + (-1 * (1. / (x63 * x63)) * x94));
  const GEN_FLT x96 = 2 * x58;
  const GEN_FLT x97 = 2 * x42;
  const GEN_FLT x98 = 2 * x48;
  const GEN_FLT x99 = (x98 * x10) + (x97 * x41);
  const GEN_FLT x100 = x99 + (x57 * x96);
  const GEN_FLT x101 = 1.0 / 2.0 * x58;
  const GEN_FLT x102 = (1. / (x60 * sqrt(x60))) * x101;
  const GEN_FLT x103 = x64 * x102;
  const GEN_FLT x104 = (x65 * x57) + (-1 * x100 * x103);
  const GEN_FLT x105 = x95 * x104;
  const GEN_FLT x106 = 2 * x88 * x84 * x72;
  const GEN_FLT x107 = x68 * x95;
  const GEN_FLT x108 = x104 * x107;
  const GEN_FLT x109 = 2.40324066e-05 * x66;
  const GEN_FLT x110 = (x66 * (x108 + (-1 * x67 * x105))) + (x69 * x105);
  const GEN_FLT x111 = (x66 * x110) + (x70 * x105);
  const GEN_FLT x112 = x52 * x59;
  const GEN_FLT x113 = 1. / sqrt(1 + (-1 * (x79 * x79) * x112));
  const GEN_FLT x114 = (1. / (x51 * sqrt(x51))) * x101;
  const GEN_FLT x115 = x99 * x114;
  const GEN_FLT x116 = x78 * x57;
  const GEN_FLT x117 = (x79 * x116) + (-1 * x79 * x115);
  const GEN_FLT x118 = cos(x83) * ogeePhase_0;
  const GEN_FLT x119 = x118 * ((-1 * x113 * x117) + x54);
  const GEN_FLT x120 = x85 * x77;
  const GEN_FLT x121 = (1. / (x87 * x87)) * x71 * x90;
  const GEN_FLT x122 = x88 * x89 * x71;
  const GEN_FLT x123 =
      x93 * (x117 + (x91 * x111) + (x119 * x122) +
             (-1 * x121 *
              ((-1 * x119 * x120) +
               (-1 * x86 *
                ((x66 * x111) + (x71 * x105) +
                 (x66 * (x111 + (x66 * ((x74 * x105) + x110 + (x66 * ((x73 * x105) + (-1 * x109 * x105) + x108)))) +
                         (x75 * x105))) +
                 (x76 * x105))))) +
             (x105 * x106));
  const GEN_FLT x124 = -1 * x54;
  const GEN_FLT x125 = -1 * x82;
  const GEN_FLT x126 = cos(x125 + asin(x92) + (-1 * gibPhase_0)) * gibMag_0;
  const GEN_FLT x127 = ((x50 * x38) + (-1 * x43 * x47)) * x53;
  const GEN_FLT x128 = (x98 * x47) + (x97 * x38);
  const GEN_FLT x129 = x128 + (x56 * x96);
  const GEN_FLT x130 = (x65 * x56) + (-1 * x103 * x129);
  const GEN_FLT x131 = x95 * x130;
  const GEN_FLT x132 = x107 * x130;
  const GEN_FLT x133 = (x66 * (x132 + (-1 * x67 * x131))) + (x69 * x131);
  const GEN_FLT x134 = (x66 * x133) + (x70 * x131);
  const GEN_FLT x135 = x79 * x114;
  const GEN_FLT x136 = (x80 * x56) + (-1 * x128 * x135);
  const GEN_FLT x137 = (-1 * x113 * x136) + x127;
  const GEN_FLT x138 = x118 * x120;
  const GEN_FLT x139 = x118 * x122;
  const GEN_FLT x140 =
      x93 * (x136 + (x137 * x139) +
             (-1 * x121 *
              ((-1 * x137 * x138) +
               (-1 * x86 *
                ((x66 * x134) + (x71 * x131) +
                 (x66 * (x134 + (x66 * (x133 + (x74 * x131) + (x66 * ((x73 * x131) + (-1 * x109 * x131) + x132)))) +
                         (x75 * x131))) +
                 (x76 * x131))))) +
             (x91 * x134) + (x106 * x131));
  const GEN_FLT x141 = -1 * x127;
  const GEN_FLT x142 = ((x50 * x11) + (-1 * x43 * x44)) * x53;
  const GEN_FLT x143 = (x98 * x44) + (x97 * x11);
  const GEN_FLT x144 = x143 + (x55 * x96);
  const GEN_FLT x145 = (x65 * x55) + (-1 * x103 * x144);
  const GEN_FLT x146 = x95 * x145;
  const GEN_FLT x147 = (x80 * x55) + (-1 * x135 * x143);
  const GEN_FLT x148 = (-1 * x113 * x147) + x142;
  const GEN_FLT x149 = x107 * x145;
  const GEN_FLT x150 = (x66 * (x149 + (-1 * x67 * x146))) + (x69 * x146);
  const GEN_FLT x151 = (x66 * x150) + (x70 * x146);
  const GEN_FLT x152 =
      x93 * (x147 +
             (-1 * x121 *
              ((-1 * x138 * x148) +
               (-1 * x86 *
                ((x66 * x151) + (x71 * x146) +
                 (x66 * (x151 + (x66 * (x150 + (x74 * x146) + (x66 * ((x73 * x146) + (-1 * x109 * x146) + x149)))) +
                         (x75 * x146))) +
                 (x76 * x146))))) +
             (x91 * x151) + (x139 * x148) + (x106 * x146));
  const GEN_FLT x153 = -1 * x142;
  const GEN_FLT x154 = obj_qi * obj_qi * obj_qi;
  const GEN_FLT x155 = x21 * (1. / (x15 * sqrt(x15)));
  const GEN_FLT x156 = 2 * (1. / (x15 * x15)) * x19;
  const GEN_FLT x157 = x18 * x16;
  const GEN_FLT x158 = x157 * obj_qi;
  const GEN_FLT x159 = x158 * obj_qk;
  const GEN_FLT x160 = x155 * obj_qk * obj_qi;
  const GEN_FLT x161 = x160 + (-1 * x159);
  const GEN_FLT x162 = x14 * x155;
  const GEN_FLT x163 = x156 * obj_qj;
  const GEN_FLT x164 = (-1 * x14 * x163) + (x162 * obj_qj);
  const GEN_FLT x165 = x164 + x24;
  const GEN_FLT x166 = x20 * obj_qk;
  const GEN_FLT x167 = x156 * obj_qk;
  const GEN_FLT x168 = (-1 * x14 * x167) + (x162 * obj_qk);
  const GEN_FLT x169 = x168 + x166;
  const GEN_FLT x170 = x158 * obj_qj;
  const GEN_FLT x171 = x155 * obj_qj;
  const GEN_FLT x172 = x171 * obj_qi;
  const GEN_FLT x173 = (-1 * x172) + x170;
  const GEN_FLT x174 = ((x173 + x169) * sensor_z) + ((x165 + x161) * sensor_y) +
                       (((-1 * x154 * x156) + (x154 * x155) + (2 * x28) + x31) * sensor_x);
  const GEN_FLT x175 = (-1 * x160) + x159;
  const GEN_FLT x176 = x13 * x155;
  const GEN_FLT x177 = x156 * obj_qi;
  const GEN_FLT x178 = (-1 * x13 * x177) + (x176 * obj_qi);
  const GEN_FLT x179 = x14 * x157;
  const GEN_FLT x180 = x171 * obj_qk;
  const GEN_FLT x181 = obj_qk * obj_qj;
  const GEN_FLT x182 = (-1 * x177 * x181) + (x180 * obj_qi);
  const GEN_FLT x183 = x182 + (-1 * x22);
  const GEN_FLT x184 =
      ((x183 + x162 + (-1 * x179)) * sensor_z) + ((x178 + x31) * sensor_y) + ((x165 + x175) * sensor_x);
  const GEN_FLT x185 = x172 + (-1 * x170);
  const GEN_FLT x186 = x182 + x22;
  const GEN_FLT x187 = x12 * x155;
  const GEN_FLT x188 = (-1 * x12 * x177) + (x187 * obj_qi);
  const GEN_FLT x189 =
      ((x188 + x31) * sensor_z) + ((x186 + (-1 * x162) + x179) * sensor_y) + ((x185 + x169) * sensor_x);
  const GEN_FLT x190 = (x47 * x184) + (x44 * x189) + (x10 * x174);
  const GEN_FLT x191 = (x11 * x189) + (x38 * x184) + (x41 * x174);
  const GEN_FLT x192 = ((x50 * x191) + (-1 * x43 * x190)) * x53;
  const GEN_FLT x193 = (x55 * x189) + (x56 * x184) + (x57 * x174);
  const GEN_FLT x194 = (x98 * x190) + (x97 * x191);
  const GEN_FLT x195 = x194 + (x96 * x193);
  const GEN_FLT x196 = x95 * ((x65 * x193) + (-1 * x103 * x195));
  const GEN_FLT x197 = x68 * x196;
  const GEN_FLT x198 = (x66 * (x197 + (-1 * x67 * x196))) + (x69 * x196);
  const GEN_FLT x199 = (x66 * x198) + (x70 * x196);
  const GEN_FLT x200 = (x80 * x193) + (-1 * x194 * x135);
  const GEN_FLT x201 = (-1 * x200 * x113) + x192;
  const GEN_FLT x202 =
      x93 * (x200 + (x91 * x199) + (x201 * x139) +
             (-1 * x121 *
              ((-1 * x201 * x138) +
               (-1 * x86 *
                ((x71 * x196) +
                 (x66 * (x199 + (x66 * (x198 + (x74 * x196) + (x66 * ((x73 * x196) + (-1 * x109 * x196) + x197)))) +
                         (x75 * x196))) +
                 (x66 * x199) + (x76 * x196))))) +
             (x106 * x196));
  const GEN_FLT x203 = -1 * x192;
  const GEN_FLT x204 = x181 * x157;
  const GEN_FLT x205 = (-1 * x180) + x204;
  const GEN_FLT x206 = x178 + x28;
  const GEN_FLT x207 = obj_qj * obj_qj * obj_qj;
  const GEN_FLT x208 = (-1 * x13 * x167) + (x176 * obj_qk);
  const GEN_FLT x209 = x208 + x166;
  const GEN_FLT x210 = ((x209 + x185) * sensor_z) +
                       (((x207 * x155) + (-1 * x207 * x156) + (2 * x24) + x27) * sensor_y) +
                       ((x206 + x205) * sensor_x);
  const GEN_FLT x211 = x180 + (-1 * x204);
  const GEN_FLT x212 = x13 * x157;
  const GEN_FLT x213 =
      ((x186 + (-1 * x176) + x212) * sensor_z) + ((x211 + x206) * sensor_y) + ((x164 + x27) * sensor_x);
  const GEN_FLT x214 = (-1 * x12 * x163) + (x187 * obj_qj);
  const GEN_FLT x215 =
      ((x214 + x27) * sensor_z) + ((x209 + x173) * sensor_y) + ((x183 + x176 + (-1 * x212)) * sensor_x);
  const GEN_FLT x216 = (x44 * x215) + (x10 * x213) + (x47 * x210);
  const GEN_FLT x217 = (x11 * x215) + (x38 * x210) + (x41 * x213);
  const GEN_FLT x218 = ((x50 * x217) + (-1 * x43 * x216)) * x53;
  const GEN_FLT x219 = (x55 * x215) + (x56 * x210) + (x57 * x213);
  const GEN_FLT x220 = (x98 * x216) + (x97 * x217);
  const GEN_FLT x221 = x220 + (x96 * x219);
  const GEN_FLT x222 = x95 * ((x65 * x219) + (-1 * x221 * x103));
  const GEN_FLT x223 = x68 * x222;
  const GEN_FLT x224 = (x66 * (x223 + (-1 * x67 * x222))) + (x69 * x222);
  const GEN_FLT x225 = (x66 * x224) + (x70 * x222);
  const GEN_FLT x226 = (x80 * x219) + (-1 * x220 * x135);
  const GEN_FLT x227 = (-1 * x226 * x113) + x218;
  const GEN_FLT x228 =
      x93 * (x226 + (x91 * x225) +
             (-1 * x121 *
              ((-1 * x227 * x138) +
               (-1 * x86 *
                ((x66 * x225) + (x71 * x222) +
                 (x66 * (x225 + (x66 * (x224 + (x74 * x222) + (x66 * ((x73 * x222) + (-1 * x222 * x109) + x223)))) +
                         (x75 * x222))) +
                 (x76 * x222))))) +
             (x227 * x139) + (x222 * x106));
  const GEN_FLT x229 = -1 * x218;
  const GEN_FLT x230 = x12 * x157;
  const GEN_FLT x231 = x188 + x28;
  const GEN_FLT x232 =
      ((x183 + x187 + (-1 * x230)) * sensor_y) + ((x205 + x231) * sensor_z) + ((x168 + x39) * sensor_x);
  const GEN_FLT x233 = x214 + x24;
  const GEN_FLT x234 =
      ((x161 + x233) * sensor_z) + ((x208 + x39) * sensor_y) + ((x186 + (-1 * x187) + x230) * sensor_x);
  const GEN_FLT x235 = obj_qk * obj_qk * obj_qk;
  const GEN_FLT x236 = (((-1 * x235 * x156) + (x235 * x155) + x39 + (2 * x166)) * sensor_z) +
                       ((x175 + x233) * sensor_y) + ((x211 + x231) * sensor_x);
  const GEN_FLT x237 = (x44 * x236) + (x47 * x234) + (x10 * x232);
  const GEN_FLT x238 = (x11 * x236) + (x38 * x234) + (x41 * x232);
  const GEN_FLT x239 = ((x50 * x238) + (-1 * x43 * x237)) * x53;
  const GEN_FLT x240 = (x55 * x236) + (x56 * x234) + (x57 * x232);
  const GEN_FLT x241 = (x98 * x237) + (x97 * x238);
  const GEN_FLT x242 = x241 + (x96 * x240);
  const GEN_FLT x243 = x95 * ((x65 * x240) + (-1 * x242 * x103));
  const GEN_FLT x244 = x68 * x243;
  const GEN_FLT x245 = (x66 * (x244 + (-1 * x67 * x243))) + (x69 * x243);
  const GEN_FLT x246 = (x66 * x245) + (x70 * x243);
  const GEN_FLT x247 = (x80 * x240) + (-1 * x241 * x135);
  const GEN_FLT x248 = (-1 * x247 * x113) + x239;
  const GEN_FLT x249 =
      x93 * ((x91 * x246) + x247 + (x248 * x139) +
             (-1 * x121 *
              ((-1 * x248 * x138) +
               (-1 * x86 *
                ((x71 * x243) +
                 (x66 * (x246 + (x66 * (x245 + (x74 * x243) + (x66 * ((x73 * x243) + (-1 * x243 * x109) + x244)))) +
                         (x75 * x243))) +
                 (x66 * x246) + (x76 * x243))))) +
             (x243 * x106));
  const GEN_FLT x250 = -1 * x239;
  const GEN_FLT x251 = 0.523598775598299 + (-1 * tilt_1);
  const GEN_FLT x252 = cos(x251);
  const GEN_FLT x253 = 1. / x252;
  const GEN_FLT x254 = x61 * x253;
  const GEN_FLT x255 = asin(x58 * x254);
  const GEN_FLT x256 = 8.0108022e-06 * x255;
  const GEN_FLT x257 = -8.0108022e-06 + (-1 * x256);
  const GEN_FLT x258 = 0.0028679863 + (x255 * x257);
  const GEN_FLT x259 = 5.3685255e-06 + (x255 * x258);
  const GEN_FLT x260 = 0.0076069798 + (x255 * x259);
  const GEN_FLT x261 = x260 * x255;
  const GEN_FLT x262 = -8.0108022e-06 + (-1.60216044e-05 * x255);
  const GEN_FLT x263 = x258 + (x262 * x255);
  const GEN_FLT x264 = x259 + (x263 * x255);
  const GEN_FLT x265 = x260 + (x264 * x255);
  const GEN_FLT x266 = (x265 * x255) + x261;
  const GEN_FLT x267 = tan(x251);
  const GEN_FLT x268 = x78 * x267;
  const GEN_FLT x269 = -1 * x58 * x268;
  const GEN_FLT x270 = x82 + ogeeMag_1 + (-1 * asin(x269));
  const GEN_FLT x271 = (sin(x270) * ogeePhase_1) + curve_1;
  const GEN_FLT x272 = sin(x251);
  const GEN_FLT x273 = x272 * x271;
  const GEN_FLT x274 = x252 + (x273 * x266);
  const GEN_FLT x275 = 1. / x274;
  const GEN_FLT x276 = x255 * x255;
  const GEN_FLT x277 = x276 * x271;
  const GEN_FLT x278 = x277 * x275;
  const GEN_FLT x279 = x269 + (x278 * x260);
  const GEN_FLT x280 = 1. / sqrt(1 + (-1 * (x279 * x279)));
  const GEN_FLT x281 = 1. / sqrt(1 + (-1 * (x267 * x267) * x112));
  const GEN_FLT x282 = (-1 * x267 * x116) + (x267 * x115);
  const GEN_FLT x283 = cos(x270) * ogeePhase_1;
  const GEN_FLT x284 = x283 * ((-1 * x282 * x281) + x54);
  const GEN_FLT x285 = x276 * x275 * x260;
  const GEN_FLT x286 = 1. / sqrt(1 + (-1 * x94 * (1. / (x252 * x252))));
  const GEN_FLT x287 = x253 * x102;
  const GEN_FLT x288 = x286 * ((x57 * x254) + (-1 * x287 * x100));
  const GEN_FLT x289 = 2 * x275 * x271 * x261;
  const GEN_FLT x290 = x288 * x257;
  const GEN_FLT x291 = (x288 * x258) + (x255 * (x290 + (-1 * x288 * x256)));
  const GEN_FLT x292 = (x288 * x259) + (x291 * x255);
  const GEN_FLT x293 = x272 * x266;
  const GEN_FLT x294 = 2.40324066e-05 * x255;
  const GEN_FLT x295 = x277 * (1. / (x274 * x274)) * x260;
  const GEN_FLT x296 =
      x280 *
      (x282 +
       (-1 * x295 *
        ((x273 *
          ((x292 * x255) + (x260 * x288) +
           (x255 * (x292 + (x255 * (x291 + (x263 * x288) + (x255 * ((x262 * x288) + (-1 * x294 * x288) + x290)))) +
                    (x264 * x288))) +
           (x265 * x288))) +
         (x293 * x284))) +
       (x278 * x292) + (x288 * x289) + (x284 * x285));
  const GEN_FLT x297 = cos(x125 + asin(x279) + (-1 * gibPhase_1)) * gibMag_1;
  const GEN_FLT x298 = x267 * x114;
  const GEN_FLT x299 = (-1 * x56 * x268) + (x298 * x128);
  const GEN_FLT x300 = (-1 * x299 * x281) + x127;
  const GEN_FLT x301 = x283 * x285;
  const GEN_FLT x302 = x286 * ((x56 * x254) + (-1 * x287 * x129));
  const GEN_FLT x303 = x257 * x302;
  const GEN_FLT x304 = (x258 * x302) + (x255 * (x303 + (-1 * x256 * x302)));
  const GEN_FLT x305 = (x259 * x302) + (x255 * x304);
  const GEN_FLT x306 = x293 * x283;
  const GEN_FLT x307 =
      x280 *
      (x299 +
       (-1 * x295 *
        ((x273 *
          ((x255 * x305) + (x260 * x302) +
           (x255 * (x305 + (x255 * (x304 + (x263 * x302) + (x255 * ((x262 * x302) + (-1 * x294 * x302) + x303)))) +
                    (x264 * x302))) +
           (x265 * x302))) +
         (x300 * x306))) +
       (x289 * x302) + (x278 * x305) + (x301 * x300));
  const GEN_FLT x308 = (-1 * x55 * x268) + (x298 * x143);
  const GEN_FLT x309 = (-1 * x281 * x308) + x142;
  const GEN_FLT x310 = (x55 * x254) + (-1 * x287 * x144);
  const GEN_FLT x311 = x265 * x286;
  const GEN_FLT x312 = x286 * x310;
  const GEN_FLT x313 = x257 * x312;
  const GEN_FLT x314 = (x258 * x312) + (x255 * (x313 + (-1 * x256 * x312)));
  const GEN_FLT x315 = (x259 * x312) + (x255 * x314);
  const GEN_FLT x316 =
      x280 *
      ((x278 * x315) + x308 + (x289 * x312) +
       (-1 * x295 *
        ((x273 *
          ((x255 * x315) + (x260 * x312) +
           (x255 * (x315 + (x255 * (x314 + (x263 * x312) + (x255 * ((x262 * x312) + (-1 * x294 * x312) + x313)))) +
                    (x264 * x312))) +
           (x311 * x310))) +
         (x309 * x306))) +
       (x309 * x301));
  const GEN_FLT x317 = (-1 * x268 * x193) + (x298 * x194);
  const GEN_FLT x318 = (-1 * x281 * x317) + x192;
  const GEN_FLT x319 = (x254 * x193) + (-1 * x287 * x195);
  const GEN_FLT x320 = x286 * x319;
  const GEN_FLT x321 = x257 * x320;
  const GEN_FLT x322 = (x258 * x320) + (x255 * (x321 + (-1 * x256 * x320)));
  const GEN_FLT x323 = (x259 * x320) + (x255 * x322);
  const GEN_FLT x324 =
      x280 *
      (x317 +
       (-1 * x295 *
        ((x273 *
          ((x255 * x323) +
           (x255 * (x323 + (x255 * (x322 + (x263 * x320) + (x255 * ((x262 * x320) + (-1 * x294 * x320) + x321)))) +
                    (x264 * x320))) +
           (x260 * x320) + (x311 * x319))) +
         (x306 * x318))) +
       (x278 * x323) + (x289 * x320) + (x301 * x318));
  const GEN_FLT x325 = (-1 * x219 * x268) + (x298 * x220);
  const GEN_FLT x326 = (-1 * x281 * x325) + x218;
  const GEN_FLT x327 = (x219 * x254) + (-1 * x287 * x221);
  const GEN_FLT x328 = x286 * x327;
  const GEN_FLT x329 = x257 * x328;
  const GEN_FLT x330 = (x258 * x328) + (x255 * (x329 + (-1 * x256 * x328)));
  const GEN_FLT x331 = (x259 * x328) + (x255 * x330);
  const GEN_FLT x332 =
      x280 *
      ((-1 * x295 *
        ((x273 *
          ((x255 * x331) + (x260 * x328) +
           (x255 * (x331 + (x255 * (x330 + (x263 * x328) + (x255 * ((x262 * x328) + (-1 * x294 * x328) + x329)))) +
                    (x264 * x328))) +
           (x327 * x311))) +
         (x306 * x326))) +
       x325 + (x278 * x331) + (x289 * x328) + (x301 * x326));
  const GEN_FLT x333 = (-1 * x268 * x240) + (x298 * x241);
  const GEN_FLT x334 = (-1 * x281 * x333) + x239;
  const GEN_FLT x335 = ((x254 * x240) + (-1 * x287 * x242)) * x286;
  const GEN_FLT x336 = x257 * x335;
  const GEN_FLT x337 = (x258 * x335) + (x255 * (x336 + (-1 * x256 * x335)));
  const GEN_FLT x338 = (x259 * x335) + (x255 * x337);
  const GEN_FLT x339 =
      x280 *
      ((x278 * x338) + x333 +
       (-1 * x295 *
        ((x273 *
          ((x255 * x338) + (x260 * x335) +
           (x255 * (x338 + (x255 * (x337 + (x263 * x335) + (x255 * ((x262 * x335) + (-1 * x294 * x335) + x336)))) +
                    (x264 * x335))) +
           (x265 * x335))) +
         (x306 * x334))) +
       (x289 * x335) + (x301 * x334));
  out[0] = (-1 * (x123 + x124) * x126) + (-1 * x123) + x54;
  out[1] = (-1 * (x140 + x141) * x126) + (-1 * x140) + x127;
  out[2] = (-1 * (x152 + x153) * x126) + (-1 * x152) + x142;
  out[3] = (-1 * (x202 + x203) * x126) + (-1 * x202) + x192;
  out[4] = (-1 * x228) + (-1 * (x228 + x229) * x126) + x218;
  out[5] = (-1 * (x249 + x250) * x126) + (-1 * x249) + x239;
  out[6] = (-1 * x296) + (-1 * (x296 + x124) * x297) + x54;
  out[7] = (-1 * x307) + (-1 * (x307 + x141) * x297) + x127;
  out[8] = (-1 * x316) + (-1 * (x316 + x153) * x297) + x142;
  out[9] = (-1 * x324) + (-1 * (x324 + x203) * x297) + x192;
  out[10] = (-1 * x332) + (-1 * (x332 + x229) * x297) + x218;
  out[11] = (-1 * x339) + (-1 * (x339 + x250) * x297) + x239;
}
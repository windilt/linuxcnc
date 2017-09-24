// Copyright 2017 Dewey Garrett <dgarrett@panix.com>
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

#include "motion.h"
#include "hal.h"
#include "rtapi.h"
#include "rtapi.h"
#include "rtapi_app.h"
#include "rtapi_math.h"
#include "rtapi_string.h"

#define MY_MAX_JOINTS 3
#define NUMBER_OF_METHODS 2

static struct haldata {
    hal_u32_t *kins_method;
} *haldata;

static int    kmethod_changed = 0; // set when kinematics method change
                                   // detected in   kinematicsForward()
                                   // reset at next kinematicsInverse()

static int    kins_method; // kins_method halpin modulo NUMBER_OF_METHODS
static int    old_kins_method;

static double correction0[EMCMOT_MAX_JOINTS];
static double correction1[EMCMOT_MAX_JOINTS];

//forward declarations ----------------------------------------------
static int k0_Forward(const double *joints
              ,EmcPose *pos
              ,const KINEMATICS_FORWARD_FLAGS *fflags
              ,KINEMATICS_INVERSE_FLAGS *iflags
              );
static int k1_Forward(const double *joints
              ,EmcPose *pos
              ,const KINEMATICS_FORWARD_FLAGS *fflags
              ,KINEMATICS_INVERSE_FLAGS *iflags
              );
static int k0_Inverse(const EmcPose *pos
              ,double *joints
              ,const KINEMATICS_INVERSE_FLAGS *iflags
              ,const KINEMATICS_FORWARD_FLAGS *fflags
              );
static int k1_Inverse(const EmcPose *pos
              ,double *joints
              ,const KINEMATICS_INVERSE_FLAGS *iflags
              ,const KINEMATICS_FORWARD_FLAGS *fflags
              );
//-------------------------------------------------------------------


static int k0_Forward(const double *joints
              ,EmcPose *pos
              ,const KINEMATICS_FORWARD_FLAGS *fflags
              ,KINEMATICS_INVERSE_FLAGS *iflags
              ) {
    pos->tran.x = joints[0];
    pos->tran.y = joints[1];
    pos->tran.z = joints[2];
    pos->a      = joints[3];
    pos->b      = joints[4];
    pos->c      = joints[5];
    pos->u      = joints[6];
    pos->v      = joints[7];
    pos->w      = joints[8];
    return 0;
}

static int k1_Forward(const double *joints
              ,EmcPose *pos
              ,const KINEMATICS_FORWARD_FLAGS *fflags
              ,KINEMATICS_INVERSE_FLAGS *iflags
              ) {
    pos->tran.x = joints[2]; // switch x,z
    pos->tran.y = joints[1];
    pos->tran.z = joints[0]; // switch x,z
    pos->a      = joints[3];
    pos->b      = joints[4];
    pos->c      = joints[5];
    pos->u      = joints[6];
    pos->v      = joints[7];
    pos->w      = joints[8];
    return 0;
}

static int compute_correction(const double *joints
                      ,const EmcPose *pos
                      ,const KINEMATICS_FORWARD_FLAGS *fflags
                      ,const KINEMATICS_INVERSE_FLAGS *iflags
                      ) {
    int jno;
    double jt0[EMCMOT_MAX_JOINTS];
    double jt1[EMCMOT_MAX_JOINTS];

    k0_Inverse(pos,jt0,iflags,fflags);
    k1_Inverse(pos,jt1,iflags,fflags);

    for (jno = 0; jno < MY_MAX_JOINTS; jno++) {
        correction0[jno] = joints[jno] -jt0[jno]; // add for method 0
        correction1[jno] = joints[jno] -jt1[jno]; // add for method 1
    }
    return 0;
}

int kinematicsForward(const double *joints
                     ,EmcPose *pos
                     ,const KINEMATICS_FORWARD_FLAGS *fflags
                     ,KINEMATICS_INVERSE_FLAGS *iflags
                     ) {
    int jno;
    double jt[EMCMOT_MAX_JOINTS];
    // Note: The first call to kinematicsForward() occurs
    //       in motion/control.c:do_forward_kins()
    //       (before the call to kinematicsInverse())
    kins_method = (*haldata->kins_method)%NUMBER_OF_METHODS;
    if (kins_method != old_kins_method) {
        kmethod_changed = 1;
        rtapi_print("\nForward, kmethod_changed old-->new: %d-->%d\n"
                   ,old_kins_method, kins_method);
        rtapi_print("Fxyz: %12.4f %12.4f %12.4f\n"
                   ,pos->tran.x, pos->tran.y, pos->tran.z);
        rtapi_print("F012: %12.4f %12.4f %12.4f\n"
                   ,joints[0],joints[1],joints[2]);
        compute_correction(joints,pos,fflags,iflags);
        old_kins_method = kins_method;
    }
    // remove (-) correction, then Forward: joints --> cartesian position
    switch (kins_method) {
        default: for (jno = 0; jno < MY_MAX_JOINTS; jno++) {
                     jt[jno] = joints[jno] - correction0[jno];
                 }
                 k0_Forward(jt,pos,fflags,iflags);
                 break;
        case 1:  for (jno = 0; jno < MY_MAX_JOINTS; jno++) {
                     jt[jno] = joints[jno] - correction1[jno];
                 }
                 k1_Forward(jt,pos,fflags,iflags);
                 break;
    }
    return 0;
}

static int k0_Inverse(const EmcPose *pos
              ,double *joints
              ,const KINEMATICS_INVERSE_FLAGS *iflags
              ,const KINEMATICS_FORWARD_FLAGS *fflags
              ) {
    joints[0] = pos->tran.x;
    joints[1] = pos->tran.y;
    joints[2] = pos->tran.z;
    joints[3] = pos->a;
    joints[4] = pos->b;
    joints[5] = pos->c;
    joints[6] = pos->u;
    joints[7] = pos->v;
    joints[8] = pos->w;
    return 0;
}

static int k1_Inverse(const EmcPose *pos
              ,double *joints
              ,const KINEMATICS_INVERSE_FLAGS *iflags
              ,const KINEMATICS_FORWARD_FLAGS *fflags
              ) {
    joints[0] = pos->tran.z; // switch x,z
    joints[1] = pos->tran.y;
    joints[2] = pos->tran.x; // switch x,z
    joints[3] = pos->a;
    joints[4] = pos->b;
    joints[5] = pos->c;
    joints[6] = pos->u;
    joints[7] = pos->v;
    joints[8] = pos->w;
    return 0;
}

int kinematicsInverse(const EmcPose *pos
                     ,double *joints
                     ,const KINEMATICS_INVERSE_FLAGS *iflags
                     ,KINEMATICS_FORWARD_FLAGS *fflags
                     ) {
    int jno;
    // Inverse: cartesian position --> joints, then apply (+=) correction
    switch (kins_method) {
        default: k0_Inverse(pos,joints,iflags,fflags);
                 for (jno = 0; jno < MY_MAX_JOINTS; jno++) {
                     joints[jno] += correction0[jno];
                 }
                 break;
        case 1:  k1_Inverse(pos,joints,iflags,fflags);
                 for (jno = 0; jno < MY_MAX_JOINTS; jno++) {
                     joints[jno] += correction1[jno];
                 }
                 break;
    }
    if (kmethod_changed > 0) {
        rtapi_print("Inverse, kmethod_changed kins_method=%d\n"
                   ,kins_method);
        kmethod_changed=0;
        rtapi_print("Ixyz: %12.4f %12.4f %12.4f\n"
                   ,pos->tran.x, pos->tran.y, pos->tran.z);
        rtapi_print("C012: %12.4f %12.4f %12.4f\n"
                   ,correction1[0],correction1[1],correction1[2]);
        rtapi_print("I012: %12.4f %12.4f %12.4f\n"
                   ,joints[0],joints[1],joints[2]);
        old_kins_method = kins_method;
        kmethod_changed = 0;
    }
    return 0;
}

int kinematicsHome(EmcPose *world
                  ,double *joint
                  ,KINEMATICS_FORWARD_FLAGS *fflags
                  ,KINEMATICS_INVERSE_FLAGS *iflags
                  ) {
    *fflags = 0;
    *iflags = 0;
    return kinematicsForward(joint, world, fflags, iflags);
}

KINEMATICS_TYPE kinematicsType() { return KINEMATICS_BOTH; }

EXPORT_SYMBOL(kinematicsType);
EXPORT_SYMBOL(kinematicsForward);
EXPORT_SYMBOL(kinematicsInverse);
MODULE_LICENSE("GPL");

static int comp_id;

int rtapi_app_main(void) {
    int  answer,jno;
    comp_id = hal_init("altkins");
    if(comp_id < 0) return comp_id;

    haldata = hal_malloc(sizeof(struct haldata));

    if((answer = hal_pin_u32_new("altkins.kins-method",
                                 HAL_IN,
                                 &(haldata->kins_method),
                                 comp_id)) < 0
                                ) goto error;
    for (jno = 0; jno < MY_MAX_JOINTS; jno++) {
        correction0[jno] = 0;
        correction1[jno] = 0;
    }

    hal_ready(comp_id);
    return 0;
error:
    hal_exit(comp_id);
    return answer;
}

void rtapi_app_exit(void) { hal_exit(comp_id); }

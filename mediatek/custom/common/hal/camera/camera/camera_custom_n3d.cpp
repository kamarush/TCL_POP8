
#include "camera_custom_n3d.h"

customSensorPos_N3D_t const&
getSensorPosN3D()
{
    static customSensorPos_N3D_t inst = {
        uSensorPos   : 1,   //0:LR 1:RL (L:tg1, R:tg2)
    };
    return inst;
}

MBOOL get_N3DFeatureFlag(void)
{
#ifdef MTK_NATIVE_3D_SUPPORT
    return MTRUE;
#else
    return MFALSE;
#endif
}

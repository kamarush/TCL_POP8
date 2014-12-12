





#include "gl_os.h"
#include "debug.h"
#include "wlan_lib.h"
#include "gl_wext.h"
#include "precomp.h"
#include <linux/poll.h>
#include <net/netlink.h>
#include <net/genetlink.h>

#if CFG_CHIP_RESET_SUPPORT

#define MAX_BIND_PROCESS    (4)

#define MTK_WIFI_FAMILY_NAME        "MTK_WIFI"
#define MTK_WIFI_RESET_START_NAME   "RESET_START"
#define MTK_WIFI_RESET_END_NAME     "RESET_END"
#define MTK_WIFI_RESET_TEST_NAME    "GENETLINK_START"


enum {
    __MTK_WIFI_ATTR_INVALID,
    MTK_WIFI_ATTR_MSG,
    __MTK_WIFI_ATTR_MAX,
};
#define MTK_WIFI_ATTR_MAX       (__MTK_WIFI_ATTR_MAX - 1)


enum {
    __MTK_WIFI_COMMAND_INVALID,
    MTK_WIFI_COMMAND_BIND,
    MTK_WIFI_COMMAND_RESET,
    __MTK_WIFI_COMMAND_MAX,
};
#define MTK_WIFI_COMMAND_MAX    (__MTK_WIFI_COMMAND_MAX - 1)


static UINT_32 mtk_wifi_seqnum = 0;
static int num_bind_process = 0;
static pid_t bind_pid[MAX_BIND_PROCESS];


/* attribute policy */
static struct nla_policy mtk_wifi_genl_policy[MTK_WIFI_ATTR_MAX + 1] = {
    [MTK_WIFI_ATTR_MSG] = { .type = NLA_NUL_STRING },
};

/* family definition */
static struct genl_family mtk_wifi_gnl_family = {
    .id         = GENL_ID_GENERATE,
    .hdrsize    = 0,
    .name       = MTK_WIFI_FAMILY_NAME,
    .version    = 1,
    .maxattr    = MTK_WIFI_ATTR_MAX,
};

/* forward declaration */
static int mtk_wifi_bind(
    struct sk_buff *skb,
    struct genl_info *info
    );

static int mtk_wifi_reset(
    struct sk_buff *skb,
    struct genl_info *info
    );

/* operation definition */
static struct genl_ops mtk_wifi_gnl_ops_bind = {
    .cmd = MTK_WIFI_COMMAND_BIND,
    .flags  = 0,
    .policy = mtk_wifi_genl_policy,
    .doit   = mtk_wifi_bind,
    .dumpit = NULL,
};

static struct genl_ops mtk_wifi_gnl_ops_reset = {
    .cmd = MTK_WIFI_COMMAND_RESET,
    .flags  = 0,
    .policy = mtk_wifi_genl_policy,
    .doit   = mtk_wifi_reset,
    .dumpit = NULL,
};



extern int
mtk_wcn_wmt_msgcb_reg(
    ENUM_WMTDRV_TYPE_T eType,
    PF_WMT_CB pCb);

extern int
mtk_wcn_wmt_msgcb_unreg(
    ENUM_WMTDRV_TYPE_T eType
    );

static void *
glResetCallback (
    ENUM_WMTDRV_TYPE_T  eSrcType,
    ENUM_WMTDRV_TYPE_T  eDstType,
    ENUM_WMTMSG_TYPE_T  eMsgType,
    void *              prMsgBody,
    unsigned int        u4MsgLength
    );

static BOOLEAN
glResetSendMessage (
    char    *aucMsg,
    u8      cmd
    );


/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
VOID
glResetInit(
    VOID
    )
{
    /* 1. register for reset callback */
    mtk_wcn_wmt_msgcb_reg(WMTDRV_TYPE_WIFI, (PF_WMT_CB)glResetCallback);

    /* 2.1 registration for NETLINK_GENERIC family */
    if(genl_register_family(&mtk_wifi_gnl_family) != 0) {
        DBGLOG(INIT, WARN, ("%s(): GE_NELINK family registration fail\n", __func__));
    }
    else {
        /* 2.2 operation registration */
        if(genl_register_ops(&mtk_wifi_gnl_family, &mtk_wifi_gnl_ops_bind) != 0) {
            DBGLOG(INIT, WARN, ("%s(): BIND operation registration fail\n", __func__));
        }

        if(genl_register_ops(&mtk_wifi_gnl_family, &mtk_wifi_gnl_ops_reset) != 0) {
            DBGLOG(INIT, WARN, ("%s(): RESET operation registration fail\n", __func__));
        }
    }

    return;
}


/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
VOID
glResetUninit(
    VOID
    )
{
    /* 1. release NETLINK_GENERIC family */
    genl_unregister_family(&mtk_wifi_gnl_family);

    /* 2. deregister for reset callback */
    mtk_wcn_wmt_msgcb_unreg(WMTDRV_TYPE_WIFI);

    return;
}


/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
static void *
glResetCallback (
    ENUM_WMTDRV_TYPE_T  eSrcType,
    ENUM_WMTDRV_TYPE_T  eDstType,
    ENUM_WMTMSG_TYPE_T  eMsgType,
    void *              prMsgBody,
    unsigned int        u4MsgLength
    )
{
    switch(eMsgType) {
    case WMTMSG_TYPE_RESET:
        if(u4MsgLength == sizeof(ENUM_WMTRSTMSG_TYPE_T)) {
            P_ENUM_WMTRSTMSG_TYPE_T prRstMsg = (P_ENUM_WMTRSTMSG_TYPE_T) prMsgBody;

            switch(*prRstMsg) {
            case WMTRSTMSG_RESET_START:
                fgIsResetting = TRUE;
                glResetSendMessage(MTK_WIFI_RESET_START_NAME, MTK_WIFI_COMMAND_RESET);
                break;

            case WMTRSTMSG_RESET_END:
                glResetSendMessage(MTK_WIFI_RESET_END_NAME, MTK_WIFI_COMMAND_RESET);
                fgIsResetting = FALSE;
                break;

            default:
                break;
            }
        }

        break;

    default:
        break;
    }

    return NULL;
}


/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
static BOOLEAN
glResetSendMessage(
    char *  aucMsg,
    u8      cmd
    )
{
    struct sk_buff *skb = NULL;
    void *msg_head = NULL;
    int rc = -1;
    int i;

    if(num_bind_process == 0) {
        /* no listening process */
        return FALSE;
    }

    for(i = 0 ; i < num_bind_process ; i++) {
        skb = genlmsg_new(NLMSG_GOODSIZE, GFP_KERNEL);

        if(skb) {
            msg_head = genlmsg_put(skb, 0, mtk_wifi_seqnum++, &mtk_wifi_gnl_family, 0, cmd);

            if(msg_head == NULL) {
                nlmsg_free(skb);
                return FALSE;
            }

            rc = nla_put_string(skb, MTK_WIFI_ATTR_MSG, aucMsg);
            if(rc != 0) {
                nlmsg_free(skb);
                return FALSE;
            }
        
            /* finalize the message */
            genlmsg_end(skb, msg_head);
        
            /* sending message */
            rc = genlmsg_unicast(&init_net, skb, bind_pid[i]);
            if(rc != 0) {
                return FALSE;
            }
        }
        else {
            return FALSE;
        }
    }

    return TRUE;
}


/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
int mtk_wifi_bind(
    struct sk_buff *skb,
    struct genl_info *info
    )
{
    struct nlattr *na;
    char * mydata;

    if (info == NULL) {
        goto out;
    }

    /*for each attribute there is an index in info->attrs which points to a nlattr structure
     *in this structure the data is given
     */
    
    na = info->attrs[MTK_WIFI_ATTR_MSG];
    if (na) {
        mydata = (char *)nla_data(na);

        /* no need to parse mydata */
    }

    /* collect PID */
    if(num_bind_process < MAX_BIND_PROCESS) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 7, 0)
        bind_pid[num_bind_process] = info->snd_portid;
#else
        bind_pid[num_bind_process] = info->snd_pid;
#endif
        num_bind_process++;
        }
    else {
        DBGLOG(INIT, WARN, ("%s(): exceeding binding limit %d\n", __func__, MAX_BIND_PROCESS));
    }

out:
    return 0;
}


/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
int mtk_wifi_reset(
    struct sk_buff *skb,
    struct genl_info *info
    )
{
    DBGLOG(INIT, WARN, ("%s(): should not be invoked\n", __func__));

    return 0;
}


/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
VOID
glSendResetRequest(
    VOID
    )
{
    // WMT thread would trigger whole chip resetting itself
    return;
}


/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
BOOLEAN
kalIsResetting(
    VOID
    )
{
    return fgIsResetting;
}


#endif // CFG_CHIP_RESET_SUPPORT

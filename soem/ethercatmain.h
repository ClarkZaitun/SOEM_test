/*
 * Licensed under the GNU General Public License version 2 with exceptions. See
 * LICENSE file in the project root for full license information
 */

/** \file
 * \brief
 * Headerfile for ethercatmain.c
 */

#ifndef _ethercatmain_
#define _ethercatmain_


#ifdef __cplusplus
extern "C"
{
#endif

/** max. entries in EtherCAT error list */
#define EC_MAXELIST       64
/** max. length of readable name in slavelist and Object Description List */
#define EC_MAXNAME        40
/** max. number of slaves in array */
#define EC_MAXSLAVE       200
/** max. number of groups */
#define EC_MAXGROUP       2
/** max. number of IO segments per group */
#define EC_MAXIOSEGMENTS  64
/** max. mailbox size */
#define EC_MAXMBX         1486
/** max. eeprom PDO entries */
#define EC_MAXEEPDO       0x200
/** max. SM used */
#define EC_MAXSM          8
/** max. FMMU used */
#define EC_MAXFMMU        4
/** max. Adapter */
#define EC_MAXLEN_ADAPTERNAME    128
/** define maximum number of concurrent threads in mapping */
#define EC_MAX_MAPT           1

typedef struct ec_adapter ec_adaptert;
struct ec_adapter
{
   char   name[EC_MAXLEN_ADAPTERNAME];
   char   desc[EC_MAXLEN_ADAPTERNAME];
   ec_adaptert *next;
};

/** record for FMMU */
PACKED_BEGIN
typedef struct PACKED ec_fmmu
{
   uint32  LogStart;
   uint16  LogLength;
   uint8   LogStartbit;
   uint8   LogEndbit;
   uint16  PhysStart;
   uint8   PhysStartBit;
   uint8   FMMUtype;
   uint8   FMMUactive;
   uint8   unused1;
   uint16  unused2;
}  ec_fmmut;
PACKED_END

/** record for sync manager */
PACKED_BEGIN
typedef struct PACKED ec_sm
{
   uint16  StartAddr;
   uint16  SMlength;
   uint32  SMflags;
} ec_smt;
PACKED_END

PACKED_BEGIN
typedef struct PACKED ec_state_status
{
   uint16  State;
   uint16  Unused;
   uint16  ALstatuscode;
} ec_state_status;
PACKED_END

#define ECT_MBXPROT_AOE      0x0001
#define ECT_MBXPROT_EOE      0x0002
#define ECT_MBXPROT_COE      0x0004
#define ECT_MBXPROT_FOE      0x0008
#define ECT_MBXPROT_SOE      0x0010
#define ECT_MBXPROT_VOE      0x0020

#define ECT_COEDET_SDO       0x01
#define ECT_COEDET_SDOINFO   0x02
#define ECT_COEDET_PDOASSIGN 0x04
#define ECT_COEDET_PDOCONFIG 0x08
#define ECT_COEDET_UPLOAD    0x10
#define ECT_COEDET_SDOCA     0x20

#define EC_SMENABLEMASK      0xfffeffff

typedef struct ecx_context ecx_contextt;

/** EtherCAT从站信息结构体，用于存储检测到的从站信息 */
typedef struct ec_slave
{
   /** 从站状态 */
   uint16           state;
   /** AL状态码 */
   uint16           ALstatuscode;
   /** 配置地址 */
   uint16           configadr;
   /** 别名地址 */
   uint16           aliasadr;
   /** EEPROM中的制造商ID */
   uint32           eep_man;
   /** EEPROM中的产品ID */
   uint32           eep_id;
   /** EEPROM中的修订号 */
   uint32           eep_rev;
   /** 接口类型 */
   uint16           Itype;
   /** 设备类型，从配置列表中读取的信息，EEPROM中没有 */
   uint16           Dtype;
   /** 输出位数 */
   uint16           Obits;
   /** 输出字节数，如果Obits < 8则Obytes = 0 */
   uint32           Obytes;
   /** IOmap缓冲区中的输出指针 */
   uint8            *outputs;
   /** 第一个输出字节中的起始位 */
   uint8            Ostartbit;
   /** 输入位数 */
   uint16           Ibits;
   /** 输入字节数，如果Ibits < 8则Ibytes = 0 */
   uint32           Ibytes;
   /** IOmap缓冲区中的输入指针 */
   uint8            *inputs;
   /** 第一个输入字节中的起始位 */
   uint8            Istartbit;
   /** 同步管理器(SM)结构 */
   ec_smt           SM[EC_MAXSM];
   /** SM类型：0=未使用 1=邮箱写 2=邮箱读 3=输出 4=输入 */
   uint8            SMtype[EC_MAXSM];
   /** FMMU结构 */
   ec_fmmut         FMMU[EC_MAXFMMU];
   /** FMMU0功能 */
   uint8            FMMU0func;
   /** FMMU1功能 */
   uint8            FMMU1func;
   /** FMMU2功能 */
   uint8            FMMU2func;
   /** FMMU3功能 */
   uint8            FMMU3func;
   /** 写邮箱长度（字节），无邮箱则为0 */
   uint16           mbx_l;
   /** 邮箱写偏移量 */
   uint16           mbx_wo;
   /** 读邮箱长度（字节） */
   uint16           mbx_rl;
   /** 邮箱读偏移量 */
   uint16           mbx_ro;
   /** 邮箱支持的协议 */
   uint16           mbx_proto;
   /** 邮箱链路层协议计数器值 1..7 */
   uint8            mbx_cnt;
   /** 是否具有DC（分布式时钟）能力 */
   boolean          hasdc;
   /** 物理类型：Ebus、EtherNet组合 */
   uint8            ptype;
   /** 拓扑结构：1到3个链路 */
   uint8 topology;
   /** 活跃端口位图：....3210，相应端口活跃时置位 */
   uint8 activeports;
   /** 已消费端口位图：....3210，用于内部延迟测量 */
   uint8            consumedports;
   /** 父节点从站号，0=主站 */
   uint16           parent;
   /** 父节点上连接此从站的端口号 */
   uint8            parentport;
   /** 此从站上连接父节点的端口号 */
   uint8            entryport;
   /** 端口A的DC接收时间 */
   int32            DCrtA;
   /** 端口B的DC接收时间 */
   int32            DCrtB;
   /** 端口C的DC接收时间 */
   int32            DCrtC;
   /** 端口D的DC接收时间 */
   int32            DCrtD;
   /** 传播延迟 */
   int32            pdelay;
   /** 下一个DC从站的数组序号 */
   uint16           DCnext;
   /** 上一个DC从站的数组序号 */
   uint16           DCprevious;
   /** DC周期时间（纳秒） */
   int32            DCcycle;
   /** 距时钟模边界的DC偏移 */
   int32            DCshift;
   /** DC同步激活：0=关闭，1=开启 */
   uint8            DCactive;
   /** 链接到配置表 */
   uint16           configindex;
   /** 链接到SII配置 */
   uint16           SIIindex;
   /** 1=每次读取8字节，0=每次读取4字节 */
   uint8            eep_8byte;
   /** 0=EEPROM归主站，1=EEPROM归PDI */
   uint8            eep_pdi;
   /** CoE详细信息 */
   uint8            CoEdetails;
   /** FoE详细信息 */
   uint8            FoEdetails;
   /** EoE详细信息 */
   uint8            EoEdetails;
   /** SoE详细信息 */
   uint8            SoEdetails;
   /** E-bus电流 */
   int16            Ebuscurrent;
   /** 如果>0，在过程数据中阻止使用LRW */
   uint8            blockLRW;
   /** 从站所在组序号 */
   uint8            group;
   /** 第一个未使用的FMMU */
   uint8            FMMUunused;
   /** 用于跟踪从站是否（不）响应的布尔值，SOEM库未使用/设置 */
   boolean          islost;
   /** 注册的配置函数 PO->SO（已弃用）*/
   int              (*PO2SOconfig)(uint16 slave);
   /** 注册的配置函数 PO->SO */
   int              (*PO2SOconfigx)(ecx_contextt * context, uint16 slave);
   /** 可读名称 */
   char             name[EC_MAXNAME + 1];
} ec_slavet;

/** EtherCAT从站组列表结构体 */
typedef struct ec_group
{
   /** 该组的逻辑起始地址 */
   uint32           logstartaddr;
   /** 输出字节数，如果Obits < 8则Obytes = 0 */
   uint32           Obytes;
   /** IOmap缓冲区中的输出指针 */
   uint8            *outputs;
   /** 输入字节数，如果Ibits < 8则Ibytes = 0 */
   uint32           Ibytes;
   /** IOmap缓冲区中的输入指针 */
   uint8            *inputs;
   /** 是否具有DC（分布式时钟）能力 */
   // 用于决定是否需要发送FRMW 0x910命令
   boolean          hasdc;
   /** 下一个DC从站 */
   // 发送FRMW 0x910命令的从站地址
   uint16           DCnext;
   /** E-bus电流 */
   // 目前只用于打印
   int16            Ebuscurrent;
   /** 如果>0，则在过程数据中阻止使用LRW */
   uint8            blockLRW;
   /** 使用的IO段数量 */
   uint16           nsegments;
   /** 第一个输入段 */
   uint16           Isegment;
   /** 输入段中的偏移量 */
   uint16           Ioffset;
   /** 预期的输出工作计数器 */
   uint16           outputsWKC;
   /** 预期的输入工作计数器 */
   uint16           inputsWKC;
   // docheckstate作用：标记是否需要持续检查从站状态，
   // 当发现有从站状态异常时会被设置为true，确保在下一次循环中
   // 继续检查状态，直到所有从站恢复正常
   boolean          docheckstate;
   /** IO分段列表。数据报不能将SM分成两部分。 */
   uint32           IOsegment[EC_MAXIOSEGMENTS];
} ec_groupt;

/** SII FMMU structure */
typedef struct ec_eepromFMMU
{
   uint16  Startpos;
   uint8   nFMMU;
   uint8   FMMU0;
   uint8   FMMU1;
   uint8   FMMU2;
   uint8   FMMU3;
} ec_eepromFMMUt;

/** SII SM structure */
typedef struct ec_eepromSM
{
   uint16  Startpos;
   uint8   nSM;
   uint16  PhStart;
   uint16  Plength;
   uint8   Creg;
   uint8   Sreg;       /* don't care */
   uint8   Activate;
   uint8   PDIctrl;      /* don't care */
} ec_eepromSMt;

/** record to store rxPDO and txPDO table from eeprom */
typedef struct ec_eepromPDO
{
   uint16  Startpos;
   uint16  Length;
   uint16  nPDO;
   uint16  Index[EC_MAXEEPDO];
   uint16  SyncM[EC_MAXEEPDO];
   uint16  BitSize[EC_MAXEEPDO];
   uint16  SMbitsize[EC_MAXSM];
} ec_eepromPDOt;

/** mailbox buffer array */
typedef uint8 ec_mbxbuft[EC_MAXMBX + 1];

/** standard ethercat mailbox header */
PACKED_BEGIN
typedef struct PACKED ec_mbxheader
{
   uint16  length;
   uint16  address;
   uint8   priority;
   uint8   mbxtype;
} ec_mbxheadert;
PACKED_END

/** ALstatus and ALstatus code */
PACKED_BEGIN
typedef struct PACKED ec_alstatus
{
   uint16  alstatus;
   uint16  unused;
   uint16  alstatuscode;
} ec_alstatust;
PACKED_END

/** stack structure to store segmented LRD/LWR/LRW constructs */
typedef struct ec_idxstack
{
   uint8   pushed;
   uint8   pulled;
   uint8   idx[EC_MAXBUF];
   void    *data[EC_MAXBUF];
   uint16  length[EC_MAXBUF];
   uint16  dcoffset[EC_MAXBUF];
} ec_idxstackT;

/** ringbuf for error storage */
typedef struct ec_ering
{
   int16     head;
   int16     tail;
   ec_errort Error[EC_MAXELIST + 1];
} ec_eringt;

/** SyncManager Communication Type structure for CA */
PACKED_BEGIN
typedef struct PACKED ec_SMcommtype
{
   uint8   n;
   uint8   nu1;
   uint8   SMtype[EC_MAXSM];
} ec_SMcommtypet;
PACKED_END

/** SDO assign structure for CA */
PACKED_BEGIN
typedef struct PACKED ec_PDOassign
{
   uint8   n;
   uint8   nu1;
   uint16  index[256];
} ec_PDOassignt;
PACKED_END

/** SDO description structure for CA */
PACKED_BEGIN
typedef struct PACKED ec_PDOdesc
{
   uint8   n;
   uint8   nu1;
   uint32  PDO[256];
} ec_PDOdesct;
PACKED_END

/** EtherCAT主站上下文结构体，所有ecx函数都引用此结构 */
struct ecx_context
{
   /** 端口引用，可能包含冗余端口 */
   ecx_portt      *port;
   /** 从站列表引用 */
   ec_slavet      *slavelist;
   /** 配置中发现的从站数量 */
   int            *slavecount;
   /** 从站列表允许的最大从站数 */
   int            maxslave;
   /** 组列表引用 */
   ec_groupt      *grouplist;
   /** 组列表允许的最大组数 */
   int            maxgroup;
   /** 内部：EEPROM缓存缓冲区引用 */
   uint8          *esibuf;
   /** 内部：EEPROM缓存映射引用 */
   uint32         *esimap;
   /** 内部：当前EEPROM缓存的从站 */
   uint16         esislave;
   /** 内部：错误列表引用 */
   ec_eringt      *elist;
   /** 内部：过程数据栈缓冲区信息引用 */
   ec_idxstackT   *idxstack;
   /** EtherCAT错误状态引用 */
   boolean        *ecaterror;
   /** 从站最后一次DC时间引用（纳秒） */
   int64          *DCtime;
   /** 内部：SM缓冲区 */
   ec_SMcommtypet *SMcommtype;
   /** 内部：PDO分配列表 */
   ec_PDOassignt  *PDOassign;
   /** 内部：PDO描述列表 */
   ec_PDOdesct    *PDOdesc;
   /** 内部：来自EEPROM的SM列表 */
   ec_eepromSMt   *eepSM;
   /** 内部：来自EEPROM的FMMU列表 */
   ec_eepromFMMUt *eepFMMU;
   /** 注册的FoE钩子函数 */
   int            (*FOEhook)(uint16 slave, int packetnumber, int datasize);
   /** 注册的EoE钩子函数 */
   int            (*EOEhook)(ecx_contextt * context, uint16 slave, void * eoembx);
   /** 控制传统自动状态变更或手动状态变更的标志 */
   int            manualstatechange;
   /** 用户数据，用于应用程序配置，特别是在EC_VER2中支持多个ec_context实例。
    * 注意：用户数据内存由应用程序管理，而非SOEM */
   void           *userdata;
};

#ifdef EC_VER1
/** global struct to hold default master context */
extern ecx_contextt  ecx_context;
/** main slave data structure array */
extern ec_slavet   ec_slave[EC_MAXSLAVE];
/** number of slaves found by configuration function */
extern int         ec_slavecount;
/** slave group structure */
extern ec_groupt   ec_group[EC_MAXGROUP];
extern boolean     EcatError;
extern int64       ec_DCtime;

void ec_pusherror(const ec_errort *Ec);
boolean ec_poperror(ec_errort *Ec);
boolean ec_iserror(void);
void ec_packeterror(uint16 Slave, uint16 Index, uint8 SubIdx, uint16 ErrorCode);
int ec_init(const char * ifname);
int ec_init_redundant(const char *ifname, char *if2name);
void ec_close(void);
uint8 ec_siigetbyte(uint16 slave, uint16 address);
int16 ec_siifind(uint16 slave, uint16 cat);
void ec_siistring(char *str, uint16 slave, uint16 Sn);
uint16 ec_siiFMMU(uint16 slave, ec_eepromFMMUt* FMMU);
uint16 ec_siiSM(uint16 slave, ec_eepromSMt* SM);
uint16 ec_siiSMnext(uint16 slave, ec_eepromSMt* SM, uint16 n);
uint32 ec_siiPDO(uint16 slave, ec_eepromPDOt* PDO, uint8 t);
int ec_readstate(void);
int ec_writestate(uint16 slave);
uint16 ec_statecheck(uint16 slave, uint16 reqstate, int timeout);
int ec_mbxempty(uint16 slave, int timeout);
int ec_mbxsend(uint16 slave,ec_mbxbuft *mbx, int timeout);
int ec_mbxreceive(uint16 slave, ec_mbxbuft *mbx, int timeout);
void ec_esidump(uint16 slave, uint8 *esibuf);
uint32 ec_readeeprom(uint16 slave, uint16 eeproma, int timeout);
int ec_writeeeprom(uint16 slave, uint16 eeproma, uint16 data, int timeout);
int ec_eeprom2master(uint16 slave);
int ec_eeprom2pdi(uint16 slave);
uint64 ec_readeepromAP(uint16 aiadr, uint16 eeproma, int timeout);
int ec_writeeepromAP(uint16 aiadr, uint16 eeproma, uint16 data, int timeout);
uint64 ec_readeepromFP(uint16 configadr, uint16 eeproma, int timeout);
int ec_writeeepromFP(uint16 configadr, uint16 eeproma, uint16 data, int timeout);
void ec_readeeprom1(uint16 slave, uint16 eeproma);
uint32 ec_readeeprom2(uint16 slave, int timeout);
int ec_send_processdata_group(uint8 group);
int ec_send_overlap_processdata_group(uint8 group);
int ec_receive_processdata_group(uint8 group, int timeout);
int ec_send_processdata(void);
int ec_send_overlap_processdata(void);
int ec_receive_processdata(int timeout);
#endif

ec_adaptert * ec_find_adapters(void);
void ec_free_adapters(ec_adaptert * adapter);
uint8 ec_nextmbxcnt(uint8 cnt);
void ec_clearmbx(ec_mbxbuft *Mbx);
void ecx_pusherror(ecx_contextt *context, const ec_errort *Ec);
boolean ecx_poperror(ecx_contextt *context, ec_errort *Ec);
boolean ecx_iserror(ecx_contextt *context);
void ecx_packeterror(ecx_contextt *context, uint16 Slave, uint16 Index, uint8 SubIdx, uint16 ErrorCode);
int ecx_init(ecx_contextt *context, const char * ifname);
int ecx_init_redundant(ecx_contextt *context, ecx_redportt *redport, const char *ifname, char *if2name);
void ecx_close(ecx_contextt *context);
uint8 ecx_siigetbyte(ecx_contextt *context, uint16 slave, uint16 address);
int16 ecx_siifind(ecx_contextt *context, uint16 slave, uint16 cat);
void ecx_siistring(ecx_contextt *context, char *str, uint16 slave, uint16 Sn);
uint16 ecx_siiFMMU(ecx_contextt *context, uint16 slave, ec_eepromFMMUt* FMMU);
uint16 ecx_siiSM(ecx_contextt *context, uint16 slave, ec_eepromSMt* SM);
uint16 ecx_siiSMnext(ecx_contextt *context, uint16 slave, ec_eepromSMt* SM, uint16 n);
uint32 ecx_siiPDO(ecx_contextt *context, uint16 slave, ec_eepromPDOt* PDO, uint8 t);
int ecx_readstate(ecx_contextt *context);
int ecx_writestate(ecx_contextt *context, uint16 slave);
uint16 ecx_statecheck(ecx_contextt *context, uint16 slave, uint16 reqstate, int timeout);
int ecx_mbxempty(ecx_contextt *context, uint16 slave, int timeout);
int ecx_mbxsend(ecx_contextt *context, uint16 slave,ec_mbxbuft *mbx, int timeout);
int ecx_mbxreceive(ecx_contextt *context, uint16 slave, ec_mbxbuft *mbx, int timeout);
void ecx_esidump(ecx_contextt *context, uint16 slave, uint8 *esibuf);
uint32 ecx_readeeprom(ecx_contextt *context, uint16 slave, uint16 eeproma, int timeout);
int ecx_writeeeprom(ecx_contextt *context, uint16 slave, uint16 eeproma, uint16 data, int timeout);
int ecx_eeprom2master(ecx_contextt *context, uint16 slave);
int ecx_eeprom2pdi(ecx_contextt *context, uint16 slave);
uint64 ecx_readeepromAP(ecx_contextt *context, uint16 aiadr, uint16 eeproma, int timeout);
int ecx_writeeepromAP(ecx_contextt *context, uint16 aiadr, uint16 eeproma, uint16 data, int timeout);
uint64 ecx_readeepromFP(ecx_contextt *context, uint16 configadr, uint16 eeproma, int timeout);
int ecx_writeeepromFP(ecx_contextt *context, uint16 configadr, uint16 eeproma, uint16 data, int timeout);
void ecx_readeeprom1(ecx_contextt *context, uint16 slave, uint16 eeproma);
uint32 ecx_readeeprom2(ecx_contextt *context, uint16 slave, int timeout);
int ecx_send_overlap_processdata_group(ecx_contextt *context, uint8 group);
int ecx_receive_processdata_group(ecx_contextt *context, uint8 group, int timeout);
int ecx_send_processdata(ecx_contextt *context);
int ecx_send_overlap_processdata(ecx_contextt *context);
int ecx_receive_processdata(ecx_contextt *context, int timeout);
int ecx_send_processdata_group(ecx_contextt *context, uint8 group);

#ifdef __cplusplus
}
#endif

#endif

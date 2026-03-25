/*
 * Licensed under the GNU General Public License version 2 with exceptions. See
 * LICENSE file in the project root for full license information
 */

/** \file
 * \brief
 * Configuration module for EtherCAT master.
 *
 * After successful initialisation with ec_init() or ec_init_redundant()
 * the slaves can be auto configured with this module.
 */

#include <stdio.h>
#include <string.h>
#include "osal.h"
#include "oshw.h"
#include "ethercattype.h"
#include "ethercatbase.h"
#include "ethercatmain.h"
#include "ethercatcoe.h"
#include "ethercatsoe.h"
#include "ethercatconfig.h"


typedef struct
{
   int thread_n;
   int running;
   ecx_contextt *context;
   uint16 slave;
} ecx_mapt_t;

ecx_mapt_t ecx_mapt[EC_MAX_MAPT];
#if EC_MAX_MAPT > 1
OSAL_THREAD_HANDLE ecx_threadh[EC_MAX_MAPT];
#endif

#ifdef EC_VER1
/** Slave configuration structure */
typedef const struct
{
   /** Manufacturer code of slave */
   uint32           man;
   /** ID of slave */
   uint32           id;
   /** Readable name */
   char             name[EC_MAXNAME + 1];
   /** Data type */
   uint8            Dtype;
   /** Input bits */
   uint16            Ibits;
   /** Output bits */
   uint16           Obits;
   /** SyncManager 2 address */
   uint16           SM2a;
   /** SyncManager 2 flags */
   uint32           SM2f;
   /** SyncManager 3 address */
   uint16           SM3a;
   /** SyncManager 3 flags */
   uint32           SM3f;
   /** FMMU 0 activation */
   uint8            FM0ac;
   /** FMMU 1 activation */
   uint8            FM1ac;
} ec_configlist_t;

#include "ethercatconfiglist.h"
#endif

/** standard SM0 flags configuration for mailbox slaves */
#define EC_DEFAULTMBXSM0  0x00010026
/** standard SM1 flags configuration for mailbox slaves */
#define EC_DEFAULTMBXSM1  0x00010022
/** standard SM0 flags configuration for digital output slaves */
#define EC_DEFAULTDOSM0   0x00010044

#ifdef EC_VER1
/** Find slave in standard configuration list ec_configlist[]
 *
 * @param[in] man      = manufacturer
 * @param[in] id       = ID
 * @return index in ec_configlist[] when found, otherwise 0
 */
int ec_findconfig( uint32 man, uint32 id)
{
   int i = 0;

   do
   {
      i++;
   } while ( (ec_configlist[i].man != EC_CONFIGEND) &&
           ((ec_configlist[i].man != man) || (ec_configlist[i].id != id)) );
   if (ec_configlist[i].man == EC_CONFIGEND)
   {
      i = 0;
   }
   return i;
}
#endif

/** 初始化EtherCAT主站上下文。
 * 清除从站列表和组列表，并为每个组设置默认的逻辑起始地址。
 *
 * @param[in] context = 要初始化的上下文结构体
 */
void ecx_init_context(ecx_contextt *context)
{
   int lp;  // 循环变量

   // 初始化从站计数为0
   *(context->slavecount) = 0;

   /* 清除从站列表数组 */
   // 将所有从站结构体清零，确保初始状态干净
   memset(context->slavelist, 0x00, sizeof(ec_slavet) * context->maxslave);

   // 清除组列表数组
   // 将所有组结构体清零，确保初始状态干净
   memset(context->grouplist, 0x00, sizeof(ec_groupt) * context->maxgroup);

   /* 清除从站EEPROM缓存，实际上并不读取任何EEPROM */
   // 通过访问超出范围的地址来清除缓存，初始化ESI缓存系统
   ecx_siigetbyte(context, 0, EC_MAXEEPBUF);

   // 为每个组设置默认的逻辑起始地址
   // 使用组索引左移EC_LOGGROUPOFFSET位来计算逻辑起始地址
   // 这样每个组的逻辑地址空间不会重叠
   for(lp = 0; lp < context->maxgroup; lp++)
   {
      /* 每个组条目的默认起始地址 */
      context->grouplist[lp].logstartaddr = lp << EC_LOGGROUPOFFSET;
   }
}

/**
 * 检测总线上的从站数量
 *
 * 该函数通过广播读取类型寄存器来检测总线上的从站数量。
 * 主要功能包括：
 * 1. 对旧版netX100从站进行特殊预初始化
 * 2. 忽略别名寄存器
 * 3. 将所有从站重置为INIT状态
 * 4. 检测从站数量
 *
 * @param[in] context EtherCAT上下文结构体
 * @return 成功时返回从站数量，失败时返回错误码
 */
int ecx_detect_slaves(ecx_contextt *context)
{
   uint8  b;      // 8位临时变量
   uint16 w;      // 16位临时变量
   int    wkc;    // 工作计数器

   /* 对旧版netX100从站进行特殊的预初始化寄存器写入 */
   /* 以启用MAC[1]本地管理位设置 */
   b = 0x00;
   // 忽略别名寄存器
   ecx_BWR(context->port, 0x0000, ECT_REG_DLALIAS, sizeof(b), &b, EC_TIMEOUTRET3);

   w = htoes(EC_STATE_INIT | EC_STATE_ACK);
   // 将所有从站重置为INIT状态
   ecx_BWR(context->port, 0x0000, ECT_REG_ALCTL, sizeof(w), &w, EC_TIMEOUTRET3);
   /* netX100现在应该可以正常工作了 */
   // 再次重置所有从站为INIT状态
   ecx_BWR(context->port, 0x0000, ECT_REG_ALCTL, sizeof(w), &w, EC_TIMEOUTRET3);

   // 检测从站数量（广播读取类型寄存器）
   wkc = ecx_BRD(context->port, 0x0000, ECT_REG_TYPE, sizeof(w), &w, EC_TIMEOUTSAFE);

   if (wkc > 0)
   {
      /* 严格使用"小于"，因为主站是"从站0" */
      if (wkc < context->maxslave)
      {
         *(context->slavecount) = wkc;
      }
      else
      {
         EC_PRINT("Error: too many slaves on network: num_slaves=%d, max_slaves=%d\n",
               wkc, context->maxslave);
         return EC_SLAVECOUNTEXCEEDED;
      }
   }
   return wkc;
}

/**
 * 将所有从站设置为默认状态
 *
 * 该函数通过广播写入将所有从站的寄存器重置为默认值。
 * 主要功能包括：
 * 1. 禁用环路手动控制
 * 2. 设置IRQ掩码
 * 3. 重置CRC计数器
 * 4. 重置FMMU和Sync Manager
 * 5. 重置DC相关寄存器
 * 6. 将EEPROM控制权交给主站
 *
 * @param[in] context EtherCAT上下文结构体
 */
static void ecx_set_slaves_to_default(ecx_contextt *context)
{
   uint8 b;           // 8位临时变量
   uint16 w;          // 16位临时变量
   uint8 zbuf[64];    // 零缓冲区

   // 初始化零缓冲区
   memset(&zbuf, 0x00, sizeof(zbuf));

   b = 0x00;
   // 禁用环路手动控制
   ecx_BWR(context->port, 0x0000, ECT_REG_DLPORT      , sizeof(b) , &b, EC_TIMEOUTRET3);

   w = htoes(0x0004);
   // 设置IRQ掩码
   ecx_BWR(context->port, 0x0000, ECT_REG_IRQMASK     , sizeof(w) , &w, EC_TIMEOUTRET3);

   // 重置CRC计数器
   ecx_BWR(context->port, 0x0000, ECT_REG_RXERR       , 8         , &zbuf, EC_TIMEOUTRET3);

   // 重置FMMU（3个FMMU，每个16字节）
   ecx_BWR(context->port, 0x0000, ECT_REG_FMMU0       , 16 * 3    , &zbuf, EC_TIMEOUTRET3);

   // 重置Sync Manager（4个SM，每个8字节）
   ecx_BWR(context->port, 0x0000, ECT_REG_SM0         , 8 * 4     , &zbuf, EC_TIMEOUTRET3);

   b = 0x00;
   // 重置DC激活寄存器
   ecx_BWR(context->port, 0x0000, ECT_REG_DCSYNCACT   , sizeof(b) , &b, EC_TIMEOUTRET3);

   // 重置系统时间和偏移
   ecx_BWR(context->port, 0x0000, ECT_REG_DCSYSTIME   , 4         , &zbuf, EC_TIMEOUTRET3);

   w = htoes(0x1000);
   // DC速度启动
   ecx_BWR(context->port, 0x0000, ECT_REG_DCSPEEDCNT  , sizeof(w) , &w, EC_TIMEOUTRET3);

   w = htoes(0x0c00);
   // DC滤波器表达式
   ecx_BWR(context->port, 0x0000, ECT_REG_DCTIMEFILT  , sizeof(w) , &w, EC_TIMEOUTRET3);

   b = 0x00;
   // 忽略别名寄存器
   ecx_BWR(context->port, 0x0000, ECT_REG_DLALIAS     , sizeof(b) , &b, EC_TIMEOUTRET3);

   w = htoes(EC_STATE_INIT | EC_STATE_ACK);
   // 将所有从站重置为INIT状态
   ecx_BWR(context->port, 0x0000, ECT_REG_ALCTL       , sizeof(w) , &w, EC_TIMEOUTRET3);

   b = 2;
   // 强制EEPROM从PDI读取
   ecx_BWR(context->port, 0x0000, ECT_REG_EEPCFG      , sizeof(b) , &b, EC_TIMEOUTRET3);

   b = 0;
   // 将EEPROM控制权交给主站
   ecx_BWR(context->port, 0x0000, ECT_REG_EEPCFG      , sizeof(b) , &b, EC_TIMEOUTRET3);
}

#ifdef EC_VER1
static int ecx_config_from_table(ecx_contextt *context, uint16 slave)
{
   int cindex;
   ec_slavet *csl;

   csl = &(context->slavelist[slave]);
   cindex = ec_findconfig( csl->eep_man, csl->eep_id );
   csl->configindex= cindex;
   /* slave found in configuration table ? */
   if (cindex)
   {
      csl->Dtype = ec_configlist[cindex].Dtype;
      strcpy(csl->name ,ec_configlist[cindex].name);
      csl->Ibits = ec_configlist[cindex].Ibits;
      csl->Obits = ec_configlist[cindex].Obits;
      if (csl->Obits)
      {
         csl->FMMU0func = 1;
      }
      if (csl->Ibits)
      {
         csl->FMMU1func = 2;
      }
      csl->FMMU[0].FMMUactive = ec_configlist[cindex].FM0ac;
      csl->FMMU[1].FMMUactive = ec_configlist[cindex].FM1ac;
      csl->SM[2].StartAddr = htoes(ec_configlist[cindex].SM2a);
      csl->SM[2].SMflags = htoel(ec_configlist[cindex].SM2f);
      /* simple (no mailbox) output slave found ? */
      if (csl->Obits && !csl->SM[2].StartAddr)
      {
         csl->SM[0].StartAddr = htoes(0x0f00);
         csl->SM[0].SMlength = htoes((csl->Obits + 7) / 8);
         csl->SM[0].SMflags = htoel(EC_DEFAULTDOSM0);
         csl->FMMU[0].FMMUactive = 1;
         csl->FMMU[0].FMMUtype = 2;
         csl->SMtype[0] = 3;
      }
      /* complex output slave */
      else
      {
         csl->SM[2].SMlength = htoes((csl->Obits + 7) / 8);
         csl->SMtype[2] = 3;
      }
      csl->SM[3].StartAddr = htoes(ec_configlist[cindex].SM3a);
      csl->SM[3].SMflags = htoel(ec_configlist[cindex].SM3f);
      /* simple (no mailbox) input slave found ? */
      if (csl->Ibits && !csl->SM[3].StartAddr)
      {
         csl->SM[1].StartAddr = htoes(0x1000);
         csl->SM[1].SMlength = htoes((csl->Ibits + 7) / 8);
         csl->SM[1].SMflags = htoel(0x00000000);
         csl->FMMU[1].FMMUactive = 1;
         csl->FMMU[1].FMMUtype = 1;
         csl->SMtype[1] = 4;
      }
      /* complex input slave */
      else
      {
         csl->SM[3].SMlength = htoes((csl->Ibits + 7) / 8);
         csl->SMtype[3] = 4;
      }
   }
   return cindex;
}
#else
static int ecx_config_from_table(ecx_contextt *context, uint16 slave)
{
   (void)context;
   (void)slave;
   return 0;
}
#endif

/**
 * 查找之前相同从站的SII信息
 *
 * 如果从站有SII（从站信息接口）且之前已经处理过相同ID的从站，
 * 则复用之前的SII数据。这是安全的，因为相同ID从站的SII信息是相同的。
 *
 * 该优化可以显著减少配置时间，避免重复读取EEPROM。
 *
 * 复用的信息包括：
 * - CoE/FoE/EoE/SoE详情
 * - blockLRW标志
 * - E-bus电流
 * - 从站名称
 * - SM配置
 * - FMMU功能
 *
 * @param[in] context EtherCAT上下文结构体
 * @param[in] slave   当前从站编号
 * @return 1表示找到并复制了之前的SII信息，0表示未找到
 */
static int ecx_lookup_prev_sii(ecx_contextt *context, uint16 slave)
{
   int i, nSM;
   if ((slave > 1) && (*(context->slavecount) > 0))
   {
      i = 1;
      // 遍历之前的从站，查找相同制造商ID、设备ID和版本的从站
      while(((context->slavelist[i].eep_man != context->slavelist[slave].eep_man) ||
             (context->slavelist[i].eep_id  != context->slavelist[slave].eep_id ) ||
             (context->slavelist[i].eep_rev != context->slavelist[slave].eep_rev)) &&
            (i < slave))
      {
         i++;
      }
      // 找到相同的从站
      if(i < slave)
      {
         // 复制协议详情
         context->slavelist[slave].CoEdetails = context->slavelist[i].CoEdetails;
         context->slavelist[slave].FoEdetails = context->slavelist[i].FoEdetails;
         context->slavelist[slave].EoEdetails = context->slavelist[i].EoEdetails;
         context->slavelist[slave].SoEdetails = context->slavelist[i].SoEdetails;
         // 复制blockLRW标志
         if(context->slavelist[i].blockLRW > 0)
         {
            context->slavelist[slave].blockLRW = 1;
            context->slavelist[0].blockLRW++;
         }
         // 复制E-bus电流
         context->slavelist[slave].Ebuscurrent = context->slavelist[i].Ebuscurrent;
         context->slavelist[0].Ebuscurrent += context->slavelist[slave].Ebuscurrent;
         // 复制从站名称
         memcpy(context->slavelist[slave].name, context->slavelist[i].name, EC_MAXNAME + 1);
         // 复制SM配置
         for( nSM=0 ; nSM < EC_MAXSM ; nSM++ )
         {
            context->slavelist[slave].SM[nSM].StartAddr = context->slavelist[i].SM[nSM].StartAddr;
            context->slavelist[slave].SM[nSM].SMlength  = context->slavelist[i].SM[nSM].SMlength;
            context->slavelist[slave].SM[nSM].SMflags   = context->slavelist[i].SM[nSM].SMflags;
         }
         // 复制FMMU功能
         context->slavelist[slave].FMMU0func = context->slavelist[i].FMMU0func;
         context->slavelist[slave].FMMU1func = context->slavelist[i].FMMU1func;
         context->slavelist[slave].FMMU2func = context->slavelist[i].FMMU2func;
         context->slavelist[slave].FMMU3func = context->slavelist[i].FMMU3func;
         EC_PRINT("Copy SII slave %d from %d.\n", slave, i);
         return 1;
      }
   }
   return 0;
}

/**
 * 枚举并初始化所有从站
 *
 * 该函数是EtherCAT配置的核心函数，负责初始化和配置总线上的所有从站。
 * 主要功能包括：
 * 1. 初始化EtherCAT上下文
 * 2. 检测总线上的从站
 * 3. 读取从站的基本信息（接口类型、节点地址、别名地址等）
 * 4. 读取从站EEPROM信息（制造商ID、设备ID、版本等）
 * 5. 配置从站邮箱
 * 6. 处理从站拓扑结构
 * 7. 根据配置表或SII信息配置从站
 * 8. 编程从站Sync Manager
 * 9. 将从站状态设置为PRE_OP
 *
 * @param[in] context    EtherCAT上下文结构体
 * @param[in] usetable   是否使用配置表初始化从站（TRUE/FALSE）
 * @return 从站发现数据报的工作计数器，即找到的从站数量
 */
int ecx_config_init(ecx_contextt *context, uint8 usetable)
{
   uint16 slave, ADPh, configadr, ssigen;     // 从站索引、地址指针、配置地址、SII通用部分索引
   uint16 topology, estat;                    // 拓扑结构、EEPROM状态
   int16 topoc, slavec, aliasadr;             // 拓扑计数、从站计数、别名地址
   uint8 b,h;                                 // 临时变量
   uint8 SMc;                                 // Sync Manager计数器
   uint32 eedat;                              // EEPROM数据
   int wkc, cindex, nSM;                      // 工作计数器、配置索引、Sync Manager数量
   uint16 val16;                              // 16位临时变量

   EC_PRINT("ec_config_init %d\n", usetable);

   // 初始化EtherCAT上下文。清除从站列表和组列表，并为每个组设置默认的逻辑起始地址。
   ecx_init_context(context);

   // 检测总线上的从站数量
   wkc = ecx_detect_slaves(context);

   if (wkc > 0)
   {
      // 将所有从站设置为默认值
      ecx_set_slaves_to_default(context);

      // 遍历所有从站，配置基本信息
      for (slave = 1; slave <= *(context->slavecount); slave++)
      {
         // 计算地址指针（AP寻址方式）
         ADPh = (uint16)(1 - slave);

         // 读取从站PDI接口类型 0x0140
         val16 = ecx_APRDw(context->port, ADPh, ECT_REG_PDICTL, EC_TIMEOUTRET3);
         context->slavelist[slave].Itype = etohs(val16);

         /* 使用节点偏移量来提高网络帧的可读性 */
         /* 这对可寻址从站的数量没有影响（自动环绕） */
         // 设置从站节点地址
         ecx_APWRw(context->port, ADPh, ECT_REG_STADR, htoes(slave + EC_NODEOFFSET) , EC_TIMEOUTRET3);

         // 设置非EtherCAT帧的处理方式
         if (slave == 1)
         {
            b = 1; /* 第一个从站丢弃非EtherCAT帧 */
         }
         else
         {
            b = 0; /* 后续从站传递所有帧 */
         }
         // 设置从站是否丢弃非EtherCAT帧
         ecx_APWRw(context->port, ADPh, ECT_REG_DLCTL, htoes(b), EC_TIMEOUTRET3);

         // 读取配置地址
         configadr = ecx_APRDw(context->port, ADPh, ECT_REG_STADR, EC_TIMEOUTRET3);
         configadr = etohs(configadr);
         context->slavelist[slave].configadr = configadr;

         // 读取别名地址
         ecx_FPRD(context->port, configadr, ECT_REG_ALIAS, sizeof(aliasadr), &aliasadr, EC_TIMEOUTRET3);
         context->slavelist[slave].aliasadr = etohs(aliasadr);

         // 读取EEPROM状态 0x502
         ecx_FPRD(context->port, configadr, ECT_REG_EEPSTAT, sizeof(estat), &estat, EC_TIMEOUTRET3);
         estat = etohs(estat);

         // 检查从站是否支持8字节块读取
         if (estat & EC_ESTAT_R64)
         {
            context->slavelist[slave].eep_8byte = 1;
         }

         // 准备读取EEPROM制造商信息
         ecx_readeeprom1(context, slave, ECT_SII_MANUF);
      }
      // 读取所有从站的制造商ID
      for (slave = 1; slave <= *(context->slavecount); slave++)
      {
         eedat = ecx_readeeprom2(context, slave, EC_TIMEOUTEEP);
         context->slavelist[slave].eep_man = etohl(eedat);
         // 准备读取设备ID
         ecx_readeeprom1(context, slave, ECT_SII_ID);
      }

      // 读取所有从站的设备ID
      for (slave = 1; slave <= *(context->slavecount); slave++)
      {
         eedat = ecx_readeeprom2(context, slave, EC_TIMEOUTEEP);
         context->slavelist[slave].eep_id = etohl(eedat);
         // 准备读取版本信息
         ecx_readeeprom1(context, slave, ECT_SII_REV);
      }

      // 读取所有从站的版本信息
      for (slave = 1; slave <= *(context->slavecount); slave++)
      {
         eedat = ecx_readeeprom2(context, slave, EC_TIMEOUTEEP);
         context->slavelist[slave].eep_rev = etohl(eedat);
         // 准备读取邮箱写入地址和大小
         ecx_readeeprom1(context, slave, ECT_SII_RXMBXADR);
      }

      // 读取所有从站的邮箱写入地址和大小
      for (slave = 1; slave <= *(context->slavecount); slave++)
      {
         eedat = ecx_readeeprom2(context, slave, EC_TIMEOUTEEP);
         context->slavelist[slave].mbx_wo = (uint16)LO_WORD(etohl(eedat));
         context->slavelist[slave].mbx_l = (uint16)HI_WORD(etohl(eedat));

         // 如果邮箱大小大于0，准备读取邮箱读取地址
         if (context->slavelist[slave].mbx_l > 0)
         {
            ecx_readeeprom1(context, slave, ECT_SII_TXMBXADR);
         }
      }

      // 读取所有从站的邮箱读取地址和大小
      for (slave = 1; slave <= *(context->slavecount); slave++)
      {
         if (context->slavelist[slave].mbx_l > 0)
         {
            eedat = ecx_readeeprom2(context, slave, EC_TIMEOUTEEP);
            context->slavelist[slave].mbx_ro = (uint16)LO_WORD(etohl(eedat));
            context->slavelist[slave].mbx_rl = (uint16)HI_WORD(etohl(eedat));

            // 如果读取邮箱长度为0，使用写入邮箱长度
            if (context->slavelist[slave].mbx_rl == 0)
            {
               context->slavelist[slave].mbx_rl = context->slavelist[slave].mbx_l;
            }

            // 准备读取邮箱协议
            ecx_readeeprom1(context, slave, ECT_SII_MBXPROTO);
         }
         // 读取从站配置地址
         configadr = context->slavelist[slave].configadr;

         // 检查从站是否支持分布式时钟(DC) 0x0008
         val16 = ecx_FPRDw(context->port, configadr, ECT_REG_ESCSUP, EC_TIMEOUTRET3);
         if ((etohs(val16) & 0x04) > 0)
         {
            context->slavelist[slave].hasdc = TRUE;
         }
         else
         {
            context->slavelist[slave].hasdc = FALSE;
         }

         // 从DL状态中提取拓扑结构 0x0110
         topology = ecx_FPRDw(context->port, configadr, ECT_REG_DLSTAT, EC_TIMEOUTRET3);
         topology = etohs(topology);
         h = 0;  // 活动端口数量
         b = 0;  // 活动端口掩码

         // 检查各端口状态
         if ((topology & 0x0300) == 0x0200) /* port0 打开且通信建立 */
         {
            h++;
            b |= 0x01;
         }
         if ((topology & 0x0c00) == 0x0800) /* port1 打开且通信建立 */
         {
            h++;
            b |= 0x02;
         }
         if ((topology & 0x3000) == 0x2000) /* port2 打开且通信建立 */
         {
            h++;
            b |= 0x04;
         }
         if ((topology & 0xc000) == 0x8000) /* port3 打开且通信建立 */
         {
            h++;
            b |= 0x08;
         }

         /* ptype = 物理类型 读取 0x0007 */
         val16 = ecx_FPRDw(context->port, configadr, ECT_REG_PORTDES, EC_TIMEOUTRET3);
         context->slavelist[slave].ptype = LO_BYTE(etohs(val16));
         context->slavelist[slave].topology = h;
         context->slavelist[slave].activeports = b;
         /* 0=no links, not possible             */
         /* 1=1 link  , end of line              */
         /* 2=2 links , one before and one after */
         /* 3=3 links , split point              */
         /* 4=4 links , cross point              */

         /* 搜索父从站 */
         context->slavelist[slave].parent = 0; /* 父从站是主站 */
         if (slave > 1)
         {
            topoc = 0;
            slavec = slave - 1;
            do
            {
               topology = context->slavelist[slavec].topology;
               if (topology == 1)
               {
                  topoc--; /* 找到端点 */
               }
               if (topology == 3)
               {
                  topoc++; /* 找到分支 */
               }
               if (topology == 4)
               {
                  topoc += 2; /* 找到交叉点 */
               }
               if (((topoc >= 0) && (topology > 1)) ||
                   (slavec == 1)) /* 找到父从站 */
               {
                  context->slavelist[slave].parent = slavec;
                  slavec = 1;
               }
               slavec--;
            }
            while (slavec > 0);
         }

         // 检查从站状态是否为INIT
         (void)ecx_statecheck(context, slave, EC_STATE_INIT,  EC_TIMEOUTSTATE);

         /* 如果从站有邮箱，设置默认邮箱配置 */
         if (context->slavelist[slave].mbx_l>0)
         {
            // 设置Sync Manager类型
            context->slavelist[slave].SMtype[0] = 1;  // SM0: 邮箱入
            context->slavelist[slave].SMtype[1] = 2;  // SM1: 邮箱出
            context->slavelist[slave].SMtype[2] = 3;  // SM2: 过程数据出
            context->slavelist[slave].SMtype[3] = 4;  // SM3: 过程数据入

            // 配置SM0（邮箱入）
            context->slavelist[slave].SM[0].StartAddr = htoes(context->slavelist[slave].mbx_wo);
            context->slavelist[slave].SM[0].SMlength = htoes(context->slavelist[slave].mbx_l);
            context->slavelist[slave].SM[0].SMflags = htoel(EC_DEFAULTMBXSM0);

            // 配置SM1（邮箱出）
            context->slavelist[slave].SM[1].StartAddr = htoes(context->slavelist[slave].mbx_ro);
            context->slavelist[slave].SM[1].SMlength = htoes(context->slavelist[slave].mbx_rl);
            context->slavelist[slave].SM[1].SMflags = htoel(EC_DEFAULTMBXSM1);

            // 读取支持的邮箱协议
            eedat = ecx_readeeprom2(context, slave, EC_TIMEOUTEEP);
            context->slavelist[slave].mbx_proto = (uint16)etohl(eedat);
         }

         cindex = 0;
         /* 使用配置表？ */
         if (usetable == 1)
         {
            cindex = ecx_config_from_table(context, slave);
         }

         /* 从站不在配置表中，并且未通过SII查找信息 */
         if (!cindex && !ecx_lookup_prev_sii(context, slave))
         {
            // 查找SII通用部分
            ssigen = ecx_siifind(context, slave, ECT_SII_GENERAL);

            /* SII通用部分 */
            if (ssigen)
            {
               // 读取CoE、FoE、EoE、SoE详情
               context->slavelist[slave].CoEdetails = ecx_siigetbyte(context, slave, ssigen + 0x07);
               context->slavelist[slave].FoEdetails = ecx_siigetbyte(context, slave, ssigen + 0x08);
               context->slavelist[slave].EoEdetails = ecx_siigetbyte(context, slave, ssigen + 0x09);
               context->slavelist[slave].SoEdetails = ecx_siigetbyte(context, slave, ssigen + 0x0a);

               // 检查是否支持块LRW
               if((ecx_siigetbyte(context, slave, ssigen + 0x0d) & 0x02) > 0)
               {
                  context->slavelist[slave].blockLRW = 1;
                  context->slavelist[0].blockLRW++;
               }

               // 读取E-bus电流
               context->slavelist[slave].Ebuscurrent = ecx_siigetbyte(context, slave, ssigen + 0x0e);
               context->slavelist[slave].Ebuscurrent += ecx_siigetbyte(context, slave, ssigen + 0x0f) << 8;
               context->slavelist[0].Ebuscurrent += context->slavelist[slave].Ebuscurrent;
            }

            /* SII字符串部分 */
            if (ecx_siifind(context, slave, ECT_SII_STRING) > 0)
            {
               // 读取从站名称
               ecx_siistring(context, context->slavelist[slave].name, slave, 1);
            }
            /* 未找到从站名称，使用构造的名称 */
            else
            {
               sprintf(context->slavelist[slave].name, "? M:%8.8x I:%8.8x",
                       (unsigned int)context->slavelist[slave].eep_man,
                       (unsigned int)context->slavelist[slave].eep_id);
            }

            /* SII Sync Manager部分 */
            nSM = ecx_siiSM(context, slave, context->eepSM);
            if (nSM>0)
            {
               // 配置SM0
               context->slavelist[slave].SM[0].StartAddr = htoes(context->eepSM->PhStart);
               context->slavelist[slave].SM[0].SMlength = htoes(context->eepSM->Plength);
               context->slavelist[slave].SM[0].SMflags =
                  htoel((context->eepSM->Creg) + (context->eepSM->Activate << 16));

               // 配置其他SM
               SMc = 1;
               while ((SMc < EC_MAXSM) &&  ecx_siiSMnext(context, slave, context->eepSM, SMc))
               {
                  context->slavelist[slave].SM[SMc].StartAddr = htoes(context->eepSM->PhStart);
                  context->slavelist[slave].SM[SMc].SMlength = htoes(context->eepSM->Plength);
                  context->slavelist[slave].SM[SMc].SMflags =
                     htoel((context->eepSM->Creg) + (context->eepSM->Activate << 16));
                  SMc++;
               }
            }

            /* SII FMMU部分 */
            if (ecx_siiFMMU(context, slave, context->eepFMMU))
            {
               // 配置FMMU功能
               if (context->eepFMMU->FMMU0 !=0xff)
               {
                  context->slavelist[slave].FMMU0func = context->eepFMMU->FMMU0;
               }
               if (context->eepFMMU->FMMU1 !=0xff)
               {
                  context->slavelist[slave].FMMU1func = context->eepFMMU->FMMU1;
               }
               if (context->eepFMMU->FMMU2 !=0xff)
               {
                  context->slavelist[slave].FMMU2func = context->eepFMMU->FMMU2;
               }
               if (context->eepFMMU->FMMU3 !=0xff)
               {
                  context->slavelist[slave].FMMU3func = context->eepFMMU->FMMU3;
               }
            }
         }

         // 如果从站有邮箱
         if (context->slavelist[slave].mbx_l > 0)
         {
            // 检查SM0配置是否正确
            if (context->slavelist[slave].SM[0].StartAddr == 0x0000) /* 不应该发生 */
            {
               EC_PRINT("Slave %d has no proper mailbox in configuration, try default.\n", slave);
               // 使用默认配置
               context->slavelist[slave].SM[0].StartAddr = htoes(0x1000);
               context->slavelist[slave].SM[0].SMlength = htoes(0x0080);
               context->slavelist[slave].SM[0].SMflags = htoel(EC_DEFAULTMBXSM0);
               context->slavelist[slave].SMtype[0] = 1;
            }

            // 检查SM1配置是否正确
            if (context->slavelist[slave].SM[1].StartAddr == 0x0000) /* 不应该发生 */
            {
               EC_PRINT("Slave %d has no proper mailbox out configuration, try default.\n", slave);
               // 使用默认配置
               context->slavelist[slave].SM[1].StartAddr = htoes(0x1080);
               context->slavelist[slave].SM[1].SMlength = htoes(0x0080);
               context->slavelist[slave].SM[1].SMflags = htoel(EC_DEFAULTMBXSM1);
               context->slavelist[slave].SMtype[1] = 2;
            }

            /* 为从站编程SM0（邮箱入）和SM1（邮箱出） */
            /* 在一个数据报中写入两个SM可以解决旧版NETX的时序问题 */
            ecx_FPWR(context->port, configadr, ECT_REG_SM0, sizeof(ec_smt) * 2,
               &(context->slavelist[slave].SM[0]), EC_TIMEOUTRET3);
         }

         /* 一些从站在init->preop转换时需要EEPROM对PDI可用 */
         ecx_eeprom2pdi(context, slave);

         /* 用户可以覆盖自动状态更改 */
         if (context->manualstatechange == 0)
         {
            /* 请求从站进入pre_op状态 */
            ecx_FPWRw(context->port,
               configadr,
               ECT_REG_ALCTL,
               htoes(EC_STATE_PRE_OP | EC_STATE_ACK),
               EC_TIMEOUTRET3); /* 设置preop状态 */
         }
      }
   }
   return wkc;
}

/**
 * 查找之前相同从站的PDO映射
 *
 * 如果从站有SII映射且之前已经处理过相同ID的从站，
 * 则复用之前的映射数据。这是安全的，因为相同ID从站的SII映射是相同的。
 *
 * 该优化可以显著减少配置时间，避免重复读取PDO映射。
 *
 * @param[in] context EtherCAT上下文结构体
 * @param[in] slave   当前从站编号
 * @param[out] Osize  输出数据大小（位）
 * @param[out] Isize  输入数据大小（位）
 * @return 1表示找到并复制了之前的映射信息，0表示未找到
 */
static int ecx_lookup_mapping(ecx_contextt *context, uint16 slave, uint32 *Osize, uint32 *Isize)
{
   int i, nSM;
   if ((slave > 1) && (*(context->slavecount) > 0))
   {
      i = 1;
      // 遍历之前的从站，查找相同制造商ID、设备ID和版本的从站
      while(((context->slavelist[i].eep_man != context->slavelist[slave].eep_man) ||
             (context->slavelist[i].eep_id  != context->slavelist[slave].eep_id ) ||
             (context->slavelist[i].eep_rev != context->slavelist[slave].eep_rev)) &&
            (i < slave))
      {
         i++;
      }
      // 找到相同的从站
      if(i < slave)
      {
         // 复制SM长度和类型
         for( nSM=0 ; nSM < EC_MAXSM ; nSM++ )
         {
            context->slavelist[slave].SM[nSM].SMlength = context->slavelist[i].SM[nSM].SMlength;
            context->slavelist[slave].SMtype[nSM] = context->slavelist[i].SMtype[nSM];
         }
         // 复制输入输出大小
         *Osize = context->slavelist[i].Obits;
         *Isize = context->slavelist[i].Ibits;
         context->slavelist[slave].Obits = (uint16)*Osize;
         context->slavelist[slave].Ibits = (uint16)*Isize;
         EC_PRINT("Copy mapping slave %d from %d.\n", slave, i);
         return 1;
      }
   }
   return 0;
}

/**
 * 通过CoE和SoE协议映射PDO
 *
 * 该函数使用CoE（CANopen over EtherCAT）和SoE（Servo over EtherCAT）
 * 协议读取从站的PDO映射信息。
 *
 * 映射策略：
 * 1. 首先检查从站是否支持CoE协议
 * 2. 如果支持CoE且支持完全访问(CA)，使用CA方式读取PDO映射
 * 3. 如果CA不可用，使用标准SDO方式读取PDO映射
 * 4. 如果CoE映射失败且从站支持SoE，使用SoE方式读取IDN映射
 *
 * @param[in] context   EtherCAT上下文结构体
 * @param[in] slave     从站编号
 * @param[in] thread_n  线程号（用于多线程映射）
 * @return 1表示成功
 */
static int ecx_map_coe_soe(ecx_contextt *context, uint16 slave, int thread_n)
{
   uint32 Isize, Osize;
   int rval;

   ecx_statecheck(context, slave, EC_STATE_PRE_OP, EC_TIMEOUTSTATE); /* check state change pre-op */

   EC_PRINT(" >Slave %d, configadr %x, state %2.2x\n",
            slave, context->slavelist[slave].configadr, context->slavelist[slave].state);

   /* execute special slave configuration hook Pre-Op to Safe-OP */
   // 执行从站特殊配置钩子函数（PRE_OP到SAFE_OP转换）
   if(context->slavelist[slave].PO2SOconfig) /* only if registered */
   {
      context->slavelist[slave].PO2SOconfig(slave);
   }
   if (context->slavelist[slave].PO2SOconfigx) /* only if registered */
   {
      context->slavelist[slave].PO2SOconfigx(context, slave);
   }
   /* if slave not found in configlist find IO mapping in slave self */
   // 如果从站不在配置表中，从从站自身查找IO映射
   if (!context->slavelist[slave].configindex)
   {
      Isize = 0;
      Osize = 0;
      // 检查从站是否支持CoE协议
      if (context->slavelist[slave].mbx_proto & ECT_MBXPROT_COE) /* has CoE */
      {
         rval = 0;
         // 检查是否支持完全访问(CA)
         if (context->slavelist[slave].CoEdetails & ECT_COEDET_SDOCA) /* has Complete Access */
         {
            /* read PDO mapping via CoE and use Complete Access */
            // 使用CA方式读取PDO映射，效率更高
            rval = ecx_readPDOmapCA(context, slave, thread_n, &Osize, &Isize);
         }
         if (!rval) /* CA not available or not succeeded */
         {
            /* read PDO mapping via CoE */
            // 使用标准SDO方式读取PDO映射
            rval = ecx_readPDOmap(context, slave, &Osize, &Isize);
         }
         EC_PRINT("  CoE Osize:%u Isize:%u\n", Osize, Isize);
      }
      // 如果CoE映射失败且从站支持SoE协议
      if ((!Isize && !Osize) && (context->slavelist[slave].mbx_proto & ECT_MBXPROT_SOE)) /* has SoE */
      {
         /* read AT / MDT mapping via SoE */
         // 使用SoE方式读取AT/MDT映射
         rval = ecx_readIDNmap(context, slave, &Osize, &Isize);
         context->slavelist[slave].SM[2].SMlength = htoes((uint16)((Osize + 7) / 8));
         context->slavelist[slave].SM[3].SMlength = htoes((uint16)((Isize + 7) / 8));
         EC_PRINT("  SoE Osize:%u Isize:%u\n", Osize, Isize);
      }
      // 保存输入输出位数
      context->slavelist[slave].Obits = (uint16)Osize;
      context->slavelist[slave].Ibits = (uint16)Isize;
   }

   return 1;
}

/**
 * 通过SII（从站信息接口）映射PDO
 *
 * 该函数从从站EEPROM的SII区域读取PDO映射信息。
 * 如果之前已经处理过相同ID的从站，则复用之前的映射数据。
 *
 * SII PDO映射流程：
 * 1. 检查是否已有输入输出大小
 * 2. 如果没有，查找之前相同从站的映射
 * 3. 如果未找到，从SII读取PDO映射
 * 4. 配置SM长度和类型
 *
 * @param[in] context EtherCAT上下文结构体
 * @param[in] slave   从站编号
 * @return 1表示成功
 */
static int ecx_map_sii(ecx_contextt *context, uint16 slave)
{
   uint32 Isize, Osize;
   int nSM;
   ec_eepromPDOt eepPDO;

   Osize = context->slavelist[slave].Obits;
   Isize = context->slavelist[slave].Ibits;

   if (!Isize && !Osize) /* find PDO in previous slave with same ID */
   {
      // 查找之前相同从站的映射
      (void)ecx_lookup_mapping(context, slave, &Osize, &Isize);
   }
   if (!Isize && !Osize) /* find PDO mapping by SII */
   {
      // 从SII读取PDO映射
      memset(&eepPDO, 0, sizeof(eepPDO));
      // 读取输入PDO映射
      Isize = ecx_siiPDO(context, slave, &eepPDO, 0);
      EC_PRINT("  SII Isize:%u\n", Isize);
      // 配置输入SM
      for( nSM=0 ; nSM < EC_MAXSM ; nSM++ )
      {
         if (eepPDO.SMbitsize[nSM] > 0)
         {
            context->slavelist[slave].SM[nSM].SMlength =  htoes((eepPDO.SMbitsize[nSM] + 7) / 8);
            context->slavelist[slave].SMtype[nSM] = 4;  // 类型4：输入过程数据
            EC_PRINT("    SM%d length %d\n", nSM, eepPDO.SMbitsize[nSM]);
         }
      }
      // 读取输出PDO映射
      Osize = ecx_siiPDO(context, slave, &eepPDO, 1);
      EC_PRINT("  SII Osize:%u\n", Osize);
      // 配置输出SM
      for( nSM=0 ; nSM < EC_MAXSM ; nSM++ )
      {
         if (eepPDO.SMbitsize[nSM] > 0)
         {
            context->slavelist[slave].SM[nSM].SMlength =  htoes((eepPDO.SMbitsize[nSM] + 7) / 8);
            context->slavelist[slave].SMtype[nSM] = 3;  // 类型3：输出过程数据
            EC_PRINT("    SM%d length %d\n", nSM, eepPDO.SMbitsize[nSM]);
         }
      }
   }
   // 保存输入输出位数
   context->slavelist[slave].Obits = (uint16)Osize;
   context->slavelist[slave].Ibits = (uint16)Isize;
   EC_PRINT("     ISIZE:%d %d OSIZE:%d\n",
      context->slavelist[slave].Ibits, Isize,context->slavelist[slave].Obits);

   return 1;
}

/**
 * 编程从站的同步管理器(SM)
 *
 * 该函数将配置好的同步管理器写入从站寄存器。
 * 根据SM类型和长度设置使能标志，并计算输入输出字节数。
 *
 * SM编程流程：
 * 1. 如果从站没有邮箱且SM0/SM1有起始地址，编程SM0/SM1
 * 2. 编程SM2到SMx（过程数据SM）
 * 3. 根据SM长度设置使能标志
 * 4. 计算输入输出字节数
 *
 * SM类型：
 * - 类型1：邮箱入
 * - 类型2：邮箱出
 * - 类型3：输出过程数据
 * - 类型4：输入过程数据
 *
 * @param[in] context EtherCAT上下文结构体
 * @param[in] slave   从站编号
 * @return 1表示成功
 */
static int ecx_map_sm(ecx_contextt *context, uint16 slave)
{
   uint16 configadr;
   int nSM;

   configadr = context->slavelist[slave].configadr;

   EC_PRINT("  SM programming\n");
   // 如果从站没有邮箱且SM0有起始地址，编程SM0
   if (!context->slavelist[slave].mbx_l && context->slavelist[slave].SM[0].StartAddr)
   {
      ecx_FPWR(context->port, configadr, ECT_REG_SM0,
         sizeof(ec_smt), &(context->slavelist[slave].SM[0]), EC_TIMEOUTRET3);
      EC_PRINT("    SM0 Type:%d StartAddr:%4.4x Flags:%8.8x\n",
          context->slavelist[slave].SMtype[0],
          etohs(context->slavelist[slave].SM[0].StartAddr),
          etohl(context->slavelist[slave].SM[0].SMflags));
   }
   // 如果从站没有邮箱且SM1有起始地址，编程SM1
   if (!context->slavelist[slave].mbx_l && context->slavelist[slave].SM[1].StartAddr)
   {
      ecx_FPWR(context->port, configadr, ECT_REG_SM1,
         sizeof(ec_smt), &context->slavelist[slave].SM[1], EC_TIMEOUTRET3);
      EC_PRINT("    SM1 Type:%d StartAddr:%4.4x Flags:%8.8x\n",
          context->slavelist[slave].SMtype[1],
          etohs(context->slavelist[slave].SM[1].StartAddr),
          etohl(context->slavelist[slave].SM[1].SMflags));
   }
   /* program SM2 to SMx */
   // 编程SM2到SMx（过程数据SM）
   for( nSM = 2 ; nSM < EC_MAXSM ; nSM++ )
   {
      if (context->slavelist[slave].SM[nSM].StartAddr)
      {
         /* check if SM length is zero -> clear enable flag */
         // 如果SM长度为0，清除使能标志
         if( context->slavelist[slave].SM[nSM].SMlength == 0)
         {
            context->slavelist[slave].SM[nSM].SMflags =
               htoel( etohl(context->slavelist[slave].SM[nSM].SMflags) & EC_SMENABLEMASK);
         }
         /* if SM length is non zero always set enable flag */
         // 如果SM长度不为0，设置使能标志
         else
         {
            context->slavelist[slave].SM[nSM].SMflags =
               htoel( etohl(context->slavelist[slave].SM[nSM].SMflags) | ~EC_SMENABLEMASK);
         }
         // 写入SM配置到从站
         ecx_FPWR(context->port, configadr, (uint16)(ECT_REG_SM0 + (nSM * sizeof(ec_smt))),
            sizeof(ec_smt), &context->slavelist[slave].SM[nSM], EC_TIMEOUTRET3);
         EC_PRINT("    SM%d Type:%d StartAddr:%4.4x Flags:%8.8x\n", nSM,
             context->slavelist[slave].SMtype[nSM],
             etohs(context->slavelist[slave].SM[nSM].StartAddr),
             etohl(context->slavelist[slave].SM[nSM].SMflags));
      }
   }
   // 计算输入字节数
   if (context->slavelist[slave].Ibits > 7)
   {
      context->slavelist[slave].Ibytes = (context->slavelist[slave].Ibits + 7) / 8;
   }
   // 计算输出字节数
   if (context->slavelist[slave].Obits > 7)
   {
      context->slavelist[slave].Obytes = (context->slavelist[slave].Obits + 7) / 8;
   }

   return 1;
}

#if EC_MAX_MAPT > 1
OSAL_THREAD_FUNC ecx_mapper_thread(void *param)
{
   ecx_mapt_t *maptp;
   maptp = param;
   ecx_map_coe_soe(maptp->context, maptp->slave, maptp->thread_n);
   maptp->running = 0;
}

static int ecx_find_mapt(void)
{
   int p;
   p = 0;
   while((p < EC_MAX_MAPT) && ecx_mapt[p].running)
   {
      p++;
   }
   if(p < EC_MAX_MAPT)
   {
      return p;
   }
   else
   {
      return -1;
   }
}
#endif

static int ecx_get_threadcount(void)
{
   int thrc, thrn;
   thrc = 0;
   for(thrn = 0 ; thrn < EC_MAX_MAPT ; thrn++)
   {
      thrc += ecx_mapt[thrn].running;
   }
   return thrc;
}

/** 查找并配置指定组中从站的PDO映射。
 *
 * 该函数使用CoE（CANopen over EtherCAT）、SoE（Servo over EtherCAT）
 * 和SII（从站信息接口）方法发现指定组中所有从站的PDO映射。
 * 它根据发现的映射为每个从站配置同步管理器。
 *
 * @param[in] context = 包含从站列表的上下文结构体
 * @param[in] group   = 组号，0表示所有组
 */
/**
 * 查找指定组中所有从站的PDO映射
 *
 * 该函数负责查找指定组中所有从站的PDO映射信息。
 * 主要功能包括：
 * 1. 多线程查找CoE和SoE映射（如果支持）
 * 2. 查找SII映射
 * 3. 配置同步管理器(SM)
 *
 * @param[in] context EtherCAT上下文结构体
 * @param[in] group   组号，0表示所有组
 */
static void ecx_config_find_mappings(ecx_contextt *context, uint8 group)
{
   int thrn, thrc;       // 线程号、线程计数
   uint16 slave;         // 从站索引

   // 初始化所有映射线程状态为非运行
   for (thrn = 0; thrn < EC_MAX_MAPT; thrn++)
   {
      ecx_mapt[thrn].running = 0;
   }

   /* 在多线程中查找从站的CoE和SoE映射 */
   for (slave = 1; slave <= *(context->slavecount); slave++)
   {
      // 检查从站是否属于指定组
      if (!group || (group == context->slavelist[slave].group))
      {
#if EC_MAX_MAPT > 1
         /* 多线程版本 */
         // 查找空闲的映射线程
         while ((thrn = ecx_find_mapt()) < 0)
         {
            osal_usleep(1000);
         }

         // 设置线程参数
         ecx_mapt[thrn].context = context;
         ecx_mapt[thrn].slave = slave;
         ecx_mapt[thrn].thread_n = thrn;
         ecx_mapt[thrn].running = 1;

         // 创建映射线程
         osal_thread_create(&(ecx_threadh[thrn]), 128000,
            &ecx_mapper_thread, &(ecx_mapt[thrn]));
#else
         /* 串行版本 */
         ecx_map_coe_soe(context, slave, 0);
#endif
      }
   }

   /* 等待所有线程完成 */
   do
   {
      thrc = ecx_get_threadcount();
      if (thrc)
      {
         osal_usleep(1000);
      }
   } while (thrc);

   /* 查找从站的SII映射并配置SM */
   for (slave = 1; slave <= *(context->slavecount); slave++)
   {
      // 检查从站是否属于指定组
      if (!group || (group == context->slavelist[slave].group))
      {
         // 查找SII映射
         ecx_map_sii(context, slave);
         // 配置同步管理器
         ecx_map_sm(context, slave);
      }
   }
}

/** 为指定组中的从站创建输入FMMU映射。
 *
 * 该函数为从站的输入数据配置FMMU（现场总线内存管理单元）。
 * 它将从站的输入同步管理器映射到IOmap中的逻辑地址空间。
 * 该函数同时处理面向位和面向字节的从站。
 *
 * @param[in]     context  = 包含从站列表的上下文结构体
 * @param[in,out] pIOmap   = 指向IOmap缓冲区的指针
 * @param[in]     group    = 组号，0表示所有组
 * @param[in]     slave    = 要创建输入映射的从站号
 * @param[in,out] LogAddr  = 指向当前逻辑地址的指针（会被更新）
 * @param[in,out] BitPos   = 指向当前位位置的指针（会被更新）
 */
/**
 * 为从站创建输入映射
 *
 * 该函数负责为指定从站创建输入PDO映射，配置FMMU（现场总线内存映射单元）。
 * 主要功能包括：
 * 1. 查找贡献于输入映射的SM（同步管理器）
 * 2. 配置FMMU以映射物理地址到逻辑地址
 * 3. 设置输入指针和起始位
 * 4. 编程FMMU寄存器
 *
 * @param[in] context  EtherCAT上下文结构体
 * @param[in] pIOmap   指向IOmap缓冲区的指针
 * @param[in] group    组号
 * @param[in] slave    从站编号
 * @param[in,out] LogAddr 逻辑地址指针
 * @param[in,out] BitPos   位位置指针
 */
static void ecx_config_create_input_mappings(ecx_contextt *context, void *pIOmap,
   uint8 group, int16 slave, uint32 * LogAddr, uint8 * BitPos)
{
   int BitCount = 0;          // 位计数器
   int FMMUdone = 0;          // FMMU完成字节数
   int AddToInputsWKC = 0;    // 是否添加到输入工作计数器
   uint16 ByteCount = 0;      // 字节计数器
   uint16 FMMUsize = 0;       // FMMU大小
   uint8 SMc = 0;             // SM计数器
   uint16 EndAddr;            // 结束地址
   uint16 SMlength;           // SM长度
   uint16 configadr;          // 配置地址
   uint8 FMMUc;               // FMMU计数器

   EC_PRINT(" =Slave %d, INPUT MAPPING\n", slave);

   configadr = context->slavelist[slave].configadr;
   FMMUc = context->slavelist[slave].FMMUunused;

   // 如果从站有输出，查找空闲的FMMU
   if (context->slavelist[slave].Obits)
   {
      while (context->slavelist[slave].FMMU[FMMUc].LogStart)
      {
         FMMUc++;
      }
   }

   /* 搜索贡献于输入映射的SM */
   while ((SMc < EC_MAXSM) && (FMMUdone < ((context->slavelist[slave].Ibits + 7) / 8)))
   {
      EC_PRINT("    FMMU %d\n", FMMUc);

      // 查找类型为4（输入）的SM
      while ((SMc < (EC_MAXSM - 1)) && (context->slavelist[slave].SMtype[SMc] != 4))
      {
         SMc++;
      }

      EC_PRINT("      SM%d\n", SMc);

      // 设置FMMU物理起始地址
      context->slavelist[slave].FMMU[FMMUc].PhysStart =
         context->slavelist[slave].SM[SMc].StartAddr;

      // 获取SM长度
      SMlength = etohs(context->slavelist[slave].SM[SMc].SMlength);
      ByteCount += SMlength;
      BitCount += SMlength * 8;
      EndAddr = etohs(context->slavelist[slave].SM[SMc].StartAddr) + SMlength;

      /* 检查是否有更多SM用于输入 */
      while ((BitCount < context->slavelist[slave].Ibits) && (SMc < (EC_MAXSM - 1)))
      {
         SMc++;

         // 查找类型为4（输入）的SM
         while ((SMc < (EC_MAXSM - 1)) && (context->slavelist[slave].SMtype[SMc] != 4))
         {
            SMc++;
         }

         /* 如果来自更多SM的地址连接，使用一个FMMU，否则分解为多个FMMU */
         if (etohs(context->slavelist[slave].SM[SMc].StartAddr) > EndAddr)
         {
            break;
         }

         EC_PRINT("      SM%d\n", SMc);
         SMlength = etohs(context->slavelist[slave].SM[SMc].SMlength);
         ByteCount += SMlength;
         BitCount += SMlength * 8;
         EndAddr = etohs(context->slavelist[slave].SM[SMc].StartAddr) + SMlength;
      }

      /* 面向位的从站 */
      if (!context->slavelist[slave].Ibytes)
      {
         // 设置逻辑起始地址和位
         context->slavelist[slave].FMMU[FMMUc].LogStart = htoel(*LogAddr);
         context->slavelist[slave].FMMU[FMMUc].LogStartbit = *BitPos;

         // 计算位位置
         *BitPos += context->slavelist[slave].Ibits - 1;
         if (*BitPos > 7)
         {
            *LogAddr += 1;
            *BitPos -= 8;
         }

         // 计算FMMU大小
         FMMUsize = (uint16)(*LogAddr - etohl(context->slavelist[slave].FMMU[FMMUc].LogStart) + 1);
         context->slavelist[slave].FMMU[FMMUc].LogLength = htoes(FMMUsize);
         context->slavelist[slave].FMMU[FMMUc].LogEndbit = *BitPos;

         // 更新位位置
         *BitPos += 1;
         if (*BitPos > 7)
         {
            *LogAddr += 1;
            *BitPos -= 8;
         }
      }
      /* 面向字节的从站 */
      else
      {
         // 如果位位置不为0，移动到下一个字节
         if (*BitPos)
         {
            *LogAddr += 1;
            *BitPos = 0;
         }

         // 设置逻辑起始地址和位
         context->slavelist[slave].FMMU[FMMUc].LogStart = htoel(*LogAddr);
         context->slavelist[slave].FMMU[FMMUc].LogStartbit = *BitPos;
         *BitPos = 7;

         // 计算FMMU大小
         FMMUsize = ByteCount;
         if ((FMMUsize + FMMUdone)> (int)context->slavelist[slave].Ibytes)
         {
            FMMUsize = (uint16)(context->slavelist[slave].Ibytes - FMMUdone);
         }

         // 更新逻辑地址
         *LogAddr += FMMUsize;
         context->slavelist[slave].FMMU[FMMUc].LogLength = htoes(FMMUsize);
         context->slavelist[slave].FMMU[FMMUc].LogEndbit = *BitPos;
         *BitPos = 0;
      }

      // 更新FMMU完成计数
      FMMUdone += FMMUsize;

      // 如果FMMU长度不为0，编程FMMU
      if (context->slavelist[slave].FMMU[FMMUc].LogLength)
      {
         // 设置FMMU属性
         context->slavelist[slave].FMMU[FMMUc].PhysStartBit = 0;
         context->slavelist[slave].FMMU[FMMUc].FMMUtype = 1;      // 输入类型
         context->slavelist[slave].FMMU[FMMUc].FMMUactive = 1;    // 激活FMMU

         /* 编程输入FMMU */
         ecx_FPWR(context->port, configadr, ECT_REG_FMMU0 + (sizeof(ec_fmmut) * FMMUc),
            sizeof(ec_fmmut), &(context->slavelist[slave].FMMU[FMMUc]), EC_TIMEOUTRET3);

         /* 设置标志为输入FMMU添加一个工作计数器，
            单个ESC只能贡献一次 */
         AddToInputsWKC = 1;
      }

      // 设置输入指针
      if (!context->slavelist[slave].inputs)
      {
         if (group)
         {
            context->slavelist[slave].inputs =
               (uint8 *)(pIOmap) +
               etohl(context->slavelist[slave].FMMU[FMMUc].LogStart) -
               context->grouplist[group].logstartaddr;
         }
         else
         {
            context->slavelist[slave].inputs =
               (uint8 *)(pIOmap) +
               etohl(context->slavelist[slave].FMMU[FMMUc].LogStart);
         }

         // 设置输入起始位
         context->slavelist[slave].Istartbit =
            context->slavelist[slave].FMMU[FMMUc].LogStartbit;

         EC_PRINT("    Inputs %p startbit %d\n",
            context->slavelist[slave].inputs,
            context->slavelist[slave].Istartbit);
      }

      // 移动到下一个FMMU
      FMMUc++;
   }

   // 更新未使用的FMMU索引
   context->slavelist[slave].FMMUunused = FMMUc;

   /* 如果标志为真，为输入添加一个工作计数器 */
   if (AddToInputsWKC)
      context->grouplist[group].inputsWKC++;
}

/** 为指定组中的从站创建输出FMMU映射。
 *
 * 该函数为从站的输出数据配置FMMU（现场总线内存管理单元）。
 * 它将从站的输出同步管理器映射到IOmap中的逻辑地址空间。
 * 该函数同时处理面向位和面向字节的从站。
 *
 * @param[in]     context  = 包含从站列表的上下文结构体
 * @param[in,out] pIOmap   = 指向IOmap缓冲区的指针
 * @param[in]     group    = 组号，0表示所有组
 * @param[in]     slave    = 要创建输出映射的从站号
 * @param[in,out] LogAddr  = 指向当前逻辑地址的指针（会被更新）
 * @param[in,out] BitPos   = 指向当前位位置的指针（会被更新）
 */
/**
 * 为从站创建输出映射
 *
 * 该函数负责为指定从站创建输出PDO映射，配置FMMU（现场总线内存映射单元）。
 * 主要功能包括：
 * 1. 查找贡献于输出映射的SM（同步管理器）
 * 2. 配置FMMU以映射物理地址到逻辑地址
 * 3. 设置输出指针和起始位
 * 4. 编程FMMU寄存器
 *
 * @param[in] context  EtherCAT上下文结构体
 * @param[in] pIOmap   指向IOmap缓冲区的指针
 * @param[in] group    组号
 * @param[in] slave    从站编号
 * @param[in,out] LogAddr 逻辑地址指针
 * @param[in,out] BitPos   位位置指针
 */
static void ecx_config_create_output_mappings(ecx_contextt *context, void *pIOmap,
   uint8 group, int16 slave, uint32 * LogAddr, uint8 * BitPos)
{
   int BitCount = 0;          // 位计数器
   int FMMUdone = 0;          // FMMU完成字节数
   int AddToOutputsWKC = 0;   // 是否添加到输出工作计数器
   uint16 ByteCount = 0;      // 字节计数器
   uint16 FMMUsize = 0;       // FMMU大小
   uint8 SMc = 0;             // SM计数器
   uint16 EndAddr;            // 结束地址
   uint16 SMlength;           // SM长度
   uint16 configadr;          // 配置地址
   uint8 FMMUc;               // FMMU计数器

   EC_PRINT("  OUTPUT MAPPING\n");

   // 获取未使用的FMMU索引
   FMMUc = context->slavelist[slave].FMMUunused;
   configadr = context->slavelist[slave].configadr;

   /* 搜索贡献于输出映射的SM */
   while ((SMc < EC_MAXSM) && (FMMUdone < ((context->slavelist[slave].Obits + 7) / 8)))
   {
      EC_PRINT("    FMMU %d\n", FMMUc);

      // 查找类型为3（输出）的SM
      while ((SMc < (EC_MAXSM - 1)) && (context->slavelist[slave].SMtype[SMc] != 3))
      {
         SMc++;
      }

      EC_PRINT("      SM%d\n", SMc);

      // 设置FMMU物理起始地址
      context->slavelist[slave].FMMU[FMMUc].PhysStart =
         context->slavelist[slave].SM[SMc].StartAddr;

      // 获取SM长度
      SMlength = etohs(context->slavelist[slave].SM[SMc].SMlength);
      ByteCount += SMlength;
      BitCount += SMlength * 8;
      EndAddr = etohs(context->slavelist[slave].SM[SMc].StartAddr) + SMlength;

      /* 检查是否有更多SM用于输出 */
      while ((BitCount < context->slavelist[slave].Obits) && (SMc < (EC_MAXSM - 1)))
      {
         SMc++;

         // 查找类型为3（输出）的SM
         while ((SMc < (EC_MAXSM - 1)) && (context->slavelist[slave].SMtype[SMc] != 3))
         {
            SMc++;
         }

         /* 如果来自更多SM的地址连接，使用一个FMMU，否则分解为多个FMMU */
         if (etohs(context->slavelist[slave].SM[SMc].StartAddr) > EndAddr)
         {
            break;
         }

         EC_PRINT("      SM%d\n", SMc);
         SMlength = etohs(context->slavelist[slave].SM[SMc].SMlength);
         ByteCount += SMlength;
         BitCount += SMlength * 8;
         EndAddr = etohs(context->slavelist[slave].SM[SMc].StartAddr) + SMlength;
      }

      /* 面向位的从站 */
      if (!context->slavelist[slave].Obytes)
      {
         // 设置逻辑起始地址和位
         context->slavelist[slave].FMMU[FMMUc].LogStart = htoel(*LogAddr);
         context->slavelist[slave].FMMU[FMMUc].LogStartbit = *BitPos;

         // 计算位位置
         *BitPos += context->slavelist[slave].Obits - 1;
         if (*BitPos > 7)
         {
            *LogAddr += 1;
            *BitPos -= 8;
         }

         // 计算FMMU大小
         FMMUsize = (uint16)(*LogAddr - etohl(context->slavelist[slave].FMMU[FMMUc].LogStart) + 1);
         context->slavelist[slave].FMMU[FMMUc].LogLength = htoes(FMMUsize);
         context->slavelist[slave].FMMU[FMMUc].LogEndbit = *BitPos;

         // 更新位位置
         *BitPos += 1;
         if (*BitPos > 7)
         {
            *LogAddr += 1;
            *BitPos -= 8;
         }
      }
      /* 面向字节的从站 */
      else
      {
         // 如果位位置不为0，移动到下一个字节
         if (*BitPos)
         {
            *LogAddr += 1;
            *BitPos = 0;
         }

         // 设置逻辑起始地址和位
         context->slavelist[slave].FMMU[FMMUc].LogStart = htoel(*LogAddr);
         context->slavelist[slave].FMMU[FMMUc].LogStartbit = *BitPos;
         *BitPos = 7;

         // 计算FMMU大小
         FMMUsize = ByteCount;
         if ((FMMUsize + FMMUdone)> (int)context->slavelist[slave].Obytes)
         {
            FMMUsize = (uint16)(context->slavelist[slave].Obytes - FMMUdone);
         }

         // 更新逻辑地址
         *LogAddr += FMMUsize;
         context->slavelist[slave].FMMU[FMMUc].LogLength = htoes(FMMUsize);
         context->slavelist[slave].FMMU[FMMUc].LogEndbit = *BitPos;
         *BitPos = 0;
      }

      // 更新FMMU完成计数
      FMMUdone += FMMUsize;

      // 如果FMMU长度不为0，编程FMMU
      if (context->slavelist[slave].FMMU[FMMUc].LogLength)
      {
         // 设置FMMU属性
         context->slavelist[slave].FMMU[FMMUc].PhysStartBit = 0;
         context->slavelist[slave].FMMU[FMMUc].FMMUtype = 2;      // 输出类型
         context->slavelist[slave].FMMU[FMMUc].FMMUactive = 1;    // 激活FMMU

         /* 编程输出FMMU */
         ecx_FPWR(context->port, configadr, ECT_REG_FMMU0 + (sizeof(ec_fmmut) * FMMUc),
            sizeof(ec_fmmut), &(context->slavelist[slave].FMMU[FMMUc]), EC_TIMEOUTRET3);

         /* 设置标志为输出FMMU添加一个工作计数器，
            单个ESC只能贡献一次 */
         AddToOutputsWKC = 1;
      }

      // 设置输出指针
      if (!context->slavelist[slave].outputs)
      {
         if (group)
         {
            context->slavelist[slave].outputs =
               (uint8 *)(pIOmap) +
               etohl(context->slavelist[slave].FMMU[FMMUc].LogStart) -
               context->grouplist[group].logstartaddr;
         }
         else
         {
            context->slavelist[slave].outputs =
               (uint8 *)(pIOmap) +
               etohl(context->slavelist[slave].FMMU[FMMUc].LogStart);
         }

         // 设置输出起始位
         context->slavelist[slave].Ostartbit =
            context->slavelist[slave].FMMU[FMMUc].LogStartbit;

         EC_PRINT("    slave %d Outputs %p startbit %d\n",
            slave,
            context->slavelist[slave].outputs,
            context->slavelist[slave].Ostartbit);
      }

      // 移动到下一个FMMU
      FMMUc++;
   }

   // 更新未使用的FMMU索引
   context->slavelist[slave].FMMUunused = FMMUc;

   /* 如果标志为真，为输出添加一个工作计数器 */
   if (AddToOutputsWKC)
      context->grouplist[group].outputsWKC++;
}

/** 将指定组从站的所有PDO映射到IOmap，输出和输入按顺序排列。
 *
 * 这是主要的IO映射函数，用于将指定组中所有从站的PDO数据映射到
 * IOmap缓冲区。输出数据首先映射，然后是输入数据。该函数配置
 * 所有必要的FMMU，并计算工作计数器（WKC）值。
 *
 * @param[in]  context           = 包含从站列表的上下文结构体
 * @param[out] pIOmap            = 指向IOmap缓冲区的指针
 * @param[in]  group             = 组号，0表示所有组
 * @param[in]  forceByteAlignment = 是否强制字节对齐
 * @return IOmap的大小（字节），失败时返回0
 */
static int ecx_main_config_map_group(ecx_contextt *context, void *pIOmap, uint8 group, boolean forceByteAlignment)
{
   uint16 slave, configadr;        // 从站索引、配置地址
   uint8 BitPos;                   // 位位置（用于位级映射）
   uint32 LogAddr = 0;             // 逻辑地址（当前映射位置）
   uint32 oLogAddr = 0;            // 旧的逻辑地址（用于计算差值）
   uint32 diff;                    // 地址差值
   uint16 currentsegment = 0;      // 当前IO段索引
   uint32 segmentsize = 0;         // 当前IO段大小

   // 检查从站数量和组号是否有效
   if ((*(context->slavecount) > 0) && (group < context->maxgroup))
   {
      EC_PRINT("ec_config_map_group IOmap:%p group:%d\n", pIOmap, group);

      // 初始化逻辑地址为组的起始地址
      LogAddr = context->grouplist[group].logstartaddr;
      oLogAddr = LogAddr;
      BitPos = 0;

      // 初始化组信息
      context->grouplist[group].nsegments = 0;      // IO段数量
      context->grouplist[group].outputsWKC = 0;     // 输出工作计数器
      context->grouplist[group].inputsWKC = 0;      // 输入工作计数器

      /* 查找PDO映射并配置同步管理器 */
      ecx_config_find_mappings(context, group);

      /* 为所有从站创建输出映射并编程FMMU */
      for (slave = 1; slave <= *(context->slavecount); slave++)
      {
         configadr = context->slavelist[slave].configadr;

         // 检查从站是否属于指定组
         if (!group || (group == context->slavelist[slave].group))
         {
            /* 创建输出映射 */
            if (context->slavelist[slave].Obits)
            {
               // 为从站创建输出映射
               ecx_config_create_output_mappings (context, pIOmap, group, slave, &LogAddr, &BitPos);

               // 如果需要强制字节对齐
               if (forceByteAlignment)
               {
                  /* 如果输出小于8位，强制字节对齐 */
                  if (BitPos)
                  {
                     LogAddr++;     // 移动到下一个字节
                     BitPos = 0;    // 重置位位置
                  }
               }

               // 计算地址差值
               diff = LogAddr - oLogAddr;
               oLogAddr = LogAddr;

               // 检查是否需要创建新的IO段
               // 每个数据报的最大数据量为EC_MAXLRWDATA，减去第一个DC数据报的空间
               if ((segmentsize + diff) > (EC_MAXLRWDATA - EC_FIRSTDCDATAGRAM))
               {
                  // 保存当前段大小
                  context->grouplist[group].IOsegment[currentsegment] = segmentsize;

                  // 移动到下一个段
                  if (currentsegment < (EC_MAXIOSEGMENTS - 1))
                  {
                     currentsegment++;
                     segmentsize = diff;
                  }
               }
               else
               {
                  // 累加段大小
                  segmentsize += diff;
               }
            }
         }
      }

      // 处理最后的位对齐
      if (BitPos)
      {
         LogAddr++;
         oLogAddr = LogAddr;
         BitPos = 0;

         // 检查是否需要创建新的IO段
         if ((segmentsize + 1) > (EC_MAXLRWDATA - EC_FIRSTDCDATAGRAM))
         {
            context->grouplist[group].IOsegment[currentsegment] = segmentsize;
            if (currentsegment < (EC_MAXIOSEGMENTS - 1))
            {
               currentsegment++;
               segmentsize = 1;
            }
         }
         else
         {
            segmentsize += 1;
         }
      }

      // 保存输出映射信息
      context->grouplist[group].outputs = pIOmap;               // 设置组的输出映射指针，指向IO映射数组
      context->grouplist[group].Obytes = LogAddr - context->grouplist[group].logstartaddr;  // 计算输出字节数，从组起始地址到当前逻辑地址的差值
      context->grouplist[group].nsegments = currentsegment + 1;  // 设置组的段数量，currentsegment从0开始计数
      context->grouplist[group].Isegment = currentsegment;       // 设置输入段索引，指向当前最后一个段
      context->grouplist[group].Ioffset = (uint16)segmentsize;   // 设置输入偏移量，即当前段的大小

      // 如果是组0，保存到主站记录
      if (!group)
      {
         context->slavelist[0].outputs = pIOmap;
         context->slavelist[0].Obytes = LogAddr -
            context->grouplist[group].logstartaddr; /* 在主站记录中存储输出字节数 */
      }

      /* 为所有从站创建输入映射并编程FMMU */
      for (slave = 1; slave <= *(context->slavecount); slave++)
      {
         configadr = context->slavelist[slave].configadr;

         // 检查从站是否属于指定组
         if (!group || (group == context->slavelist[slave].group))
         {
            /* 创建输入映射 */
            if (context->slavelist[slave].Ibits)
            {
               // 为从站创建输入映射
               ecx_config_create_input_mappings(context, pIOmap, group, slave, &LogAddr, &BitPos);

               // 如果需要强制字节对齐
               if (forceByteAlignment)
               {
                  /* 如果输入小于8位，强制字节对齐 */
                  if (BitPos)
                  {
                     LogAddr++;
                     BitPos = 0;
                  }
               }

               // 计算地址差值
               diff = LogAddr - oLogAddr;
               oLogAddr = LogAddr;

               // 检查是否需要创建新的IO段
               if ((segmentsize + diff) > (EC_MAXLRWDATA - EC_FIRSTDCDATAGRAM))
               {
                  context->grouplist[group].IOsegment[currentsegment] = segmentsize;
                  if (currentsegment < (EC_MAXIOSEGMENTS - 1))
                  {
                     currentsegment++;
                     segmentsize = diff;
                  }
               }
               else
               {
                  segmentsize += diff;
               }
            }

            // 将EEPROM控制权交给PDI
            ecx_eeprom2pdi(context, slave);

            /* 用户可以覆盖自动状态更改 */
            if (context->manualstatechange == 0)
            {
               /* 请求从站进入SAFE_OP状态 */
               ecx_FPWRw(context->port,
                  configadr,
                  ECT_REG_ALCTL,
                  htoes(EC_STATE_SAFE_OP),
                  EC_TIMEOUTRET3); /* 设置safeop状态 */
            }

            // 统计块LRW从站数量
            if (context->slavelist[slave].blockLRW)
            {
               context->grouplist[group].blockLRW++;
            }

            // 累加E-bus电流
            context->grouplist[group].Ebuscurrent += context->slavelist[slave].Ebuscurrent;
         }
      }

      // 处理最后的位对齐
      if (BitPos)
      {
         LogAddr++;
         oLogAddr = LogAddr;
         BitPos = 0;

         // 检查是否需要创建新的IO段
         if ((segmentsize + 1) > (EC_MAXLRWDATA - EC_FIRSTDCDATAGRAM))
         {
            context->grouplist[group].IOsegment[currentsegment] = segmentsize;
            if (currentsegment < (EC_MAXIOSEGMENTS - 1))
            {
               currentsegment++;
               segmentsize = 1;
            }
         }
         else
         {
            segmentsize += 1;
         }
      }

      // 保存最后的IO段大小
      context->grouplist[group].IOsegment[currentsegment] = segmentsize;
      context->grouplist[group].nsegments = currentsegment + 1;

      // 设置输入映射指针（在输出数据之后）
      context->grouplist[group].inputs = (uint8 *)(pIOmap) + context->grouplist[group].Obytes;
      context->grouplist[group].Ibytes = LogAddr -
         context->grouplist[group].logstartaddr -
         context->grouplist[group].Obytes;

      // 如果是组0，保存到主站记录
      if (!group)
      {
         context->slavelist[0].inputs = (uint8 *)(pIOmap) + context->slavelist[0].Obytes;
         context->slavelist[0].Ibytes = LogAddr -
            context->grouplist[group].logstartaddr -
            context->slavelist[0].Obytes; /* 在主站记录中存储输入字节数 */
      }

      EC_PRINT("IOmapSize %d\n", LogAddr - context->grouplist[group].logstartaddr);

      // 返回IOmap总大小
      return (LogAddr - context->grouplist[group].logstartaddr);
   }

   return 0;
}

/** 将指定组从站的所有PDO映射到IOmap，输出和输入按顺序排列（传统SOEM方式）。
 *
 * @param[in]  context    = 上下文结构体
 * @param[out] pIOmap     = 指向IOmap的指针
 * @param[in]  group      = 要映射的组，0表示所有组
 * @return IOmap大小
 */
int ecx_config_map_group(ecx_contextt *context, void *pIOmap, uint8 group)
{
   return ecx_main_config_map_group(context, pIOmap, group, FALSE);
}

/** 将指定组从站的所有PDO映射到IOmap，输出和输入按顺序排列（传统SOEM方式），并强制字节对齐。
 *
 * @param[in]  context    = 上下文结构体
 * @param[out] pIOmap     = 指向IOmap的指针
 * @param[in]  group      = 要映射的组，0表示所有组
 * @return IOmap大小
 */
int ecx_config_map_group_aligned(ecx_contextt *context, void *pIOmap, uint8 group)
{
   return ecx_main_config_map_group(context, pIOmap, group, TRUE);
}

/** 将指定组从站的所有PDO映射到IOmap，输出和输入重叠。
 * 注意：使用LRW时必须为TI ESC使用此函数。
 *
 * @param[in]  context    = 上下文结构体
 * @param[out] pIOmap     = 指向IOmap的指针
 * @param[in]  group      = 要映射的组，0表示所有组
 * @return IOmap大小
 */
int ecx_config_overlap_map_group(ecx_contextt *context, void *pIOmap, uint8 group)
{
   uint16 slave, configadr;
   uint8 BitPos;
   uint32 mLogAddr = 0;
   uint32 siLogAddr = 0;
   uint32 soLogAddr = 0;
   uint32 tempLogAddr;
   uint32 diff;
   uint16 currentsegment = 0;
   uint32 segmentsize = 0;

   if ((*(context->slavecount) > 0) && (group < context->maxgroup))
   {
      EC_PRINT("ec_config_map_group IOmap:%p group:%d\n", pIOmap, group);
      mLogAddr = context->grouplist[group].logstartaddr;
      siLogAddr = mLogAddr;
      soLogAddr = mLogAddr;
      BitPos = 0;
      context->grouplist[group].nsegments = 0;
      context->grouplist[group].outputsWKC = 0;
      context->grouplist[group].inputsWKC = 0;

      /* Find mappings and program syncmanagers */
      ecx_config_find_mappings(context, group);

      /* do IO mapping of slave and program FMMUs */
      for (slave = 1; slave <= *(context->slavecount); slave++)
      {
         configadr = context->slavelist[slave].configadr;
         siLogAddr = soLogAddr = mLogAddr;

         if (!group || (group == context->slavelist[slave].group))
         {
            /* create output mapping */
            if (context->slavelist[slave].Obits)
            {

               ecx_config_create_output_mappings(context, pIOmap, group,
                  slave, &soLogAddr, &BitPos);
               if (BitPos)
               {
                  soLogAddr++;
                  BitPos = 0;
               }
            }

            /* create input mapping */
            if (context->slavelist[slave].Ibits)
            {
               ecx_config_create_input_mappings(context, pIOmap, group,
                  slave, &siLogAddr, &BitPos);
               if (BitPos)
               {
                  siLogAddr++;
                  BitPos = 0;
               }
            }

            tempLogAddr = (siLogAddr > soLogAddr) ?  siLogAddr : soLogAddr;
            diff = tempLogAddr - mLogAddr;
            mLogAddr = tempLogAddr;

            if ((segmentsize + diff) > (EC_MAXLRWDATA - EC_FIRSTDCDATAGRAM))
            {
               context->grouplist[group].IOsegment[currentsegment] = segmentsize;
               if (currentsegment < (EC_MAXIOSEGMENTS - 1))
               {
                  currentsegment++;
                  segmentsize = diff;
               }
            }
            else
            {
               segmentsize += diff;
            }

            ecx_eeprom2pdi(context, slave); /* set Eeprom control to PDI */
            /* User may override automatic state change */
            if (context->manualstatechange == 0)
            {
               /* request safe_op for slave */
               ecx_FPWRw(context->port,
                  configadr,
                  ECT_REG_ALCTL,
                  htoes(EC_STATE_SAFE_OP),
                  EC_TIMEOUTRET3);
            }
            if (context->slavelist[slave].blockLRW)
            {
               context->grouplist[group].blockLRW++;
            }
            context->grouplist[group].Ebuscurrent += context->slavelist[slave].Ebuscurrent;

         }
      }

      context->grouplist[group].IOsegment[currentsegment] = segmentsize;
      context->grouplist[group].nsegments = currentsegment + 1;
      context->grouplist[group].Isegment = 0;
      context->grouplist[group].Ioffset = 0;

      context->grouplist[group].Obytes = soLogAddr - context->grouplist[group].logstartaddr;
      context->grouplist[group].Ibytes = siLogAddr - context->grouplist[group].logstartaddr;
      context->grouplist[group].outputs = pIOmap;
      context->grouplist[group].inputs = (uint8 *)pIOmap + context->grouplist[group].Obytes;

      /* Move calculated inputs with OBytes offset*/
      for (slave = 1; slave <= *(context->slavecount); slave++)
      {
         if (!group || (group == context->slavelist[slave].group))
         {
            if(context->slavelist[slave].Ibits > 0)
            {
               context->slavelist[slave].inputs += context->grouplist[group].Obytes;
            }
         }
      }

      if (!group)
      {
         /* store output bytes in master record */
         context->slavelist[0].outputs = pIOmap;
         context->slavelist[0].Obytes = soLogAddr - context->grouplist[group].logstartaddr;
         context->slavelist[0].inputs = (uint8 *)pIOmap + context->slavelist[0].Obytes;
         context->slavelist[0].Ibytes = siLogAddr - context->grouplist[group].logstartaddr;
      }

      EC_PRINT("IOmapSize %d\n", context->grouplist[group].Obytes + context->grouplist[group].Ibytes);

      return (context->grouplist[group].Obytes + context->grouplist[group].Ibytes);
   }

   return 0;
}


/** Recover slave.
 *
 * @param[in] context = context struct
 * @param[in] slave   = slave to recover
 * @param[in] timeout = local timeout f.e. EC_TIMEOUTRET3
 * @return >0 if successful
 */
int ecx_recover_slave(ecx_contextt *context, uint16 slave, int timeout)
{
   int rval;
   int wkc;
   uint16 ADPh, configadr, readadr;

   rval = 0;
   configadr = context->slavelist[slave].configadr;
   ADPh = (uint16)(1 - slave);
   /* check if we found another slave than the requested */
   readadr = 0xfffe;
   wkc = ecx_APRD(context->port, ADPh, ECT_REG_STADR, sizeof(readadr), &readadr, timeout);
   /* correct slave found, finished */
   if(readadr == configadr)
   {
       return 1;
   }
   /* only try if no config address*/
   if( (wkc > 0) && (readadr == 0))
   {
      /* clear possible slaves at EC_TEMPNODE */
      ecx_FPWRw(context->port, EC_TEMPNODE, ECT_REG_STADR, htoes(0) , 0);
      /* set temporary node address of slave */
      if(ecx_APWRw(context->port, ADPh, ECT_REG_STADR, htoes(EC_TEMPNODE) , timeout) <= 0)
      {
         ecx_FPWRw(context->port, EC_TEMPNODE, ECT_REG_STADR, htoes(0) , 0);
         return 0; /* slave fails to respond */
      }

      context->slavelist[slave].configadr = EC_TEMPNODE; /* temporary config address */
      ecx_eeprom2master(context, slave); /* set Eeprom control to master */

      /* check if slave is the same as configured before */
      if ((ecx_FPRDw(context->port, EC_TEMPNODE, ECT_REG_ALIAS, timeout) ==
             htoes(context->slavelist[slave].aliasadr)) &&
          (ecx_readeeprom(context, slave, ECT_SII_ID, EC_TIMEOUTEEP) ==
             htoel(context->slavelist[slave].eep_id)) &&
          (ecx_readeeprom(context, slave, ECT_SII_MANUF, EC_TIMEOUTEEP) ==
             htoel(context->slavelist[slave].eep_man)) &&
          (ecx_readeeprom(context, slave, ECT_SII_REV, EC_TIMEOUTEEP) ==
             htoel(context->slavelist[slave].eep_rev)))
      {
         rval = ecx_FPWRw(context->port, EC_TEMPNODE, ECT_REG_STADR, htoes(configadr) , timeout);
         context->slavelist[slave].configadr = configadr;
      }
      else
      {
         /* slave is not the expected one, remove config address*/
         ecx_FPWRw(context->port, EC_TEMPNODE, ECT_REG_STADR, htoes(0) , timeout);
         context->slavelist[slave].configadr = configadr;
      }
   }

   return rval;
}

/** Reconfigure slave.
 *
 * @param[in] context = context struct
 * @param[in] slave   = slave to reconfigure
 * @param[in] timeout = local timeout f.e. EC_TIMEOUTRET3
 * @return Slave state
 */
int ecx_reconfig_slave(ecx_contextt *context, uint16 slave, int timeout)
{
   int state, nSM, FMMUc;
   uint16 configadr;

   configadr = context->slavelist[slave].configadr;
   if (ecx_FPWRw(context->port, configadr, ECT_REG_ALCTL, htoes(EC_STATE_INIT) , timeout) <= 0)
   {
      return 0;
   }
   state = 0;
   ecx_eeprom2pdi(context, slave); /* set Eeprom control to PDI */
   /* check state change init */
   state = ecx_statecheck(context, slave, EC_STATE_INIT, EC_TIMEOUTSTATE);
   if(state == EC_STATE_INIT)
   {
      /* program all enabled SM */
      for( nSM = 0 ; nSM < EC_MAXSM ; nSM++ )
      {
         if (context->slavelist[slave].SM[nSM].StartAddr)
         {
            ecx_FPWR(context->port, configadr, (uint16)(ECT_REG_SM0 + (nSM * sizeof(ec_smt))),
               sizeof(ec_smt), &context->slavelist[slave].SM[nSM], timeout);
         }
      }
      ecx_FPWRw(context->port, configadr, ECT_REG_ALCTL, htoes(EC_STATE_PRE_OP) , timeout);
      state = ecx_statecheck(context, slave, EC_STATE_PRE_OP, EC_TIMEOUTSTATE); /* check state change pre-op */
      if( state == EC_STATE_PRE_OP)
      {
         /* execute special slave configuration hook Pre-Op to Safe-OP */
         if(context->slavelist[slave].PO2SOconfig) /* only if registered */
         {
            context->slavelist[slave].PO2SOconfig(slave);
         }
         if (context->slavelist[slave].PO2SOconfigx) /* only if registered */
         {
            context->slavelist[slave].PO2SOconfigx(context, slave);
         }
         ecx_FPWRw(context->port, configadr, ECT_REG_ALCTL, htoes(EC_STATE_SAFE_OP) , timeout); /* set safeop status */
         state = ecx_statecheck(context, slave, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE); /* check state change safe-op */
         /* program configured FMMU */
         for( FMMUc = 0 ; FMMUc < context->slavelist[slave].FMMUunused ; FMMUc++ )
         {
            ecx_FPWR(context->port, configadr, (uint16)(ECT_REG_FMMU0 + (sizeof(ec_fmmut) * FMMUc)),
               sizeof(ec_fmmut), &context->slavelist[slave].FMMU[FMMUc], timeout);
         }
      }
   }

   return state;
}

#ifdef EC_VER1
/** Enumerate and init all slaves.
 *
 * @param[in] usetable     = TRUE when using configtable to init slaves, FALSE otherwise
 * @return Workcounter of slave discover datagram = number of slaves found
 * @see ecx_config_init
 */
int ec_config_init(uint8 usetable)
{
   return ecx_config_init(&ecx_context, usetable);
}

/** 将指定组从站的所有PDO映射到IOmap，输出和输入按顺序排列（传统SOEM方式）。
 *
 * @param[out] pIOmap     = 指向IOmap的指针
 * @param[in]  group      = 要映射的组，0表示所有组
 * @return IOmap大小
 * @see ecx_config_map_group
 */
int ec_config_map_group(void *pIOmap, uint8 group)
{
   return ecx_config_map_group(&ecx_context, pIOmap, group);
}

/** 将指定组从站的所有PDO映射到IOmap，输出和输入重叠。
 * 注意：使用LRW时必须为TI ESC使用此函数。
 *
 * @param[out] pIOmap     = 指向IOmap的指针
 * @param[in]  group      = 要映射的组，0表示所有组
 * @return IOmap大小
 * @see ecx_config_overlap_map_group
 */
int ec_config_overlap_map_group(void *pIOmap, uint8 group)
{
   return ecx_config_overlap_map_group(&ecx_context, pIOmap, group);
}

/** 将指定组从站的所有PDO映射到IOmap，输出和输入按顺序排列（传统SOEM方式），并强制字节对齐。
 *
 * @param[out] pIOmap     = 指向IOmap的指针
 * @param[in]  group      = 要映射的组，0表示所有组
 * @return IOmap大小
 * @see ecx_config_map_group
 */
int ec_config_map_group_aligned(void *pIOmap, uint8 group)
{
   return ecx_config_map_group_aligned(&ecx_context, pIOmap, group);
}

/** Map all PDOs from slaves to IOmap with Outputs/Inputs
 * in sequential order (legacy SOEM way).
 *
 * @param[out] pIOmap     = pointer to IOmap
 * @return IOmap size
 */
int ec_config_map(void *pIOmap)
{
   return ec_config_map_group(pIOmap, 0);
}

/** Map all PDOs from slaves to IOmap with Outputs/Inputs
* overlapping. NOTE: Must use this for TI ESC when using LRW.
*
* @param[out] pIOmap     = pointer to IOmap
* @return IOmap size
*/
int ec_config_overlap_map(void *pIOmap)
{
   return ec_config_overlap_map_group(pIOmap, 0);
}

/** Map all PDOs from slaves to IOmap with Outputs/Inputs
 * in sequential order (legacy SOEM way) and force byte alignment.
 *
 * @param[out] pIOmap     = pointer to IOmap
 * @return IOmap size
 */
int ec_config_map_aligned(void *pIOmap)
{
   return ec_config_map_group_aligned(pIOmap, 0);
}

/** Enumerate / map and init all slaves.
 *
 * @param[in] usetable    = TRUE when using configtable to init slaves, FALSE otherwise
 * @param[out] pIOmap     = pointer to IOmap
 * @return Workcounter of slave discover datagram = number of slaves found
 */
int ec_config(uint8 usetable, void *pIOmap)
{
   int wkc;
   wkc = ec_config_init(usetable);
   if (wkc)
   {
      ec_config_map(pIOmap);
   }
   return wkc;
}

/** Enumerate / map and init all slaves.
*
* @param[in] usetable    = TRUE when using configtable to init slaves, FALSE otherwise
* @param[out] pIOmap     = pointer to IOmap
* @return Workcounter of slave discover datagram = number of slaves found
*/
int ec_config_overlap(uint8 usetable, void *pIOmap)
{
   int wkc;
   wkc = ec_config_init(usetable);
   if (wkc)
   {
      ec_config_overlap_map(pIOmap);
   }
   return wkc;
}

/** Recover slave.
 *
 * @param[in] slave   = slave to recover
 * @param[in] timeout = local timeout f.e. EC_TIMEOUTRET3
 * @return >0 if successful
 * @see ecx_recover_slave
 */
int ec_recover_slave(uint16 slave, int timeout)
{
   return ecx_recover_slave(&ecx_context, slave, timeout);
}

/** Reconfigure slave.
 *
 * @param[in] slave   = slave to reconfigure
 * @param[in] timeout = local timeout f.e. EC_TIMEOUTRET3
 * @return Slave state
 * @see ecx_reconfig_slave
 */
int ec_reconfig_slave(uint16 slave, int timeout)
{
   return ecx_reconfig_slave(&ecx_context, slave, timeout);
}
#endif

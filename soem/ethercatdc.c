/*
 * Licensed under the GNU General Public License version 2 with exceptions. See
 * LICENSE file in the project root for full license information
 */

/** \file
 * \brief
 * Distributed Clock EtherCAT functions.
 *
 */
#include "oshw.h"
#include "osal.h"
#include "ethercattype.h"
#include "ethercatbase.h"
#include "ethercatmain.h"
#include "ethercatdc.h"

#define PORTM0 0x01
#define PORTM1 0x02
#define PORTM2 0x04
#define PORTM3 0x08

/** 1st sync pulse delay in ns here 100ms */
#define SyncDelay       ((int32)100000000)

/**
 * 设置从站DC以CyclTime间隔触发sync0，并带有CyclShift偏移量。
 *
 * @param[in]  context        = 上下文结构体
 * @param [in] slave            从站编号
 * @param [in] act              TRUE = 激活, FALSE = 停用
 * @param [in] CyclTime         循环时间，单位ns
 * @param [in] CyclShift        循环偏移，单位ns
 */
void ecx_dcsync0(ecx_contextt *context, uint16 slave, boolean act, uint32 CyclTime, int32 CyclShift)
{
   uint8 h, RA;
   uint16 slaveh;
   int64 t, t1;
   int32 tc;

   // 获取从站配置地址
   slaveh = context->slavelist[slave].configadr;
   // 初始化RA变量为0，用于停止循环操作
   RA = 0;

   // FPWR 0x981 1字节
   /* 停止循环操作，为下次触发做准备 */
   (void)ecx_FPWR(context->port, slaveh, ECT_REG_DCSYNCACT, sizeof(RA), &RA, EC_TIMEOUTRET);
   if (act)
   {
       // 如果激活，则设置为1+2=3，表示激活循环操作和sync0，sync1停用（激活需加4）
       RA = 1 + 2;    /* act cyclic operation and sync0, sync1 deactivated */
   }
   // 设置为0，写入访问以太网控制寄存器
   h = 0;
   // FPWR 0x980 1字节
   (void)ecx_FPWR(context->port, slaveh, ECT_REG_DCCUC, sizeof(h), &h, EC_TIMEOUTRET); /* write access to ethercat */
   // 初始化时间变量
   t1 = 0;
   // 读取从站的本地时间 0x910 8字节
   (void)ecx_FPRD(context->port, slaveh, ECT_REG_DCSYSTIME, sizeof(t1), &t1, EC_TIMEOUTRET); /* read local time of slave */
   // 将读取的时间转换为主机字节序
   t1 = etohll(t1);

   /* 计算首次触发时间，总是CyclTime的整数倍（向上舍入）
   加上偏移时间（可以是负数）
   这确保了从站之间最佳同步，具有相同CyclTime的
   从站将在同一时刻同步（您可以使用CyclShift来移动同步） */
   if (CyclTime > 0)
   {
       // 当循环时间大于0时，计算下一个周期的开始时间
       t = ((t1 + SyncDelay) / CyclTime) * CyclTime + CyclTime + CyclShift;
   }
   else
   {
      // 当循环时间等于0时，直接使用当前时间加上延迟和偏移
      t = t1 + SyncDelay + CyclShift;
      /* first trigger at T1 + CyclTime + SyncDelay + CyclShift in ns */
   }
   // 转换时间为网络字节序
   t = htoell(t);
   // 写入SYNC0启动时间 0x990 8字节
   (void)ecx_FPWR(context->port, slaveh, ECT_REG_DCSTART0, sizeof(t), &t, EC_TIMEOUTRET); /* SYNC0 start time */
   // 转换循环时间为网络字节序
   tc = htoel(CyclTime);
   // 写入SYNC0循环周期 0x9A0 4字节
   (void)ecx_FPWR(context->port, slaveh, ECT_REG_DCCYCLE0, sizeof(tc), &tc, EC_TIMEOUTRET); /* SYNC0 cycle time */
   // 重新激活循环操作 0x981 1字节
   (void)ecx_FPWR(context->port, slaveh, ECT_REG_DCSYNCACT, sizeof(RA), &RA, EC_TIMEOUTRET); /* activate cyclic operation */

    // 更新ec_slave状态
    context->slavelist[slave].DCactive = (uint8)act;
    context->slavelist[slave].DCshift = CyclShift;
    context->slavelist[slave].DCcycle = CyclTime;
}

/**
 * 设置从站DC以CyclTime间隔触发sync0和sync1，并带有CyclShift偏移量。
 *
 * @param[in]  context        = 上下文结构体
 * @param [in] slave            从站编号
 * @param [in] act              TRUE = 激活, FALSE = 停用
 * @param [in] CyclTime0        SYNC0循环时间，单位ns
 * @param [in] CyclTime1        SYNC1循环时间，单位ns。这是相对于SYNC0触发的增量时间。
                                如果CylcTime1 = 0则SYNC1与SYNC0同时触发。
 * @param [in] CyclShift        循环偏移，单位ns
 */
void ecx_dcsync01(ecx_contextt *context, uint16 slave, boolean act, uint32 CyclTime0, uint32 CyclTime1, int32 CyclShift)
{
   uint8 h, RA;
   uint16 slaveh;
   int64 t, t1;
   int32 tc;
   uint32 TrueCyclTime;

   /* Sync1可以作为Sync0的倍数使用，使用真实循环时间 */
   // 计算实际循环时间：如果CyclTime1是CyclTime0的倍数，则TrueCyclTime为倍数乘以CyclTime0
   TrueCyclTime = ((CyclTime1 / CyclTime0) + 1) * CyclTime0;

   // 获取从站配置地址
   slaveh = context->slavelist[slave].configadr;
   // 初始化RA变量为0，用于停止循环操作
   RA = 0;

   /* 停止循环操作，为下次触发做准备 */
   (void)ecx_FPWR(context->port, slaveh, ECT_REG_DCSYNCACT, sizeof(RA), &RA, EC_TIMEOUTRET); // FPWR 0x981 1字节
   if (act)
   {
      // 如果激活，则设置为1+2+4=7，表示激活循环操作和sync0+sync1
      RA = 1 + 2 + 4;    /* act cyclic operation and sync0 + sync1 */
   }
   // 设置为0，写入访问以太网控制寄存器
   h = 0;
   (void)ecx_FPWR(context->port, slaveh, ECT_REG_DCCUC, sizeof(h), &h, EC_TIMEOUTRET); /* write access to ethercat */ // FPWR 0x980 1字节
   // 初始化时间变量
   t1 = 0;
   // 读取从站的本地时间 0x910 8字节
   (void)ecx_FPRD(context->port, slaveh, ECT_REG_DCSYSTIME, sizeof(t1), &t1, EC_TIMEOUTRET); /* read local time of slave */
   // 将读取的时间转换为主机字节序
   t1 = etohll(t1);

   /* 计算首次触发时间，总是TrueCyclTime的整数倍（向上舍入）
   加上偏移时间（可以是负数）
   这确保了从站之间最佳同步，具有相同CyclTime的
   从站将在同一时刻同步（您可以使用CyclShift来移动同步） */
   if (CyclTime0 > 0)
   {
      // 当循环时间大于0时，计算下一个周期的开始时间
      t = ((t1 + SyncDelay) / TrueCyclTime) * TrueCyclTime + TrueCyclTime + CyclShift;
   }
   else
   {
      // 当循环时间等于0时，直接使用当前时间加上延迟和偏移
      t = t1 + SyncDelay + CyclShift;
      /* first trigger at T1 + CyclTime + SyncDelay + CyclShift in ns */
   }
   // 转换时间为网络字节序
   t = htoell(t);
   // 写入SYNC0启动时间 0x990 8字节
   (void)ecx_FPWR(context->port, slaveh, ECT_REG_DCSTART0, sizeof(t), &t, EC_TIMEOUTRET); /* SYNC0 start time */
   // 转换SYNC0循环时间为网络字节序
   tc = htoel(CyclTime0);
   // 写入SYNC0循环周期 0x9A0 4字节
   (void)ecx_FPWR(context->port, slaveh, ECT_REG_DCCYCLE0, sizeof(tc), &tc, EC_TIMEOUTRET); /* SYNC0 cycle time */
   // 转换SYNC1循环时间为网络字节序
   tc = htoel(CyclTime1);
   // 写入SYNC1循环周期 0x9A4 4字节
   (void)ecx_FPWR(context->port, slaveh, ECT_REG_DCCYCLE1, sizeof(tc), &tc, EC_TIMEOUTRET); /* SYNC1 cycle time */
   // 重新激活循环操作 0x981 1字节
   (void)ecx_FPWR(context->port, slaveh, ECT_REG_DCSYNCACT, sizeof(RA), &RA, EC_TIMEOUTRET); /* activate cyclic operation */

    // 更新ec_slave状态
    context->slavelist[slave].DCactive = (uint8)act;
    context->slavelist[slave].DCshift = CyclShift;
    context->slavelist[slave].DCcycle = CyclTime0;
}

// 根据从站编号和端口号获取端口时间
/* latched port time of slave */
static int32 ecx_porttime(ecx_contextt *context, uint16 slave, uint8 port)
{
   int32 ts;
   // 根据端口号选择对应的端口时间寄存器
   switch (port)
   {
      case 0:
         // 读取端口A的时间戳
         ts = context->slavelist[slave].DCrtA;
         break;
      case 1:
         // 读取端口B的时间戳
         ts = context->slavelist[slave].DCrtB;
         break;
      case 2:
         // 读取端口C的时间戳
         ts = context->slavelist[slave].DCrtC;
         break;
      case 3:
         // 读取端口D的时间戳
         ts = context->slavelist[slave].DCrtD;
         break;
      default:
         // 默认返回0
         ts = 0;
         break;
   }
   // 返回获取到的时间戳值
   return ts;
}

// 根据从站编号和端口号获取当前从站的前一个活跃端口
/* calculate previous active port of a slave */
static uint8 ecx_prevport(ecx_contextt *context, uint16 slave, uint8 port)
{
   uint8 pport = port;
   // 获取从站的活跃端口信息
   uint8 aport = context->slavelist[slave].activeports;
   // 根据当前端口判断前一个活跃端口
   switch(port)
   {
      case 0:
         // 当前端口为0时，优先检查端口2是否活跃
         if(aport & PORTM2)
            pport = 2;
         // 否则检查端口1是否活跃
         else if (aport & PORTM1)
            pport = 1;
         // 最后检查端口3是否活跃
         else if (aport & PORTM3)
            pport = 3;
         break;
      case 1:
         // 当前端口为1时，优先检查端口3是否活跃
         if(aport & PORTM3)
            pport = 3;
         // 否则检查端口0是否活跃
         else if (aport & PORTM0)
            pport = 0;
         // 最后检查端口2是否活跃
         else if (aport & PORTM2)
            pport = 2;
         break;
      case 2:
         // 当前端口为2时，优先检查端口1是否活跃
         if(aport & PORTM1)
            pport = 1;
         // 否则检查端口3是否活跃
         else if (aport & PORTM3)
            pport = 3;
         // 最后检查端口0是否活跃
         else if (aport & PORTM0)
            pport = 0;
         break;
      case 3:
         // 当前端口为3时，优先检查端口0是否活跃
         if(aport & PORTM0)
            pport = 0;
         // 否则检查端口2是否活跃
         else if (aport & PORTM2)
            pport = 2;
         // 最后检查端口1是否活跃
         else if (aport & PORTM1)
            pport = 1;
         break;
   }
   // 返回找到的前一个活跃端口
   return pport;
}

// 根据父节点编号获取未消费的端口并消费该端口
/* search unconsumed ports in parent, consume and return first open port */
static uint8 ecx_parentport(ecx_contextt *context, uint16 parent)
{
   uint8 parentport = 0;
   uint8 b;
   /* search order is important, here 3 - 1 - 2 - 0 */
   // 搜索顺序很重要，这里先搜索3，再搜索1，再搜索2，最后搜索0。因为0一般是入口端口
   // 获取父节点已消费的端口信息
   b = context->slavelist[parent].consumedports;
   // 按照特定顺序搜索未消费的端口(优先级：3 - 1 - 2 - 0)
   if (b & PORTM3)
   {
      // 如果端口3已被消费，则将其标记为未消费并选择该端口
      parentport = 3;
      b &= (uint8)~PORTM3;
   }
   else if (b & PORTM1)
   {
      // 如果端口1已被消费，则将其标记为未消费并选择该端口
      parentport = 1;
      b &= (uint8)~PORTM1;
   }
   else if (b & PORTM2)
   {
      // 如果端口2已被消费，则将其标记为未消费并选择该端口
      parentport = 2;
      b &= (uint8)~PORTM2;
   }
   else if (b & PORTM0)
   {
      // 如果端口0已被消费，则将其标记为未消费并选择该端口
      parentport = 0;
      b &= (uint8)~PORTM0;
   }
   // 更新父节点的已消费端口状态
   context->slavelist[parent].consumedports = b;
   // 返回找到的端口号
   return parentport;
}

/**
 * 配置分布式时钟（DC）：定位具有分布式时钟功能的从站设备，并测量信号传播延迟。
 * 此函数会遍历所有从站，检测哪些支持分布式时钟功能，并计算它们之间的信号传播延迟，
 * 以便实现精确的时间同步。
 *
 * @param[in]  context        = EtherCAT主站上下文结构指针
 * @return boolean             = 如果找到支持DC功能的从站则返回TRUE，否则返回FALSE
 */
boolean ecx_configdc(ecx_contextt *context)
{
   uint16 i, slaveh, parent, child;
   uint16 parenthold = 0;  /* 持有分支根父节点的临时变量 */
   uint16 prevDCslave = 0; /* 记录前一个DC从站的索引 */

   // dt3: 父DC从站的连接端口与前一个端口的时间差
   // dt1: 当前从站的入口端口与前一个端口的时间差，即最大端口时间差，即后续网络的全部延迟
   // dt2: 当前从站不是父节点的第一个子节点时，父从站入口端口与连接端口的时间差
   int32 ht, dt1, dt2, dt3;
   int64 hrt;
   uint8 entryport;  /* 从站的入口端口 */
   int8 nlist;  /* 活跃端口数量 */
   int8 plist[4];  /* 端口号列表 */
   int32 tlist[4];  /* 时间戳列表 */
   ec_timet mastertime;  /* 主站当前时间 */
   uint64 mastertime64;  /* 转换为EtherCAT时间格式的主站时间 */

   /* 初始化全局DC状态标志 */
   context->slavelist[0].hasdc = FALSE;  /* 主站没有DC功能 */
   context->grouplist[0].hasdc = FALSE;  /* 第一组没有DC功能 */
   ht = 0; // 0x900 寄存器值

   // BWR 0x0900 4字节
   ecx_BWR(context->port, 0, ECT_REG_DCTIME0, sizeof(ht), &ht, EC_TIMEOUTRET);

   /* 获取当前系统时间并转换为EtherCAT时间格式 */
   mastertime = osal_current_time();
   /* EtherCAT使用2000年1月1日作为纪元开始，而不是1970年1月1日 */
   mastertime.sec -= 946684800UL;
   mastertime64 = (((uint64)mastertime.sec * 1000000) + (uint64)mastertime.usec) * 1000;

   /* 遍历所有从站设备 */
   for (i = 1; i <= *(context->slavecount); i++)
   {
      /* 初始化从站的已消耗端口列表为活跃端口列表 */
      context->slavelist[i].consumedports = context->slavelist[i].activeports;

      /* 检查当前从站是否支持DC功能 */
      if (context->slavelist[i].hasdc)
      {
         /* 如果当前从站是网络中按顺序的第一个DC从站 */
         // slavelist[0]表示主站信息，这里条件成立表示还未设置主站支持DC标志
         if (!context->slavelist[0].hasdc)
         {
            context->slavelist[0].hasdc = TRUE;  /* 设置主站DC标志 */
            context->slavelist[0].DCnext = i;     /* 连接到第一个DC从站 */
            context->slavelist[i].DCprevious = 0; /* 第一个DC从站的前一个为0 */
            context->grouplist[context->slavelist[i].group].hasdc = TRUE;  /* 设置组DC标志 */
            context->grouplist[context->slavelist[i].group].DCnext = i;    /* 组连接到第一个DC从站 */
         }
         else  /* 如果不是第一个DC从站，则链接到链表中 */
         {
            context->slavelist[prevDCslave].DCnext = i;  /* 前一个DC从站指向当前 */
            context->slavelist[i].DCprevious = prevDCslave; /* 当前从站指向前一个 */
         }
         /* 这个分支有DC从站，所以清除parenthold */
         parenthold = 0;
         prevDCslave = i;  /* 更新前一个DC从站为当前 */

         /* 获取从站配置地址 */
         slaveh = context->slavelist[i].configadr;

         // 读端口0接收时间FPRD 0x0900 4字节
         (void)ecx_FPRD(context->port, slaveh, ECT_REG_DCTIME0, sizeof(ht), &ht, EC_TIMEOUTRET);
         context->slavelist[i].DCrtA = etohl(ht);

         /* 读取64位系统偏移量寄存器  0x0918*/
         (void)ecx_FPRD(context->port, slaveh, ECT_REG_DCSOF, sizeof(hrt), &hrt, EC_TIMEOUTRET);
         /* 使用它作为偏移量，使本地时间围绕0+mastertime设置 */
         hrt = htoell(-etohll(hrt) + mastertime64);
         /* 将偏移量写入偏移寄存器 */
         // FPWR 0x0920 8字节
         (void)ecx_FPWR(context->port, slaveh, ECT_REG_DCSYSOFFSET, sizeof(hrt), &hrt, EC_TIMEOUTRET);

         /* 读取其他端口的接收时间 */
         (void)ecx_FPRD(context->port, slaveh, ECT_REG_DCTIME1, sizeof(ht), &ht, EC_TIMEOUTRET);
         context->slavelist[i].DCrtB = etohl(ht);
         (void)ecx_FPRD(context->port, slaveh, ECT_REG_DCTIME2, sizeof(ht), &ht, EC_TIMEOUTRET);
         context->slavelist[i].DCrtC = etohl(ht);
         (void)ecx_FPRD(context->port, slaveh, ECT_REG_DCTIME3, sizeof(ht), &ht, EC_TIMEOUTRET);
         context->slavelist[i].DCrtD = etohl(ht);

         /* 创建活跃端口及其时间戳的列表 */
         nlist = 0;
         if (context->slavelist[i].activeports & PORTM0)
         {
            plist[nlist] = 0;  /* 端口0 */
            tlist[nlist] = context->slavelist[i].DCrtA;  /* 对应时间戳 */
            nlist++;
         }
         if (context->slavelist[i].activeports & PORTM3)
         {
            plist[nlist] = 3;  /* 端口3 */
            tlist[nlist] = context->slavelist[i].DCrtD;  /* 对应时间戳 */
            nlist++;
         }
         if (context->slavelist[i].activeports & PORTM1)
         {
            plist[nlist] = 1;  /* 端口1 */
            tlist[nlist] = context->slavelist[i].DCrtB;  /* 对应时间戳 */
            nlist++;
         }
         if (context->slavelist[i].activeports & PORTM2)
         {
            plist[nlist] = 2;  /* 端口2 */
            tlist[nlist] = context->slavelist[i].DCrtC;  /* 对应时间戳 */
            nlist++;
         }

         /* 寻找入口端口：入口端口是时间戳最小的端口（最先接收到信号的端口） */
         // TODO 端口时间为32位，这里没有考虑溢出情况
         entryport = 0;
         // 如果只有一个端口，则数组中保存的端口就是入口端口
         if((nlist > 1) && (tlist[1] < tlist[entryport]))
         {
            entryport = 1;
         }
         if((nlist > 2) && (tlist[2] < tlist[entryport]))
         {
            entryport = 2;
         }
         if((nlist > 3) && (tlist[3] < tlist[entryport]))
         {
            entryport = 3;
         }
         entryport = plist[entryport];
         context->slavelist[i].entryport = entryport; /* 设置从站的入口端口 */
         // NOTE 这个函数允许入口端口不为0
         /* 从活跃端口中消耗入口端口（标记为已使用） */
         context->slavelist[i].consumedports &= (uint8)~(1 << entryport);

         /* 查找当前从站的DC父节点 */
         // TODO 这里获取的DC父从站至少有两个打开的端口
         parent = i;
         do
         {
            child = parent;
            parent = context->slavelist[parent].parent;
         }
         while (!((parent == 0) || (context->slavelist[parent].hasdc)));

         /* 只有当父从站不是第一个时才计算传播延迟 */
         if (parent > 0)
         {
            /* 查找父节点上此从站所连接的端口 */
            context->slavelist[i].parentport = ecx_parentport(context, parent);
            /* 如果DC父节点只有一个打开的端口，则使用父节点的入口端口 */
            // TODO 这个情况不可能发生。因为DC父从站至少有两个打开的端口才能作为父从站
            if (context->slavelist[parent].topology == 1)
            {
               context->slavelist[i].parentport = context->slavelist[parent].entryport;
            }

            dt1 = 0;
            dt2 = 0;
            // 计算父DC从站的连接端口与前一个端口的时间差。
            // 如果父从站只有一个打开的端口，ecx_prevport返回原来的端口号，dt3=0.
            /* 注意：端口顺序是0 - 3 - 1 - 2 */
            dt3 = ecx_porttime(context, parent, context->slavelist[i].parentport) -
                  ecx_porttime(context, parent,
                    ecx_prevport(context, parent, context->slavelist[i].parentport));

            /* 当前从站有子节点，需要减去那些子节点的延迟 */
            if (context->slavelist[i].topology > 1) {
              // 计算当前从站的入口端口与前一个端口的时间差。即最大端口时间差
              // 如果从站只有一个打开的端口，ecx_prevport返回原来的端口号，dt1=0.
               dt1 = ecx_porttime(context, i,
                        ecx_prevport(context, i, context->slavelist[i].entryport)) -
                     ecx_porttime(context, i, context->slavelist[i].entryport);
            }
            /* 只对正差值感兴趣 */
            if (dt1 > dt3) dt1 = -dt1;

            /* 当前从站不是父节点的第一个子节点 */
            /* 需要加上前一个子节点的延迟 */
            if ((child - parent) > 1) {
               // 计算父DC从站的连接端口与入口端口的时间差。
               dt2 = ecx_porttime(context, parent,
                        ecx_prevport(context, parent, context->slavelist[i].parentport)) -
                     ecx_porttime(context, parent, context->slavelist[parent].entryport);
            }
            // 确保dt2为正
            // TODO 为什么，通常情况的对溢出的考虑？
            if (dt2 < 0) dt2 = -dt2;

            // 当前从站传播延迟=(父DC从站连接端口与前一个端口时间差-当前从站最大端口时间差) / 2 + 当前从站不是父节点第一个子节点时父从站入口端口与连接端口时间差 + 父从站传播延迟
            /* 假设：发送延迟等于返回延迟 */
            context->slavelist[i].pdelay = ((dt3 - dt1) / 2) + dt2 +
               context->slavelist[parent].pdelay;

            /* 将计算出的延迟转换为网络字节序并写入从站 */
            ht = htoel(context->slavelist[i].pdelay);
            /* 写入传播延迟 0x928 */
            (void)ecx_FPWR(context->port, slaveh, ECT_REG_DCSYSDELAY, sizeof(ht), &ht, EC_TIMEOUTRET);
         }
      }
      else  /* 处理不支持DC功能的从站 */
      {
         /* 清零DC相关时间记录 */
         context->slavelist[i].DCrtA = 0;
         context->slavelist[i].DCrtB = 0;
         context->slavelist[i].DCrtC = 0;
         context->slavelist[i].DCrtD = 0;

         parent = context->slavelist[i].parent;
         /* 如果父从站是分叉或者交叉从站，则保持父节点到 parenthold */
         if ( (parent > 0) && (context->slavelist[parent].topology > 2))
            parenthold = parent;

         /* 如果父从站是分叉或者交叉从站，且当前从站只有一个端口，则消耗根父节点的一个端口 */
         if ( parenthold && (context->slavelist[i].topology == 1))
         {
            ecx_parentport(context, parenthold);  /* 消耗根父节点的一个端口 */
            parenthold = 0;
         }
      }
   }

   /* 返回是否有DC从站的标志 */
   return context->slavelist[0].hasdc;
}

#ifdef EC_VER1
void ec_dcsync0(uint16 slave, boolean act, uint32 CyclTime, int32 CyclShift)
{
   ecx_dcsync0(&ecx_context, slave, act, CyclTime, CyclShift);
}

void ec_dcsync01(uint16 slave, boolean act, uint32 CyclTime0, uint32 CyclTime1, int32 CyclShift)
{
   ecx_dcsync01(&ecx_context, slave, act, CyclTime0, CyclTime1, CyclShift);
}

boolean ec_configdc(void)
{
   return ecx_configdc(&ecx_context);
}
#endif

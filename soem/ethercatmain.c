/*
 * Licensed under the GNU General Public License version 2 with exceptions. See
 * LICENSE file in the project root for full license information
 */

/**
 * \file
 * \brief EtherCAT主站核心功能模块
 *
 * 本文件包含SOEM（Simple Open EtherCAT Master）库的主要功能实现：
 *
 * - 初始化：网络接口初始化、从站发现和配置
 * - 状态管理：读取和设置从站AL状态（INIT, PRE_OP, SAFE_OP, OPERATIONAL）
 * - 邮箱通信：邮箱发送/接收原语，用于CoE、EoE、FoE等协议
 * - EEPROM操作：SII（从站信息接口）读取和写入
 * - 过程数据交换：周期性输入/输出数据的发送和接收
 *
 * 定义了全局从站数组ec_slave[]，所有从站信息都存储在此结构中。
 * 这是用户与从站交互的主要接口。
 *
 * \note 本文件支持两种API风格：
 *       - ecx_* 函数：使用显式上下文（ecx_contextt），支持多实例
 *       - ec_* 函数：使用全局上下文，兼容旧版本（EC_VER1）
 */

#include <stdio.h>
#include <string.h>
#include "osal.h"
#include "oshw.h"
#include "ethercat.h"


/** EEPROM就绪循环的延迟时间（微秒） */
#define EC_LOCALDELAY  200

/** EtherCAT EEPROM通信记录结构体
 *
 * 用于通过ESC（EtherCAT从站控制器）寄存器访问从站EEPROM。
 * 包含命令、地址和数据字段。
 */
PACKED_BEGIN
typedef struct PACKED
{
   uint16 comm;    /**< EEPROM命令：NOP(0x0000), READ(0x0100), WRITE(0x0201), RELOAD(0x0300) */
   uint16 addr;    /**< EEPROM字地址 */
   uint16 d2;      /**< 地址扩展/数据字段2 */
} ec_eepromt;
PACKED_END

/** 邮箱错误响应结构体
 *
 * 当从站无法处理邮箱请求时返回此错误响应。
 * 邮箱类型字段为0表示这是错误响应。
 */
PACKED_BEGIN
typedef struct PACKED
{
   ec_mbxheadert   MbxHeader;   /**< 邮箱头部 */
   uint16          Type;        /**< 错误类型 */
   uint16          Detail;      /**< 详细错误码 */
} ec_mbxerrort;
PACKED_END

/** CANopen紧急消息结构体
 *
 * 用于报告从站设备内部的错误和异常情况。
 * 遵循CANopen DS301规范的紧急对象格式。
 */
PACKED_BEGIN
typedef struct PACKED
{
   ec_mbxheadert   MbxHeader;    /**< 邮箱头部 */
   uint16          CANOpen;      /**< CANopen服务类型（高4位）和对象字典索引 */
   uint16          ErrorCode;    /**< 紧急错误码 */
   uint8           ErrorReg;     /**< 错误寄存器 */
   uint8           bData;        /**< 制造商特定错误数据 */
   uint16          w1,w2;        /**< 附加数据字 */
} ec_emcyt;
PACKED_END

#ifdef EC_VER1
/** 主从站数据数组。
 *  网络上发现的每个从站都有自己的记录。
 *  ec_slave[0]保留给主站使用。
 *  结构由配置函数ec_config()填充。
 */
ec_slavet               ec_slave[EC_MAXSLAVE];

/** 网络上发现的从站数量 */
int                     ec_slavecount;

/** 从站分组结构 */
ec_groupt               ec_group[EC_MAXGROUP];

/** EEPROM读取功能的缓存 */
static uint8            ec_esibuf[EC_MAXEEPBUF];

/** 已填充缓存字节的位图 */
static uint32           ec_esimap[EC_MAXEEPBITMAP];

/** 当前EEPROM缓存对应的从站 */
static ec_eringt        ec_elist;

/** 索引栈，用于分段过程数据传输 */
static ec_idxstackT     ec_idxstack;

/** 同步管理器通信类型结构，存储单个从站的数据 */
static ec_SMcommtypet   ec_SMcommtype[EC_MAX_MAPT];

/** PDO分配结构，存储单个从站的数据 */
static ec_PDOassignt    ec_PDOassign[EC_MAX_MAPT];

/** PDO描述结构，存储单个从站的数据 */
static ec_PDOdesct      ec_PDOdesc[EC_MAX_MAPT];

/** EEPROM SM数据缓冲区 */
static ec_eepromSMt     ec_SM;

/** EEPROM FMMU数据缓冲区 */
static ec_eepromFMMUt   ec_FMMU;

/** 全局变量，错误栈中有错误时为TRUE */
boolean                 EcatError = FALSE;

/** 参考时钟上一次时间（纳秒） */
int64                   ec_DCtime;

/** 主站端口数据结构 */
ecx_portt               ecx_port;

/** 冗余端口数据结构 */
ecx_redportt            ecx_redport;

/** 全局EtherCAT上下文结构
 *
 * 初始化为使用上述定义的全局变量。
 * 这是ec_*函数使用的默认上下文。
 */
ecx_contextt  ecx_context = {
    &ecx_port,          /* .port          = 端口结构指针 */
    &ec_slave[0],       /* .slavelist     = 从站列表 */
    &ec_slavecount,     /* .slavecount    = 从站数量指针 */
    EC_MAXSLAVE,        /* .maxslave      = 最大从站数 */
    &ec_group[0],       /* .grouplist     = 组列表 */
    EC_MAXGROUP,        /* .maxgroup      = 最大组数 */
    &ec_esibuf[0],      /* .esibuf        = EEPROM缓存 */
    &ec_esimap[0],      /* .esimap        = EEPROM位图 */
    0,                  /* .esislave      = 当前EEPROM从站 */
    &ec_elist,          /* .elist         = 错误列表 */
    &ec_idxstack,       /* .idxstack      = 索引栈 */
    &EcatError,         /* .ecaterror     = 错误标志 */
    &ec_DCtime,         /* .DCtime        = 分布时钟时间 */
    &ec_SMcommtype[0],  /* .SMcommtype    = SM通信类型 */
    &ec_PDOassign[0],   /* .PDOassign     = PDO分配 */
    &ec_PDOdesc[0],     /* .PDOdesc       = PDO描述 */
    &ec_SM,             /* .eepSM         = EEPROM SM缓冲区 */
    &ec_FMMU,           /* .eepFMMU       = EEPROM FMMU缓冲区 */
    NULL,               /* .FOEhook()     = FoE钩子函数 */
    NULL,               /* .EOEhook()     = EoE钩子函数 */
    0,                  /* .manualstatechange = 手动状态改变标志 */
    NULL,               /* .userdata      = 用户数据指针 */
};
#endif

/** 创建可用网络适配器列表。
 *
 * 查询系统上所有可用的网络适配器，返回链表头指针。
 * 用于选择用于EtherCAT通信的网络接口。
 *
 * @return 可用网络适配器列表的第一个元素
 */
ec_adaptert * ec_find_adapters (void)
{
   ec_adaptert * ret_adapter;

   ret_adapter = oshw_find_adapters ();

   return ret_adapter;
}

/** 释放动态分配的网络适配器列表。
 *
 * 释放由ec_find_adapters()分配的内存。
 *
 * @param[in] adapter = 包含适配器名称、描述和指向下一个元素的指针的结构体
 */
void ec_free_adapters (ec_adaptert * adapter)
{
   oshw_free_adapters (adapter);
}

/** 将错误压入错误列表。
 *
 * 将错误信息添加到环形错误缓冲区中。
 * 如果缓冲区已满，最旧的错误将被覆盖。
 *
 * @param[in] context = 上下文结构体
 * @param[in] Ec      = 描述错误的错误结构体指针
 */
void ecx_pusherror(ecx_contextt *context, const ec_errort *Ec)
{
   /* 将错误结构体复制到错误列表头部位置 */
   context->elist->Error[context->elist->head] = *Ec;
   context->elist->Error[context->elist->head].Signal = TRUE;

   /* 更新头部指针（环形缓冲区） */
   context->elist->head++;
   if (context->elist->head > EC_MAXELIST)
   {
      context->elist->head = 0;
   }

   /* 如果缓冲区已满，移动尾部指针 */
   if (context->elist->head == context->elist->tail)
   {
      context->elist->tail++;
   }
   if (context->elist->tail > EC_MAXELIST)
   {
      context->elist->tail = 0;
   }

   /* 设置全局错误标志 */
   *(context->ecaterror) = TRUE;
}

/** 从错误列表中弹出一个错误。
 *
 * 从错误列表的尾部取出一个错误。错误列表是一个环形缓冲区。
 *
 * @param[in]  context = 上下文结构体
 * @param[out] Ec      = 描述错误的结构体
 * @return TRUE: 成功弹出一个错误；FALSE: 错误列表为空
 */
boolean ecx_poperror(ecx_contextt *context, ec_errort *Ec)
{
   /* 检查错误列表是否非空 */
   boolean notEmpty = (context->elist->head != context->elist->tail);

   /* 复制错误结构体 */
   *Ec = context->elist->Error[context->elist->tail];
   context->elist->Error[context->elist->tail].Signal = FALSE;

   if (notEmpty)
   {
      /* 更新尾部指针（环形缓冲区） */
      context->elist->tail++;
      if (context->elist->tail > EC_MAXELIST)
      {
         context->elist->tail = 0;
      }
   }
   else
   {
      /* 列表为空，清除错误标志 */
      *(context->ecaterror) = FALSE;
   }
   return notEmpty;
}

/** 检查错误列表是否有条目。
 *
 * @param[in] context = 上下文结构体
 * @return TRUE: 错误列表有条目；FALSE: 错误列表为空
 */
boolean ecx_iserror(ecx_contextt *context)
{
   return (context->elist->head != context->elist->tail);
}

/** 报告数据包错误。
 *
 * 当EtherCAT数据包通信失败时调用此函数记录错误。
 *
 * @param[in]  context   = 上下文结构体
 * @param[in]  Slave     = 从站号
 * @param[in]  Index     = 产生错误的索引
 * @param[in]  SubIdx    = 产生错误的子索引
 * @param[in]  ErrorCode = 错误码
 */
void ecx_packeterror(ecx_contextt *context, uint16 Slave, uint16 Index, uint8 SubIdx, uint16 ErrorCode)
{
   ec_errort Ec;

   memset(&Ec, 0, sizeof(Ec));
   Ec.Time = osal_current_time();
   Ec.Slave = Slave;
   Ec.Index = Index;
   Ec.SubIdx = SubIdx;
   *(context->ecaterror) = TRUE;
   Ec.Etype = EC_ERR_TYPE_PACKET_ERROR;
   Ec.ErrorCode = ErrorCode;
   ecx_pusherror(context, &Ec);
}

/** 报告邮箱错误。
 *
 * 当从站返回邮箱错误响应时调用此函数记录错误。
 *
 * @param[in]  context = 上下文结构体
 * @param[in]  Slave   = 从站号
 * @param[in]  Detail  = 详细错误码（遵循EtherCAT规范）
 */
static void ecx_mbxerror(ecx_contextt *context, uint16 Slave,uint16 Detail)
{
   ec_errort Ec;

   memset(&Ec, 0, sizeof(Ec));
   Ec.Time = osal_current_time();
   Ec.Slave = Slave;
   Ec.Index = 0;
   Ec.SubIdx = 0;
   Ec.Etype = EC_ERR_TYPE_MBX_ERROR;
   Ec.ErrorCode = Detail;
   ecx_pusherror(context, &Ec);
}

/** 报告邮箱紧急错误。
 *
 * 当从站发送CANopen紧急消息时调用此函数记录错误。
 * 紧急消息用于报告设备内部的错误和异常情况。
 *
 * @param[in]  context    = 上下文结构体
 * @param[in]  Slave      = 从站号
 * @param[in]  ErrorCode  = 紧急错误码（遵循EtherCAT/CANopen规范）
 * @param[in]  ErrorReg   = 错误寄存器
 * @param[in]  b1         = 制造商特定错误数据
 * @param[in]  w1        = 附加数据字1
 * @param[in]  w2        = 附加数据字2
 */
static void ecx_mbxemergencyerror(ecx_contextt *context, uint16 Slave,uint16 ErrorCode,uint16 ErrorReg,
    uint8 b1, uint16 w1, uint16 w2)
{
   ec_errort Ec;

   memset(&Ec, 0, sizeof(Ec));
   Ec.Time = osal_current_time();
   Ec.Slave = Slave;
   Ec.Index = 0;
   Ec.SubIdx = 0;
   Ec.Etype = EC_ERR_TYPE_EMERGENCY;
   Ec.ErrorCode = ErrorCode;
   Ec.ErrorReg = (uint8)ErrorReg;
   Ec.b1 = b1;
   Ec.w1 = w1;
   Ec.w2 = w2;
   ecx_pusherror(context, &Ec);
}

/** 初始化库（单网卡模式）。
 *
 * 初始化SOEM库，使用单个网络接口进行EtherCAT通信。
 * 这是推荐的初始化方式，适用于大多数应用场景。
 *
 * @param[in]  context = 上下文结构体
 * @param[in]  ifname  = 网络接口名称，例如 "eth0"
 * @return >0: 成功；<=0: 失败
 */
int ecx_init(ecx_contextt *context, const char * ifname)
{
   return ecx_setupnic(context->port, ifname, FALSE);
}

/** 初始化库（冗余网卡模式）。
 *
 * 初始化SOEM库，使用两个网络接口进行冗余EtherCAT通信。
 * 冗余模式可以在一个网络接口故障时保持通信。
 *
 * @param[in]  context  = 上下文结构体
 * @param[in]  redport  = 冗余端口数据结构指针
 * @param[in]  ifname   = 主网卡名称，例如 "eth0"
 * @param[in]  if2name  = 备用网卡名称，例如 "eth1"
 * @return >0: 成功；<=0: 失败
 */
int ecx_init_redundant(ecx_contextt *context, ecx_redportt *redport, const char *ifname, char *if2name)
{
   int rval, zbuf;
   ec_etherheadert *ehp;

   /* 设置冗余端口指针 */
   context->port->redport = redport;

   /* 初始化主网卡 */
   ecx_setupnic(context->port, ifname, FALSE);

   /* 初始化备用网卡 */
   rval = ecx_setupnic(context->port, if2name, TRUE);

   /* 准备冗余操作用的"dummy" BRD发送帧 */
   ehp = (ec_etherheadert *)&(context->port->txbuf2);
   ehp->sa1 = oshw_htons(secMAC[0]);
   zbuf = 0;
   ecx_setupdatagram(context->port, &(context->port->txbuf2), EC_CMD_BRD, 0, 0x0000, 0x0000, 2, &zbuf);
   context->port->txbuflength2 = ETH_HEADERSIZE + EC_HEADERSIZE + EC_WKCSIZE + 2;

   return rval;
}

/** 关闭库。
 *
 * 释放网络接口资源，关闭EtherCAT通信。
 *
 * @param[in]  context = 上下文结构体
 */
void ecx_close(ecx_contextt *context)
{
   ecx_closenic(context->port);
};

/**
 * 从从站EEPROM的SII（从站信息接口）中获取一个字节
 *
 * 该函数通过缓存从从站EEPROM中读取一个字节的数据。如果缓存位置为空，
 * 则向从站发送读取请求。根据从站的能力，请求可以是4字节或8字节。
 * 使用缓存机制可以提高读取效率，避免重复读取相同的数据。
 *
 * 主要功能：
 * 1. 检查请求的字节是否已在缓存中
 * 2. 如果不在缓存中，从EEPROM读取数据并缓存
 * 3. 使用位图跟踪缓存中的数据
 * 4. 支持4字节和8字节EEPROM读取响应
 *
 * @param[in] context  EtherCAT上下文结构体
 * @param[in] slave    从站编号，0表示主站（传入0就是清空缓存）
 * @param[in] address  EEPROM字节地址（从站使用字地址），传入 EC_MAXEEPBUF 不读取任何数据
 * @return 请求的字节，如果不可用则返回0xff
 */
uint8 ecx_siigetbyte(ecx_contextt *context, uint16 slave, uint16 address)
{
   uint16 configadr, eadr;      // 配置地址、EEPROM字地址
   uint64 edat64;               // 64位EEPROM数据
   uint32 edat32;               // 32位EEPROM数据
   uint16 mapw, mapb;           // 位图字索引、位索引
   int lp,cnt;                  // 循环变量、字节数
   uint8 retval;                // 返回值

   // 默认返回值为0xff（表示失败）
   retval = 0xff;

   // 检查是否是同一个从站
   if (slave != context->esislave) /* 不是同一个从站？ */
   {
      // 清除ESI缓存位图
      memset(context->esimap, 0x00, EC_MAXEEPBITMAP * sizeof(uint32));
      context->esislave = slave;
   }

   // 检查地址是否在有效范围内
   if (address < EC_MAXEEPBUF)
   {
      // 计算位图位置
      mapw = address >> 5;                          // 字索引（每32字节一个字）
      mapb = (uint16)(address - (mapw << 5));       // 位索引（0-31）

      // 检查字节是否已在缓存中
      if (context->esimap[mapw] & (1U << mapb))
      {
         /* 字节已在缓存中 */
         retval = context->esibuf[address];
      }
      else
      {
         /* 字节不在缓存中，需要读取 */
         configadr = context->slavelist[slave].configadr;

         // 将EEPROM控制权交给主站
         ecx_eeprom2master(context, slave);

         // 计算EEPROM字地址（字节地址除以2）
         eadr = address >> 1;

         // 从EEPROM读取数据
         edat64 = ecx_readeepromFP (context, configadr, eadr, EC_TIMEOUTEEP);

         /* 8字节响应 */
         if (context->slavelist[slave].eep_8byte)
         {
            // 存储8字节数据到缓存
            put_unaligned64(edat64, &(context->esibuf[eadr << 1]));
            cnt = 8;
         }
         /* 4字节响应 */
         else
         {
            // 存储4字节数据到缓存
            edat32 = (uint32)edat64;
            put_unaligned32(edat32, &(context->esibuf[eadr << 1]));
            cnt = 4;
         }

         /* 查找位图位置 */
         mapw = eadr >> 4;                          // 字索引
         mapb = (uint16)((eadr << 1) - (mapw << 5)); // 位索引

         // 为每个读取的字节设置位图
         for(lp = 0 ; lp < cnt ; lp++)
         {
            /* 为每个读取的字节设置位图 */
            context->esimap[mapw] |= (1U << mapb);
            mapb++;

            // 如果位索引超过31，移动到下一个字
            if (mapb > 31)
            {
               mapb = 0;
               mapw++;
            }
         }

         // 从缓存中返回请求的字节
         retval = context->esibuf[address];
      }
   }

   return retval;
}

/** 在从站EEPROM中查找SII段头部。
 *
 * SII（从站信息接口）数据按段组织，每个段有一个类别标识。
 * 此函数遍历SII数据查找指定类别的段。
 *
 * SII段类别包括：
 * - ECT_SII_STRING (10): 字符串段
 * - ECT_SII_GENERAL (30): 常规信息段
 * - ECT_SII_FMMU (40): FMMU配置段
 * - ECT_SII_SM (41): 同步管理器配置段
 * - ECT_SII_PDO (50/51): PDO配置段
 *
 * @param[in]  context = 上下文结构体
 * @param[in]  slave   = 从站号
 * @param[in]  cat     = 要查找的段类别
 * @return 段长度条目处的字节地址，如果不可用则返回0
 */
int16 ecx_siifind(ecx_contextt *context, uint16 slave, uint16 cat)
{
   int16 a;
   uint16 p;
   uint8 eectl = context->slavelist[slave].eep_pdi;

   /* 从SII起始地址开始 */
   a = ECT_SII_START << 1;

   /* 读取第一个SII段类别 */
   p = ecx_siigetbyte(context, slave, a++);
   p += (ecx_siigetbyte(context, slave, a++) << 8);

   /* 遍历SII直到找到目标类别或到达EOF */
   while ((p != cat) && (p != 0xffff))
   {
      /* 读取段长度 */
      p = ecx_siigetbyte(context, slave, a++);
      p += (ecx_siigetbyte(context, slave, a++) << 8);

      /* 定位下一个段类别 */
      a += p << 1;

      /* 读取段类别 */
      p = ecx_siigetbyte(context, slave, a++);
      p += (ecx_siigetbyte(context, slave, a++) << 8);
   }

   if (p != cat)
   {
      a = 0;
   }

   /* 如果EEPROM之前由PDI控制，恢复控制权 */
   if (eectl)
   {
      ecx_eeprom2pdi(context, slave);
   }

   return a;
}

/**
 * 从从站EEPROM的SII字符串区域获取字符串
 *
 * 该函数从从站EEPROM的SII（从站信息接口）字符串区域读取指定编号的字符串。
 * SII字符串区域存储了从站的名称、描述等文本信息。
 *
 * SII字符串区域结构：
 * - 字节0-1: 字符串数量
 * - 后续: 每个字符串以长度字节开头，后跟字符串内容
 *
 * 字符串读取流程：
 * 1. 查找SII字符串区域的起始位置
 * 2. 读取字符串总数
 * 3. 遍历字符串直到找到请求的字符串
 * 4. 复制字符串内容到输出缓冲区
 * 5. 添加字符串终止符
 *
 * @param[in]  context EtherCAT上下文结构体
 * @param[out] str     输出字符串缓冲区，如果未找到则返回空字符串
 * @param[in]  slave   从站编号
 * @param[in]  Sn      字符串编号（从1开始）
 */
void ecx_siistring(ecx_contextt *context, char *str, uint16 slave, uint16 Sn)
{
   uint16 a,i,j,l,n,ba;  // 地址变量、循环变量、长度、字符串数量、基地址
   char *ptr;
   uint8 eectl = context->slavelist[slave].eep_pdi;  // 保存EEPROM控制状态

   ptr = str;

   /* 查找字符串段 */
   a = ecx_siifind (context, slave, ECT_SII_STRING);

   if (a > 0)
   {
      ba = a + 2; /* 跳过SII段头部 */
      n = ecx_siigetbyte(context, slave, ba++); /* 读取段中的字符串数量 */

      if (Sn <= n) /* 请求的字符串是否存在？ */
      {
         // 遍历字符串，找到请求的字符串编号
         for (i = 1; i <= Sn; i++) /* walk through strings */
         {
            l = ecx_siigetbyte(context, slave, ba++); /* 此字符串的长度 */
            // 如果不是请求的字符串，跳过
            if (i < Sn)
            {
               ba += l;  /* 跳过非目标字符串 */
            }
            else
            {
               // 找到请求的字符串，复制内容
               ptr = str;
               for (j = 1; j <= l; j++) /* 复制一个字符串 */
               {
                  // 限制字符串长度不超过EC_MAXNAME
                  if(j <= EC_MAXNAME)
                  {
                     *ptr = (char)ecx_siigetbyte(context, slave, ba++);
                     ptr++;
                  }
                  else
                  {
                     ba++;
                  }
               }
            }
         }
         *ptr = 0; /* 添加零终止符 */
      }
      else
      {
         // 请求的字符串编号超出范围，返回空字符串
         ptr = str;
         *ptr = 0; /* 空字符串 */
      }
   }

   /* 如果EEPROM之前由PDI控制，恢复控制权 */
   if (eectl)
   {
      ecx_eeprom2pdi(context, slave);
   }
}

/** 从从站EEPROM的SII FMMU段获取FMMU数据。
 *
 * FMMU（现场总线内存管理单元）用于将逻辑地址映射到物理地址。
 * 最多支持4个FMMU。
 *
 * @param[in]  context = 上下文结构体
 * @param[in]  slave   = 从站号
 * @param[out] FMMU    = 从SII获取的FMMU结构体，最多4个FMMU
 * @return 段中定义的FMMU数量
 */
uint16 ecx_siiFMMU(ecx_contextt *context, uint16 slave, ec_eepromFMMUt* FMMU)
{
   uint16  a;
   uint8 eectl = context->slavelist[slave].eep_pdi;

   /* 初始化FMMU结构体 */
   FMMU->nFMMU = 0;
   FMMU->FMMU0 = 0;
   FMMU->FMMU1 = 0;
   FMMU->FMMU2 = 0;
   FMMU->FMMU3 = 0;

   /* 查找FMMU段 */
   FMMU->Startpos = ecx_siifind(context, slave, ECT_SII_FMMU);

   if (FMMU->Startpos > 0)
   {
      a = FMMU->Startpos;
      /* 读取FMMU数量（字数，需要除以2得到实际FMMU数） */
      FMMU->nFMMU = ecx_siigetbyte(context, slave, a++);
      FMMU->nFMMU += (ecx_siigetbyte(context, slave, a++) << 8);
      FMMU->nFMMU *= 2;

      /* 读取每个FMMU的使用标志 */
      FMMU->FMMU0 = ecx_siigetbyte(context, slave, a++);
      FMMU->FMMU1 = ecx_siigetbyte(context, slave, a++);
      if (FMMU->nFMMU > 2)
      {
         FMMU->FMMU2 = ecx_siigetbyte(context, slave, a++);
         FMMU->FMMU3 = ecx_siigetbyte(context, slave, a++);
      }
   }

   /* 如果EEPROM之前由PDI控制，恢复控制权 */
   if (eectl)
   {
      ecx_eeprom2pdi(context, slave);
   }

   return FMMU->nFMMU;
}

/** 从从站EEPROM的SII SM段获取同步管理器数据。
 *
 * 同步管理器（SM）用于控制从站内存的访问方式。
 * SM配置包括物理起始地址、长度、控制寄存器等。
 *
 * @param[in]  context = 上下文结构体
 * @param[in]  slave   = 从站号
 * @param[out] SM      = 从SII获取的第一个SM结构体
 * @return 段中定义的SM数量
 */
uint16 ecx_siiSM(ecx_contextt *context, uint16 slave, ec_eepromSMt* SM)
{
   uint16 a,w;
   uint8 eectl = context->slavelist[slave].eep_pdi;

   SM->nSM = 0;

   /* 查找SM段 */
   SM->Startpos = ecx_siifind(context, slave, ECT_SII_SM);

   if (SM->Startpos > 0)
   {
      a = SM->Startpos;

      /* 读取段长度并计算SM数量（每个SM占4字） */
      w = ecx_siigetbyte(context, slave, a++);
      w += (ecx_siigetbyte(context, slave, a++) << 8);
      SM->nSM = (uint8)(w / 4);

      /* 读取第一个SM的配置 */
      SM->PhStart = ecx_siigetbyte(context, slave, a++);
      SM->PhStart += (ecx_siigetbyte(context, slave, a++) << 8);
      SM->Plength = ecx_siigetbyte(context, slave, a++);
      SM->Plength += (ecx_siigetbyte(context, slave, a++) << 8);
      SM->Creg = ecx_siigetbyte(context, slave, a++);
      SM->Sreg = ecx_siigetbyte(context, slave, a++);
      SM->Activate = ecx_siigetbyte(context, slave, a++);
      SM->PDIctrl = ecx_siigetbyte(context, slave, a++);
   }

   /* 如果EEPROM之前由PDI控制，恢复控制权 */
   if (eectl)
   {
      ecx_eeprom2pdi(context, slave);
   }

   return SM->nSM;
}

/** 从从站EEPROM的SII SM段获取下一个SM数据。
 *
 * 用于遍历所有SM配置，每次调用获取一个SM。
 *
 * @param[in]  context = 上下文结构体
 * @param[in]  slave   = 从站号
 * @param[out] SM      = SM结构体
 * @param[in]  n       = SM编号（0开始）
 * @return >0: 成功；0: 没有更多SM
 */
uint16 ecx_siiSMnext(ecx_contextt *context, uint16 slave, ec_eepromSMt* SM, uint16 n)
{
   uint16 a;
   uint16 retVal = 0;
   uint8 eectl = context->slavelist[slave].eep_pdi;

   if (n < SM->nSM)
   {
      /* 计算第n个SM的起始地址 */
      a = SM->Startpos + 2 + (n * 8);

      /* 读取SM配置 */
      SM->PhStart = ecx_siigetbyte(context, slave, a++);
      SM->PhStart += (ecx_siigetbyte(context, slave, a++) << 8);
      SM->Plength = ecx_siigetbyte(context, slave, a++);
      SM->Plength += (ecx_siigetbyte(context, slave, a++) << 8);
      SM->Creg = ecx_siigetbyte(context, slave, a++);
      SM->Sreg = ecx_siigetbyte(context, slave, a++);
      SM->Activate = ecx_siigetbyte(context, slave, a++);
      SM->PDIctrl = ecx_siigetbyte(context, slave, a++);
      retVal = 1;
   }
   if (eectl)
   {
      /* 如果EEPROM之前由PDI控制，恢复控制权 */
      ecx_eeprom2pdi(context, slave);
   }

   return retVal;
}

/** 从从站EEPROM的SII PDO段获取PDO数据。
 *
 * PDO（过程数据对象）用于周期性数据交换。
 * 分为RX PDO（主站到从站）和TX PDO（从站到主站）。
 *
 * @param[in]  context = 上下文结构体
 * @param[in]  slave   = 从站号
 * @param[out] PDO     = 从SII获取的PDO结构体
 * @param[in]  t       = 0: RX PDO；1: TX PDO
 * @return PDO的映射大小（位）
 */
uint32 ecx_siiPDO(ecx_contextt *context, uint16 slave, ec_eepromPDOt* PDO, uint8 t)
{
   uint16 a , w, c, e, er, Size;
   uint8 eectl = context->slavelist[slave].eep_pdi;

   Size = 0;
   PDO->nPDO = 0;
   PDO->Length = 0;
   PDO->Index[1] = 0;

   /* 初始化每个SM的位大小 */
   for (c = 0 ; c < EC_MAXSM ; c++) PDO->SMbitsize[c] = 0;

   if (t > 1)
      t = 1;

   /* 查找PDO段（RX PDO或TX PDO） */
   PDO->Startpos = ecx_siifind(context, slave, ECT_SII_PDO + t);

   if (PDO->Startpos > 0)
   {
      a = PDO->Startpos;

      /* 读取段长度 */
      w = ecx_siigetbyte(context, slave, a++);
      w += (ecx_siigetbyte(context, slave, a++) << 8);
      PDO->Length = w;
      c = 1;

      /* 遍历所有PDO */
      do
      {
         PDO->nPDO++;

         /* 读取PDO索引 */
         PDO->Index[PDO->nPDO] = ecx_siigetbyte(context, slave, a++);
         PDO->Index[PDO->nPDO] += (ecx_siigetbyte(context, slave, a++) << 8);
         PDO->BitSize[PDO->nPDO] = 0;
         c++;

         /* 读取条目数量和关联的SM */
         e = ecx_siigetbyte(context, slave, a++);
         PDO->SyncM[PDO->nPDO] = ecx_siigetbyte(context, slave, a++);
         a += 4;
         c += 2;

         if (PDO->SyncM[PDO->nPDO] < EC_MAXSM) /* SM是否有效且在范围内？ */
         {
            /* 读取PDO中定义的所有条目 */
            for (er = 1; er <= e; er++)
            {
               c += 4;
               a += 5;
               PDO->BitSize[PDO->nPDO] += ecx_siigetbyte(context, slave, a++);
               a += 2;
            }
            PDO->SMbitsize[ PDO->SyncM[PDO->nPDO] ] += PDO->BitSize[PDO->nPDO];
            Size += PDO->BitSize[PDO->nPDO];
            c++;
         }
         else /* PDO已停用，因为SM为0xff或大于EC_MAXSM */
         {
            c += 4 * e;
            a += 8 * e;
            c++;
         }

         /* 限制缓冲区中的PDO条目数量 */
         if (PDO->nPDO >= (EC_MAXEEPDO - 1))
         {
            c = PDO->Length;
         }
      }
      while (c < PDO->Length);
   }

   /* 如果EEPROM之前由PDI控制，恢复控制权 */
   if (eectl)
   {
      ecx_eeprom2pdi(context, slave);
   }

   return (Size);
}

/** FPRD多播命令最大从站数 */
#define MAX_FPRD_MULTI 64

/** 使用FPRD命令读取多个从站的AL状态。
 *
 * FPRD（配置地址读取）命令可以同时读取多个从站的寄存器。
 * 此函数使用堆叠数据报来高效读取多个从站的AL状态。
 *
 * @param[in]  context    = 上下文结构体
 * @param[in]  n          = 从站数量
 * @param[in]  configlst  = 从站配置地址列表
 * @param[out] slstatlst  = 从站AL状态列表
 * @param[in]  timeout    = 超时时间（微秒）
 * @return 工作计数器值
 */
int ecx_FPRD_multi(ecx_contextt *context, int n, uint16 *configlst, ec_alstatust *slstatlst, int timeout)
{
   int wkc;
   uint8 idx;
   ecx_portt *port;
   uint16 sldatapos[MAX_FPRD_MULTI];
   int slcnt;

   port = context->port;
   idx = ecx_getindex(port);
   slcnt = 0;

   /* 设置第一个FPRD数据报 */
   ecx_setupdatagram(port, &(port->txbuf[idx]), EC_CMD_FPRD, idx,
      *(configlst + slcnt), ECT_REG_ALSTAT, sizeof(ec_alstatust), slstatlst + slcnt);
   sldatapos[slcnt] = EC_HEADERSIZE;

   /* 添加更多FPRD数据报（堆叠） */
   while(++slcnt < (n - 1))
   {
      sldatapos[slcnt] = ecx_adddatagram(port, &(port->txbuf[idx]), EC_CMD_FPRD, idx, TRUE,
                            *(configlst + slcnt), ECT_REG_ALSTAT, sizeof(ec_alstatust), slstatlst + slcnt);
   }
   if(slcnt < n)
   {
      sldatapos[slcnt] = ecx_adddatagram(port, &(port->txbuf[idx]), EC_CMD_FPRD, idx, FALSE,
                            *(configlst + slcnt), ECT_REG_ALSTAT, sizeof(ec_alstatust), slstatlst + slcnt);
   }
   wkc = ecx_srconfirm(port, idx, timeout);
   if (wkc >= 0)
   {
      for(slcnt = 0 ; slcnt < n ; slcnt++)
      {
         memcpy(slstatlst + slcnt, &(port->rxbuf[idx][sldatapos[slcnt]]), sizeof(ec_alstatust));
      }
   }
   ecx_setbufstat(port, idx, EC_BUF_EMPTY);
   return wkc;
}

/**
 * 读取所有从站的状态
 *
 * 该函数读取总线上所有从站的AL状态，并更新从站列表中的状态信息。
 * 采用优化的读取策略：首先尝试使用广播读取，如果所有从站状态一致且无错误，
 * 则无需发送额外的数据报；否则逐个读取从站状态。
 *
 * EtherCAT从站状态：
 * - INIT (0x01): 初始化状态
 * - PRE_OP (0x02): 预运行状态
 * - BOOT (0x03): 引导状态（与INIT|PRE_OP冲突）
 * - SAFE_OP (0x04): 安全运行状态
 * - OPERATIONAL (0x08): 运行状态
 *
 * 状态读取策略：
 * 1. 首先发送广播读(BRD)命令读取所有从站状态
 * 2. 如果所有从站状态一致且无错误标志，直接更新状态
 * 3. 否则使用FPRD_multi批量读取每个从站的状态
 *
 * @warning BOOT状态实际上比INIT和PRE_OP更高（参见状态表示）
 *          BOOT状态与PRE_OP | INIT冲突，无法在此函数中正确处理
 *
 * @param[in] context EtherCAT上下文结构体
 * @return 找到的最低状态值，用于判断整个系统的状态
 */
int ecx_readstate(ecx_contextt *context)
{
   uint16 slave, fslave, lslave, configadr, lowest, rval, bitwisestate;
   ec_alstatust sl[MAX_FPRD_MULTI];       // 从站AL状态结构数组
   uint16 slca[MAX_FPRD_MULTI];           // 从站配置地址数组
   boolean noerrorflag, allslavessamestate;
   boolean allslavespresent = FALSE;      // 所有从站是否都在线
   int wkc;

   /* Try to establish the state of all slaves sending only one broadcast datagram.
    * This way a number of datagrams equal to the number of slaves will be sent only if needed.*/
   // 尝试使用广播读命令获取所有从站状态，优化性能
   rval = 0;
   wkc = ecx_BRD(context->port, 0, ECT_REG_ALSTAT, sizeof(rval), &rval, EC_TIMEOUTRET);

   // 检查工作计数器是否等于或大于从站数量，判断所有从站是否都在线
   if(wkc >= *(context->slavecount))
   {
      allslavespresent = TRUE;
   }

   // 转换字节序并提取状态位
   rval = etohs(rval);
   bitwisestate = (rval & 0x0f);          // 低4位是状态值

   // 检查是否有错误标志
   if ((rval & EC_STATE_ERROR) == 0)
   {
      noerrorflag = TRUE;
      context->slavelist[0].ALstatuscode = 0;
   }
   else
   {
      noerrorflag = FALSE;
   }

   // 判断所有从站是否处于相同状态
   switch (bitwisestate)
   {
       /* Note: BOOT State collides with PRE_OP | INIT and cannot be used here */
      case EC_STATE_INIT:
      case EC_STATE_PRE_OP:
      case EC_STATE_SAFE_OP:
      case EC_STATE_OPERATIONAL:
         allslavessamestate = TRUE;
         context->slavelist[0].state = bitwisestate;
         break;
      default:
         allslavessamestate = FALSE;
         break;
   }

   // 如果无错误、状态一致且所有从站在线，直接更新状态
   if (noerrorflag && allslavessamestate && allslavespresent)
   {
      /* No slave has toggled the error flag so the alstatuscode
       * (even if different from 0) should be ignored and
       * the slaves have reached the same state so the internal state
       * can be updated without sending any datagram. */
      for (slave = 1; slave <= *(context->slavecount); slave++)
      {
         context->slavelist[slave].ALstatuscode = 0x0000;
         context->slavelist[slave].state = bitwisestate;
      }
      lowest = bitwisestate;
   }
   else
   {
      /* Not all slaves have the same state or at least one is in error so one datagram per slave
       * is needed. */
      // 需要逐个读取从站状态
      context->slavelist[0].ALstatuscode = 0;
      lowest = 0xff;                       // 初始化为最高值
      fslave = 1;
      do
      {
         // 批量处理，每次最多处理MAX_FPRD_MULTI个从站
         lslave = (uint16)*(context->slavecount);
         if ((lslave - fslave) >= MAX_FPRD_MULTI)
         {
            lslave = fslave + MAX_FPRD_MULTI - 1;
         }
         // 准备批量读取的数据结构
         for (slave = fslave; slave <= lslave; slave++)
         {
            const ec_alstatust zero = { 0, 0, 0 };

            configadr = context->slavelist[slave].configadr;
            slca[slave - fslave] = configadr;
            sl[slave - fslave] = zero;
         }
         // 批量读取从站AL状态
         ecx_FPRD_multi(context, (lslave - fslave) + 1, &(slca[0]), &(sl[0]), EC_TIMEOUTRET3);
         // 处理读取结果
         for (slave = fslave; slave <= lslave; slave++)
         {
            configadr = context->slavelist[slave].configadr;
            rval = etohs(sl[slave - fslave].alstatus);
            context->slavelist[slave].ALstatuscode = etohs(sl[slave - fslave].alstatuscode);
            // 记录最低状态值
            if ((rval & 0xf) < lowest)
            {
               lowest = (rval & 0xf);
            }
            context->slavelist[slave].state = rval;
            // 主站记录所有从站的状态码OR值
            context->slavelist[0].ALstatuscode |= context->slavelist[slave].ALstatuscode;
         }
         fslave = lslave + 1;
      } while (lslave < *(context->slavecount));
      context->slavelist[0].state = lowest;
   }

   return lowest;
}

/**
 * 写入从站状态
 *
 * 该函数向从站写入请求的状态，如果slave=0则写入所有从站。
 * 函数不检查实际状态是否已改变，仅发送状态写入命令。
 *
 * EtherCAT状态转换：
 * - INIT -> PRE_OP: 初始化邮箱通信
 * - PRE_OP -> SAFE_OP: 配置同步管理器和FMMU
 * - SAFE_OP -> OPERATIONAL: 启动过程数据交换
 * - 任何状态 -> INIT: 完全复位
 *
 * 写入策略：
 * - slave = 0: 使用广播写(BWR)命令写入所有从站
 * - slave > 0: 使用配置地址写(FPWR)命令写入指定从站
 *
 * @param[in] context EtherCAT上下文结构体
 * @param[in] slave   从站编号，0表示写入所有从站
 * @return 工作计数器(WKC)，或EC_NOFRAME表示发送失败
 */
int ecx_writestate(ecx_contextt *context, uint16 slave)
{
   int ret;
   uint16 configadr, slstate;

   if (slave == 0)
   {
      /* 向所有从站广播状态请求 */
      slstate = htoes(context->slavelist[slave].state);
      ret = ecx_BWR(context->port, 0, ECT_REG_ALCTL, sizeof(slstate),
	            &slstate, EC_TIMEOUTRET3);
   }
   else
   {
      /* 向单个从站发送状态请求 */
      configadr = context->slavelist[slave].configadr;

      ret = ecx_FPWRw(context->port, configadr, ECT_REG_ALCTL,
	        htoes(context->slavelist[slave].state), EC_TIMEOUTRET3);
   }
   return ret;
}

/**
 * 检查从站的实际状态
 *
 * 这是一个阻塞函数，用于检查从站是否达到请求的状态。
 * 要刷新所有从站的状态，应该调用ecx_readstate()函数。
 *
 * @warning 如果用于从站0（=所有从站），所有从站的状态通过按位OR操作读取。
 * 返回值也是所有从站状态的按位OR值。
 * 这对BOOT状态有一些影响。BOOT状态表示与INIT | PRE_OP冲突，
 * 因此此函数不能用于slave = 0且reqstate = EC_STATE_BOOT的情况。
 * 同样，如果返回的状态是BOOT，某些从站可能实际上处于INIT和PRE_OP状态，而不是BOOT状态。
 *
 * @param[in] context     EtherCAT上下文结构体
 * @param[in] slave       从站编号，0 = 所有从站（仅刷新"slavelist[0].state"）
 * @param[in] reqstate    请求的状态
 * @param[in] timeout     超时值（微秒）
 * @return 请求的状态，或超时后找到的状态
 */
uint16 ecx_statecheck(ecx_contextt *context, uint16 slave, uint16 reqstate, int timeout)
{
   uint16 configadr, state, rval;    // 配置地址、状态、返回值
   ec_alstatust slstat;              // AL状态结构体
   osal_timert timer;                // 定时器

   // 检查从站编号是否有效
   if ( slave > *(context->slavecount) )
   {
      return 0;
   }

   // 启动定时器
   osal_timer_start(&timer, timeout);

   // 获取从站配置地址
   configadr = context->slavelist[slave].configadr;

   // 循环检查状态，直到达到请求状态或超时
   do
   {
      if (slave < 1)
      {
         // 从站0表示所有从站，使用广播读取
         rval = 0;
         ecx_BRD(context->port, 0, ECT_REG_ALSTAT, sizeof(rval), &rval , EC_TIMEOUTRET);
         rval = etohs(rval);
      }
      else
      {
         // 读取单个从站的状态
         slstat.alstatus = 0;
         slstat.alstatuscode = 0;
         ecx_FPRD(context->port, configadr, ECT_REG_ALSTAT, sizeof(slstat), &slstat, EC_TIMEOUTRET);
         rval = etohs(slstat.alstatus);
         context->slavelist[slave].ALstatuscode = etohs(slstat.alstatuscode);
      }

      // 读取从站状态（低4位）
      state = rval & 0x000f;

      // 如果状态不匹配，等待1毫秒后重试
      if (state != reqstate)
      {
         osal_usleep(1000);
      }
   }
   while ((state != reqstate) && (osal_timer_is_expired(&timer) == FALSE));

   // 更新从站状态
   context->slavelist[slave].state = rval;

   return state;
}

/**
 * 获取下一个邮箱计数器值
 *
 * 邮箱计数器用于邮箱链路层协议，范围是1-7。
 * 计数器用于跟踪邮箱通信的请求和响应配对。
 *
 * @param[in] cnt 当前的邮箱计数器值 [0..7]
 * @return 下一个邮箱计数器值 [1..7]
 */
uint8 ec_nextmbxcnt(uint8 cnt)
{
   cnt++;
   if (cnt > 7)
   {
      cnt = 1; /* wrap around to 1, not 0 */
   }

   return cnt;
}

/**
 * 清除邮箱缓冲区
 *
 * 将邮箱缓冲区清零，用于初始化邮箱通信。
 *
 * @param[out] Mbx 要清除的邮箱缓冲区指针
 */
void ec_clearmbx(ec_mbxbuft *Mbx)
{
    memset(Mbx, 0x00, EC_MAXMBX);
}

/**
 * 检查从站的输入邮箱是否为空
 *
 * 该函数检查从站的输入邮箱（IN mailbox）是否为空，可以接收新数据。
 * 通过读取同步管理器状态寄存器来判断邮箱状态。
 *
 * 同步管理器状态寄存器位定义：
 * - Bit 0: 邮箱已满标志
 * - Bit 1: 邮箱读取请求
 * - Bit 2: 邮箱写入请求
 * - Bit 3: 邮箱空标志
 *
 * @param[in] context EtherCAT上下文结构体
 * @param[in] slave   从站编号
 * @param[in] timeout 超时时间（微秒）
 * @return >0 表示邮箱为空，可以写入；0 表示邮箱不为空或超时
 */
int ecx_mbxempty(ecx_contextt *context, uint16 slave, int timeout)
{
   uint16 configadr;
   uint8 SMstat;
   int wkc;
   osal_timert timer;

   osal_timer_start(&timer, timeout);
   configadr = context->slavelist[slave].configadr;
   do
   {
      SMstat = 0;
      // 读取SM0状态寄存器（输入邮箱状态）
      wkc = ecx_FPRD(context->port, configadr, ECT_REG_SM0STAT, sizeof(SMstat), &SMstat, EC_TIMEOUTRET);
      SMstat = etohs(SMstat);
      // 如果邮箱不为空，等待一段时间后重试
      if (((SMstat & 0x08) != 0) && (timeout > EC_LOCALDELAY))
      {
         osal_usleep(EC_LOCALDELAY);
      }
   }
   while (((wkc <= 0) || ((SMstat & 0x08) != 0)) && (osal_timer_is_expired(&timer) == FALSE));

   // 检查邮箱是否为空（Bit 3 = 0 表示邮箱为空）
   if ((wkc > 0) && ((SMstat & 0x08) == 0))
   {
      return 1;
   }

   return 0;
}

/** 向从站写入输入邮箱数据。
 *
 * 该函数向指定从站的输入邮箱（Write Mailbox）写入邮箱数据。
 * 写入前会检查邮箱是否为空，确保不会覆盖未读取的数据。
 * 邮箱通信用于非实时数据交换，如CoE、EoE、FoE等协议。
 *
 * 邮箱写入流程：
 * 1. 检查从站是否支持邮箱通信
 * 2. 等待输入邮箱为空（读取SM状态）
 * 3. 将数据写入邮箱缓冲区
 *
 * @param[in]  context  = 上下文结构体
 * @param[in]  slave    = 从站号
 * @param[in]  mbx      = 要发送的邮箱数据
 * @param[in]  timeout  = 等待邮箱为空的超时时间（微秒）
 * @return 工作计数器值（>0表示成功）
 */
int ecx_mbxsend(ecx_contextt *context, uint16 slave,ec_mbxbuft *mbx, int timeout)
{
   uint16 mbxwo,mbxl,configadr;
   int wkc;

   wkc = 0;
   configadr = context->slavelist[slave].configadr;
   mbxl = context->slavelist[slave].mbx_l;

   /* 检查邮箱长度是否有效 */
   if ((mbxl > 0) && (mbxl <= EC_MAXMBX))
   {
      // 检查邮箱是否为空
      if (ecx_mbxempty(context, slave, timeout))
      {
         mbxwo = context->slavelist[slave].mbx_wo;  // 邮箱写入偏移地址
         /* write slave in mailbox */
         // 写入数据到从站输入邮箱
         wkc = ecx_FPWR(context->port, configadr, mbxwo, mbxl, mbx, EC_TIMEOUTRET3);
      }
      else
      {
         wkc = 0;
      }
   }

   return wkc;
}

/** 从从站接收邮箱数据，阻塞式。
 *
 * 该函数从指定从站的输出邮箱（Read Mailbox）读取邮箱数据。
 * 使用同步管理器状态位来判断邮箱是否有数据可读。
 * 函数会阻塞等待直到邮箱有数据或超时。
 * 支持邮箱链路层的重复请求机制。
 *
 * 邮箱读取流程：
 * 1. 等待从站输出邮箱状态寄存器的bit3（Mailbox Read Status）置位
 * 2. 读取邮箱内容
 * 3. 处理特殊响应类型（错误响应、紧急消息、EoE分片）
 * 4. 如果读取失败，尝试重置邮箱状态并重试
 *
 * 特殊处理：
 * - 邮箱错误响应（类型0）：调用ecx_mbxerror处理
 * - CoE紧急消息（服务类型0x01）：调用ecx_mbxemergencyerror处理
 * - EoE分片数据：调用EOEhook处理
 *
 * @param[in]  context   = 上下文结构体
 * @param[in]  slave     = 从站号
 * @param[out] mbx       = 接收邮箱数据的缓冲区
 * @param[in]  timeout   = 超时时间（微秒）
 * @return >0: 成功，工作计数器值；0: 超时或失败
 */
int ecx_mbxreceive(ecx_contextt *context, uint16 slave, ec_mbxbuft *mbx, int timeout)
{
   uint16 mbxro,mbxl,configadr;   /* mbxro: 邮箱读偏移, mbxl: 邮箱长度, configadr: 配置地址 */
   int wkc=0;                      /* 工作计数器 */
   int wkc2;                       /* 辅助工作计数器 */
   uint16 SMstat;                  /* 同步管理器状态寄存器 */
   uint8 SMcontr;                  /* 同步管理器控制寄存器 */
   ec_mbxheadert *mbxh;            /* 邮箱头部指针 */
   ec_emcyt *EMp;                  /* 紧急消息指针 */
   ec_mbxerrort *MBXEp;            /* 邮箱错误指针 */

   /* 获取从站配置地址和邮箱长度 */
   configadr = context->slavelist[slave].configadr;
   mbxl = context->slavelist[slave].mbx_rl;  // 读取邮箱长度

   /* 检查邮箱长度是否有效 */
   if ((mbxl > 0) && (mbxl <= EC_MAXMBX))
   {
      osal_timert timer;

      /* 启动超时定时器 */
      osal_timer_start(&timer, timeout);
      wkc = 0;

      /* 等待输出邮箱有数据可读 */
      /* SM状态寄存器bit3 (0x08) 表示邮箱有数据 */
      do /* wait for read mailbox available */
      {
         SMstat = 0;
         // 读取SM1状态寄存器（输出邮箱状态）
         wkc = ecx_FPRD(context->port, configadr, ECT_REG_SM1STAT, sizeof(SMstat), &SMstat, EC_TIMEOUTRET);
         SMstat = etohs(SMstat);
         // 如果邮箱为空，等待一段时间后重试
         if (((SMstat & 0x08) == 0) && (timeout > EC_LOCALDELAY))
         {
            osal_usleep(EC_LOCALDELAY);
         }
      }
      while (((wkc <= 0) || ((SMstat & 0x08) == 0)) && (osal_timer_is_expired(&timer) == FALSE));

      /* 邮箱有数据可读？ */
      if ((wkc > 0) && ((SMstat & 0x08) > 0)) /* read mailbox available ? */
      {
         mbxro = context->slavelist[slave].mbx_ro;  // 读取邮箱偏移地址
         mbxh = (ec_mbxheadert *)mbx;

         do
         {
            /* 从从站邮箱读取数据 */
            wkc = ecx_FPRD(context->port, configadr, mbxro, mbxl, mbx, EC_TIMEOUTRET); /* get mailbox */

            /* 检查是否为邮箱错误响应（类型字段为0） */
            if ((wkc > 0) && ((mbxh->mbxtype & 0x0f) == 0x00)) /* Mailbox error response? */
            {
               MBXEp = (ec_mbxerrort *)mbx;
               /* 处理邮箱错误 */
               ecx_mbxerror(context, slave, etohs(MBXEp->Detail));
               wkc = 0; /* 防止错误向上传递，已在此处理 */
            }
            // 检查是否是CoE响应
            else if ((wkc > 0) && ((mbxh->mbxtype & 0x0f) == ECT_MBXT_COE)) /* CoE response? */
            {
               EMp = (ec_emcyt *)mbx;
               /* CANopen服务类型为0x01表示紧急消息 */
               if ((etohs(EMp->CANOpen) >> 12) == 0x01) /* Emergency request? */
               {
                  /* 处理紧急消息 */
                  ecx_mbxemergencyerror(context, slave, etohs(EMp->ErrorCode), EMp->ErrorReg,
                          EMp->bData, etohs(EMp->w1), etohs(EMp->w2));
                  wkc = 0; /* 防止紧急消息向上传递，已在此处理 */
               }
            }
            // 检查是否是EoE响应
            else if ((wkc > 0) && ((mbxh->mbxtype & 0x0f) == ECT_MBXT_EOE)) /* EoE response? */
            {
               ec_EOEt * eoembx = (ec_EOEt *)mbx;
               uint16 frameinfo1 = etohs(eoembx->frameinfo1);
               /* All non fragment data frame types are expected to be handled by
               * slave send/receive API if the EoE hook is set
               */
               // 处理EoE分片数据
               if (EOE_HDR_FRAME_TYPE_GET(frameinfo1) == EOE_FRAG_DATA)
               {
                  if (context->EOEhook)
                  {
                     if (context->EOEhook(context, slave, eoembx) > 0)
                     {
                        /* 分片已由EoE钩子处理 */
                        wkc = 0;
                     }
                  }
               }
            }
            else
            {
               // 读取邮箱丢失，使用链路层重试机制
               if (wkc <= 0) /* read mailbox lost */
               {
                  /* 切换重复请求位，触发从站重发 */
                  SMstat ^= 0x0200; /* toggle repeat request */
                  SMstat = htoes(SMstat);
                  wkc2 = ecx_FPWR(context->port, configadr, ECT_REG_SM1STAT, sizeof(SMstat), &SMstat, EC_TIMEOUTRET);
                  SMstat = etohs(SMstat);

                  /* 等待从站确认切换 */
                  do /* wait for toggle ack */
                  {
                     wkc2 = ecx_FPRD(context->port, configadr, ECT_REG_SM1CONTR, sizeof(SMcontr), &SMcontr, EC_TIMEOUTRET);
                   } while (((wkc2 <= 0) || ((SMcontr & 0x02) != (HI_BYTE(SMstat) & 0x02))) && (osal_timer_is_expired(&timer) == FALSE));

                  /* 等待邮箱再次有数据可读 */
                  do /* wait for read mailbox available */
                  {
                     wkc2 = ecx_FPRD(context->port, configadr, ECT_REG_SM1STAT, sizeof(SMstat), &SMstat, EC_TIMEOUTRET);
                     SMstat = etohs(SMstat);
                     if (((SMstat & 0x08) == 0) && (timeout > EC_LOCALDELAY))
                     {
                        osal_usleep(EC_LOCALDELAY);
                     }
                  } while (((wkc2 <= 0) || ((SMstat & 0x08) == 0)) && (osal_timer_is_expired(&timer) == FALSE));
               }
            }
         } while ((wkc <= 0) && (osal_timer_is_expired(&timer) == FALSE)); /* if WKC<=0 repeat */
      }
      else /* no read mailbox available */
      {
         /* 邮箱无数据，返回超时 */
         if (wkc > 0)
            wkc = EC_TIMEOUT;
      }
   }

   return wkc;
}

/** Dump complete EEPROM data from slave in buffer.
 * @param[in]  context  = context struct
 * @param[in]  slave    = Slave number
 * @param[out] esibuf   = EEPROM data buffer, make sure it is big enough.
 */
void ecx_esidump(ecx_contextt *context, uint16 slave, uint8 *esibuf)
{
   uint16 configadr, address, incr;
   uint64 *p64;
   uint16 *p16;
   uint64 edat;
   uint8 eectl = context->slavelist[slave].eep_pdi;

   ecx_eeprom2master(context, slave); /* set eeprom control to master */
   configadr = context->slavelist[slave].configadr;
   address = ECT_SII_START;
   p16=(uint16*)esibuf;
   if (context->slavelist[slave].eep_8byte)
   {
      incr = 4;
   }
   else
   {
      incr = 2;
   }
   do
   {
      edat = ecx_readeepromFP(context, configadr, address, EC_TIMEOUTEEP);
      p64 = (uint64*)p16;
      *p64 = edat;
      p16 += incr;
      address += incr;
   } while ((address <= (EC_MAXEEPBUF >> 1)) && ((uint32)edat != 0xffffffff));

   if (eectl)
   {
      ecx_eeprom2pdi(context, slave); /* if eeprom control was previously pdi then restore */
   }
}

/** Read EEPROM from slave bypassing cache.
 * @param[in] context   = context struct
 * @param[in] slave     = Slave number
 * @param[in] eeproma   = (WORD) Address in the EEPROM
 * @param[in] timeout   = Timeout in us.
 * @return EEPROM data 32bit
 */
uint32 ecx_readeeprom(ecx_contextt *context, uint16 slave, uint16 eeproma, int timeout)
{
   uint16 configadr;

   ecx_eeprom2master(context, slave); /* set eeprom control to master */
   configadr = context->slavelist[slave].configadr;

   return ((uint32)ecx_readeepromFP(context, configadr, eeproma, timeout));
}

/** Write EEPROM to slave bypassing cache.
 * @param[in] context   = context struct
 * @param[in] slave     = Slave number
 * @param[in] eeproma   = (WORD) Address in the EEPROM
 * @param[in] data      = 16bit data
 * @param[in] timeout   = Timeout in us.
 * @return >0 if OK
 */
int ecx_writeeeprom(ecx_contextt *context, uint16 slave, uint16 eeproma, uint16 data, int timeout)
{
   uint16 configadr;

   ecx_eeprom2master(context, slave); /* set eeprom control to master */
   configadr = context->slavelist[slave].configadr;
   return (ecx_writeeepromFP(context, configadr, eeproma, data, timeout));
}

/** Set eeprom control to master. Only if set to PDI.
 * @param[in] context   = context struct
 * @param[in] slave     = Slave number
 * @return >0 if OK
 */
int ecx_eeprom2master(ecx_contextt *context, uint16 slave)
{
   int wkc = 1, cnt = 0;
   uint16 configadr;
   uint8 eepctl;

   if ( context->slavelist[slave].eep_pdi )
   {
      configadr = context->slavelist[slave].configadr;
      eepctl = 2;
      do
      {
         wkc = ecx_FPWR(context->port, configadr, ECT_REG_EEPCFG, sizeof(eepctl), &eepctl , EC_TIMEOUTRET); /* force Eeprom from PDI */
      }
      while ((wkc <= 0) && (cnt++ < EC_DEFAULTRETRIES));
      eepctl = 0;
      cnt = 0;
      do
      {
         wkc = ecx_FPWR(context->port, configadr, ECT_REG_EEPCFG, sizeof(eepctl), &eepctl , EC_TIMEOUTRET); /* set Eeprom to master */
      }
      while ((wkc <= 0) && (cnt++ < EC_DEFAULTRETRIES));
      context->slavelist[slave].eep_pdi = 0;
   }

   return wkc;
}

/**
 * 将EEPROM控制权设置为PDI
 *
 * 该函数将从站的EEPROM控制权从主站转移到PDI（从站控制器）。
 * 只有当EEPROM当前由主站控制时才执行此操作。
 * 这是从站在状态转换过程中需要EEPROM访问权限时使用的。
 *
 * @param[in] context  EtherCAT上下文结构体
 * @param[in] slave    从站编号
 * @return 成功返回大于0的值，失败返回0或负值
 */
int ecx_eeprom2pdi(ecx_contextt *context, uint16 slave)
{
   int wkc = 1, cnt = 0;      // 工作计数器、重试计数器
   uint16 configadr;           // 配置地址
   uint8 eepctl;               // EEPROM控制寄存器值

   // 检查EEPROM是否当前不由PDI控制
   if ( !context->slavelist[slave].eep_pdi )
   {
      // 获取从站配置地址
      configadr = context->slavelist[slave].configadr;

      // 设置EEPROM控制寄存器值为1（PDI控制）
      eepctl = 1;

      // 写入EEPROM控制寄存器（带重试）
      do
      {
         wkc = ecx_FPWR(context->port, configadr, ECT_REG_EEPCFG, sizeof(eepctl), &eepctl , EC_TIMEOUTRET);
      }
      while ((wkc <= 0) && (cnt++ < EC_DEFAULTRETRIES));

      // 标记EEPROM现在由PDI控制
      context->slavelist[slave].eep_pdi = 1;
   }

   return wkc;
}

// APRD 0x0502 0x00 读取 EEPROM 状态，确认是否 busy
uint16 ecx_eeprom_waitnotbusyAP(ecx_contextt *context, uint16 aiadr,uint16 *estat, int timeout)
{
   int wkc, cnt = 0;
   uint16 retval = 0;
   osal_timert timer;

   osal_timer_start(&timer, timeout);
   do
   {
      if (cnt++)
      {
         osal_usleep(EC_LOCALDELAY);
      }
      *estat = 0;
      // APRD 0x0502 0x00 读取 EEPROM 状态，确认是否 busy
      wkc=ecx_APRD(context->port, aiadr, ECT_REG_EEPSTAT, sizeof(*estat), estat, EC_TIMEOUTRET);
      *estat = etohs(*estat);
   }
   while (((wkc <= 0) || ((*estat & EC_ESTAT_BUSY) > 0)) && (osal_timer_is_expired(&timer) == FALSE)); /* wait for eeprom ready */
   if ((*estat & EC_ESTAT_BUSY) == 0)
   {
      retval = 1;
   }

   return retval;
}

/** Read EEPROM from slave bypassing cache. APRD method.
 * @param[in] context     = context struct
 * @param[in] aiadr       = auto increment address of slave
 * @param[in] eeproma     = (WORD) Address in the EEPROM
 * @param[in] timeout     = Timeout in us.
 * @return EEPROM data 64bit or 32bit
 */
uint64 ecx_readeepromAP(ecx_contextt *context, uint16 aiadr, uint16 eeproma, int timeout)
{
   uint16 estat;
   uint32 edat32;
   uint64 edat64;
   ec_eepromt ed;
   int wkc, cnt, nackcnt = 0;

   edat64 = 0;
   edat32 = 0;
   if (ecx_eeprom_waitnotbusyAP(context, aiadr, &estat, timeout))
   {
      if (estat & EC_ESTAT_EMASK) /* error bits are set */
      {
         estat = htoes(EC_ECMD_NOP); /* clear error bits */
         // APWR 0x0502 0x00 清除错误位
         wkc = ecx_APWR(context->port, aiadr, ECT_REG_EEPCTL, sizeof(estat), &estat, EC_TIMEOUTRET3);
      }

      do
      {
         ed.comm = htoes(EC_ECMD_READ);
         ed.addr = htoes(eeproma);
         ed.d2   = 0x0000;
         cnt = 0;
         do
         {
            wkc = ecx_APWR(context->port, aiadr, ECT_REG_EEPCTL, sizeof(ed), &ed, EC_TIMEOUTRET);
         }
         while ((wkc <= 0) && (cnt++ < EC_DEFAULTRETRIES));
         if (wkc)
         {
            osal_usleep(EC_LOCALDELAY);
            estat = 0x0000;
            if (ecx_eeprom_waitnotbusyAP(context, aiadr, &estat, timeout))
            {
               if (estat & EC_ESTAT_NACK)
               {
                  nackcnt++;
                  osal_usleep(EC_LOCALDELAY * 5);
               }
               else
               {
                  nackcnt = 0;
                  if (estat & EC_ESTAT_R64)
                  {
                     cnt = 0;
                     do
                     {
                        wkc = ecx_APRD(context->port, aiadr, ECT_REG_EEPDAT, sizeof(edat64), &edat64, EC_TIMEOUTRET);
                     }
                     while ((wkc <= 0) && (cnt++ < EC_DEFAULTRETRIES));
                  }
                  else
                  {
                     cnt = 0;
                     do
                     {
                        wkc = ecx_APRD(context->port, aiadr, ECT_REG_EEPDAT, sizeof(edat32), &edat32, EC_TIMEOUTRET);
                     }
                     while ((wkc <= 0) && (cnt++ < EC_DEFAULTRETRIES));
                     edat64=(uint64)edat32;
                  }
               }
            }
         }
      }
      while ((nackcnt > 0) && (nackcnt < 3));
   }

   return edat64;
}

/** Write EEPROM to slave bypassing cache. APWR method.
 * @param[in] context   = context struct
 * @param[in] aiadr     = configured address of slave
 * @param[in] eeproma   = (WORD) Address in the EEPROM
 * @param[in] data      = 16bit data
 * @param[in] timeout   = Timeout in us.
 * @return >0 if OK
 */
int ecx_writeeepromAP(ecx_contextt *context, uint16 aiadr, uint16 eeproma, uint16 data, int timeout)
{
   uint16 estat;
   ec_eepromt ed;
   int wkc, rval = 0, cnt = 0, nackcnt = 0;

   if (ecx_eeprom_waitnotbusyAP(context, aiadr, &estat, timeout))
   {
      if (estat & EC_ESTAT_EMASK) /* error bits are set */
      {
         estat = htoes(EC_ECMD_NOP); /* clear error bits */
         wkc = ecx_APWR(context->port, aiadr, ECT_REG_EEPCTL, sizeof(estat), &estat, EC_TIMEOUTRET3);
      }
      do
      {
         cnt = 0;
         do
         {
            wkc = ecx_APWR(context->port, aiadr, ECT_REG_EEPDAT, sizeof(data), &data, EC_TIMEOUTRET);
         }
         while ((wkc <= 0) && (cnt++ < EC_DEFAULTRETRIES));

         ed.comm = EC_ECMD_WRITE;  // 设置 EEPROM command 为写入
         ed.addr = eeproma; // 设置 EEPROM 写入地址
         ed.d2 = 0x0000; // 设置 EEPROM 地址后半段为 0x0000;
         cnt = 0;
         do
         {
            wkc = ecx_APWR(context->port, aiadr, ECT_REG_EEPCTL, sizeof(ed), &ed, EC_TIMEOUTRET);
         }
         while ((wkc <= 0) && (cnt++ < EC_DEFAULTRETRIES));
         if (wkc)
         {
            osal_usleep(EC_LOCALDELAY * 2);
            estat = 0x0000;
            if (ecx_eeprom_waitnotbusyAP(context, aiadr, &estat, timeout))
            {
               if (estat & EC_ESTAT_NACK)
               {
                  nackcnt++;
                  osal_usleep(EC_LOCALDELAY * 5);
               }
               else
               {
                  nackcnt = 0;
                  rval = 1;
               }
            }
         }

      }
      while ((nackcnt > 0) && (nackcnt < 3));
   }

   return rval;
}

// FPRD 0x0502 0x00 读取 EEPROM 状态，确认是否 busy
uint16 ecx_eeprom_waitnotbusyFP(ecx_contextt *context, uint16 configadr,uint16 *estat, int timeout)
{
   int wkc, cnt = 0;
   uint16 retval = 0;
   osal_timert timer;

   osal_timer_start(&timer, timeout);
   do
   {
      if (cnt++)
      {
         osal_usleep(EC_LOCALDELAY);
      }
      *estat = 0;
      wkc=ecx_FPRD(context->port, configadr, ECT_REG_EEPSTAT, sizeof(*estat), estat, EC_TIMEOUTRET);
      *estat = etohs(*estat);
   }
   while (((wkc <= 0) || ((*estat & EC_ESTAT_BUSY) > 0)) && (osal_timer_is_expired(&timer) == FALSE)); /* wait for eeprom ready */
   if ((*estat & EC_ESTAT_BUSY) == 0)
   {
      retval = 1;
   }

   return retval;
}

/** Read EEPROM from slave bypassing cache. FPRD method.
 * @param[in] context     = context struct
 * @param[in] configadr   = configured address of slave
 * @param[in] eeproma     = (WORD) Address in the EEPROM
 * @param[in] timeout     = Timeout in us.
 * @return EEPROM data 64bit or 32bit
 */
uint64 ecx_readeepromFP(ecx_contextt *context, uint16 configadr, uint16 eeproma, int timeout)
{
   uint16 estat;
   uint32 edat32;
   uint64 edat64;
   ec_eepromt ed;
   int wkc, cnt, nackcnt = 0;

   edat64 = 0;
   edat32 = 0;
   if (ecx_eeprom_waitnotbusyFP(context, configadr, &estat, timeout))
   {
      if (estat & EC_ESTAT_EMASK) /* error bits are set */
      {
         estat = htoes(EC_ECMD_NOP); /* clear error bits */
         wkc=ecx_FPWR(context->port, configadr, ECT_REG_EEPCTL, sizeof(estat), &estat, EC_TIMEOUTRET3);
      }

      do
      {
         ed.comm = htoes(EC_ECMD_READ);
         ed.addr = htoes(eeproma);
         ed.d2   = 0x0000;
         cnt = 0;
         do
         {
            wkc=ecx_FPWR(context->port, configadr, ECT_REG_EEPCTL, sizeof(ed), &ed, EC_TIMEOUTRET);
         }
         while ((wkc <= 0) && (cnt++ < EC_DEFAULTRETRIES));
         if (wkc)
         {
            osal_usleep(EC_LOCALDELAY);
            estat = 0x0000;
            if (ecx_eeprom_waitnotbusyFP(context, configadr, &estat, timeout))
            {
               if (estat & EC_ESTAT_NACK)
               {
                  nackcnt++;
                  osal_usleep(EC_LOCALDELAY * 5);
               }
               else
               {
                  nackcnt = 0;
                  if (estat & EC_ESTAT_R64)
                  {
                     cnt = 0;
                     do
                     {
                        wkc=ecx_FPRD(context->port, configadr, ECT_REG_EEPDAT, sizeof(edat64), &edat64, EC_TIMEOUTRET);
                     }
                     while ((wkc <= 0) && (cnt++ < EC_DEFAULTRETRIES));
                  }
                  else
                  {
                     cnt = 0;
                     do
                     {
                        wkc=ecx_FPRD(context->port, configadr, ECT_REG_EEPDAT, sizeof(edat32), &edat32, EC_TIMEOUTRET);
                     }
                     while ((wkc <= 0) && (cnt++ < EC_DEFAULTRETRIES));
                     edat64=(uint64)edat32;
                  }
               }
            }
         }
      }
      while ((nackcnt > 0) && (nackcnt < 3));
   }

   return edat64;
}

/** Write EEPROM to slave bypassing cache. FPWR method.
 * @param[in]  context        = context struct
 * @param[in] configadr   = configured address of slave
 * @param[in] eeproma     = (WORD) Address in the EEPROM
 * @param[in] data        = 16bit data
 * @param[in] timeout     = Timeout in us.
 * @return >0 if OK
 */
int ecx_writeeepromFP(ecx_contextt *context, uint16 configadr, uint16 eeproma, uint16 data, int timeout)
{
   uint16 estat;
   ec_eepromt ed;
   int wkc, rval = 0, cnt = 0, nackcnt = 0;

   if (ecx_eeprom_waitnotbusyFP(context, configadr, &estat, timeout))
   {
      if (estat & EC_ESTAT_EMASK) /* error bits are set */
      {
         estat = htoes(EC_ECMD_NOP); /* clear error bits */
         wkc = ecx_FPWR(context->port, configadr, ECT_REG_EEPCTL, sizeof(estat), &estat, EC_TIMEOUTRET3);
      }
      do
      {
         cnt = 0;
         do
         {
            wkc = ecx_FPWR(context->port, configadr, ECT_REG_EEPDAT, sizeof(data), &data, EC_TIMEOUTRET);
         }
         while ((wkc <= 0) && (cnt++ < EC_DEFAULTRETRIES));
         ed.comm = EC_ECMD_WRITE;
         ed.addr = eeproma;
         ed.d2   = 0x0000;
         cnt = 0;
         do
         {
            wkc = ecx_FPWR(context->port, configadr, ECT_REG_EEPCTL, sizeof(ed), &ed, EC_TIMEOUTRET);
         }
         while ((wkc <= 0) && (cnt++ < EC_DEFAULTRETRIES));
         if (wkc)
         {
            osal_usleep(EC_LOCALDELAY * 2);
            estat = 0x0000;
            if (ecx_eeprom_waitnotbusyFP(context, configadr, &estat, timeout))
            {
               if (estat & EC_ESTAT_NACK)
               {
                  nackcnt++;
                  osal_usleep(EC_LOCALDELAY * 5);
               }
               else
               {
                  nackcnt = 0;
                  rval = 1;
               }
            }
         }
      }
      while ((nackcnt > 0) && (nackcnt < 3));
   }

   return rval;
}

/**
 * 从从站读取EEPROM数据（绕过缓存）
 * 并行读取步骤1：向从站发送读取请求
 *
 * 该函数向从站发送EEPROM读取请求，但不等待数据返回。
 * 这是一个两步读取过程的第一步，用于并行读取多个从站的EEPROM。
 *
 * @param[in] context     EtherCAT上下文结构体
 * @param[in] slave       从站编号
 * @param[in] eeproma     EEPROM地址（字地址）
 */
void ecx_readeeprom1(ecx_contextt *context, uint16 slave, uint16 eeproma)
{
   uint16 configadr, estat;    // 配置地址、EEPROM状态
   ec_eepromt ed;              // EEPROM命令数据结构
   int wkc, cnt = 0;           // 工作计数器、重试计数

   // 将EEPROM控制权交给主站
   ecx_eeprom2master(context, slave);

   // 获取从站配置地址
   configadr = context->slavelist[slave].configadr;

   // 等待EEPROM不忙
   if (ecx_eeprom_waitnotbusyFP(context, configadr, &estat, EC_TIMEOUTEEP))
   {
      // 检查错误位是否设置
      if (estat & EC_ESTAT_EMASK)
      {
         // 清除错误位
         estat = htoes(EC_ECMD_NOP);
         wkc = ecx_FPWR(context->port, configadr, ECT_REG_EEPCTL, sizeof(estat), &estat, EC_TIMEOUTRET3);
      }

      // 设置读取命令
      ed.comm = htoes(EC_ECMD_READ);   // 读取命令
      ed.addr = htoes(eeproma);        // EEPROM地址
      ed.d2   = 0x0000;                // 保留字段

      // 发送读取请求（带重试）
      do
      {
         wkc = ecx_FPWR(context->port, configadr, ECT_REG_EEPCTL, sizeof(ed), &ed, EC_TIMEOUTRET);
      }
      while ((wkc <= 0) && (cnt++ < EC_DEFAULTRETRIES));
   }
}

/**
 * 从从站读取EEPROM数据（绕过缓存）
 * 并行读取步骤2：实际从从站读取数据
 *
 * 该函数从从站读取EEPROM数据，这是两步读取过程的第二步。
 * 必须在调用ecx_readeeprom1之后调用此函数。
 *
 * @param[in] context     EtherCAT上下文结构体
 * @param[in] slave       从站编号
 * @param[in] timeout     超时时间（微秒）
 * @return EEPROM数据（32位）
 */
uint32 ecx_readeeprom2(ecx_contextt *context, uint16 slave, int timeout)
{
   uint16 estat, configadr;    // EEPROM状态、配置地址
   uint32 edat;                // EEPROM数据
   int wkc, cnt = 0;           // 工作计数器、重试计数

   // 获取从站配置地址
   configadr = context->slavelist[slave].configadr;

   // 初始化数据
   edat = 0;
   estat = 0x0000;

   // 等待EEPROM不忙
   if (ecx_eeprom_waitnotbusyFP(context, configadr, &estat, timeout))
   {
      // 读取EEPROM数据（带重试）
      do
      {
          wkc = ecx_FPRD(context->port, configadr, ECT_REG_EEPDAT, sizeof(edat), &edat, EC_TIMEOUTRET);
      }
      while ((wkc <= 0) && (cnt++ < EC_DEFAULTRETRIES));
   }

   return edat;
}

/** Push index of segmented LRD/LWR/LRW combination.
 * @param[in]  context        = context struct
 * @param[in] idx         = Used datagram index.
 * @param[in] data        = Pointer to process data segment.
 * @param[in] length      = Length of data segment in bytes.
 * @param[in] DCO         = Offset position of DC frame.
 */
static void ecx_pushindex(ecx_contextt *context, uint8 idx, void *data, uint16 length, uint16 DCO)
{
   if(context->idxstack->pushed < EC_MAXBUF)
   {
      context->idxstack->idx[context->idxstack->pushed] = idx;
      context->idxstack->data[context->idxstack->pushed] = data;
      context->idxstack->length[context->idxstack->pushed] = length;
      context->idxstack->dcoffset[context->idxstack->pushed] = DCO;
      context->idxstack->pushed++;
   }
}

/** Pull index of segmented LRD/LWR/LRW combination.
 * @param[in]  context        = context struct
 * @return Stack location, -1 if stack is empty.
 */
static int ecx_pullindex(ecx_contextt *context)
{
   int rval = -1;
   if(context->idxstack->pulled < context->idxstack->pushed)
   {
      rval = context->idxstack->pulled;
      context->idxstack->pulled++;
   }

   return rval;
}

/**
 * Clear the idx stack.
 *
 * @param context           = context struct
 */
static void ecx_clearindex(ecx_contextt *context)  {

   context->idxstack->pushed = 0;
   context->idxstack->pulled = 0;

}

/** 向从站发送过程数据。
 * 使用LRW命令，如果LRW不被允许（blockLRW）则使用LRD/LWR。
 * 输入和输出过程数据都被发送。输出数据为实际数据，输入数据为占位符。
 * 输入数据通过接收过程数据函数收集。
 * 与基础LRW函数不同，此函数是非阻塞的。
 * 如果过程数据无法放入一个数据报中，则使用多个数据报。
 * 为了重新组合从站响应，使用了一个栈。
 *
 * @param[in]  context        = 上下文结构体
 * @param[in]  group          = 组号
 * @param[in]  use_overlap_io = 是否使用重叠IO映射的标志
 * @return >0 表示过程数据已发送
 */
/**
 * 向EtherCAT从站发送过程数据的核心函数
 *
 * 该函数是EtherCAT主站发送过程数据的核心实现，负责将输出数据发送到从站
 * 并准备接收输入数据。支持多种传输模式和优化策略。
 *
 * 主要功能：
 * 1. 支持重叠IO映射（Overlapping IO Map）模式，输入输出共享内存区域
 * 2. 支持分布式时钟（DC）同步，通过FRMW命令实现精确时间同步
 * 3. 根据从站能力选择最优传输命令（LRW/LRD/LWR）
 * 4. 支持分段传输，处理大数据量的过程数据
 * 5. 使用栈机制管理多个数据报的发送和接收
 *
 * 传输命令选择策略：
 * - LRW（逻辑读写）：同时读写，最高效，一个命令完成输入输出
 * - LRD（逻辑读）：仅读取输入数据
 * - LWR（逻辑写）：仅写入输出数据
 * - 当从站不支持LRW时（blockLRW=TRUE），使用LRD+LWR组合
 *
 * 分布式时钟同步：
 * - 如果组内有DC从站，在第一个数据报中附加FRMW命令
 * - FRMW命令读取参考时钟从站的系统时间并广播到所有从站
 * - 实现纳秒级的时间同步精度
 *
 * @param[in] context         EtherCAT上下文结构体
 * @param[in] group           组号，指定要发送数据的从站组
 * @param[in] use_overlap_io  是否使用重叠IO映射模式
 *                            TRUE: 输入输出共享内存，节省内存空间
 *                            FALSE: 输入输出分离，传统模式
 * @return 工作计数器，>0表示成功发送，0表示无数据发送
 */
static int ecx_main_send_processdata(ecx_contextt *context, uint8 group, boolean use_overlap_io)
{
   uint32 LogAdr;              // 逻辑地址，用于寻址从站的FMMU映射区域
   uint16 w1, w2;              // 逻辑地址的低16位和高16位
   int length;                 // 总数据长度
   uint16 sublength;           // 分段传输时的子长度
   uint8 idx;                  // 数据报索引
   int wkc;                    // 工作计数器
   uint8* data;                // 数据指针
   // 用于决定是否需要发送FRMW命令（分布式时钟同步）
   boolean first=FALSE;
   uint16 currentsegment = 0;  // 当前段索引
   uint32 iomapinputoffset;    // 重叠IO映射时的输入偏移量
   uint16 DCO;                 // DC偏移量，用于定位FRMW命令在数据报中的位置

   wkc = 0;
   // 检查该组是否有分布式时钟从站
   if(context->grouplist[group].hasdc)
   {
      first = TRUE;  // 标记需要在第一个数据报中添加FRMW命令
   }

   /* For overlapping IO map use the biggest */
   // 根据IO映射模式计算数据长度
   if(use_overlap_io == TRUE)
   {
      /* For overlap IOmap make the frame EQ big to biggest part */
      // 重叠模式下，帧大小取输入输出的最大值
      length = (context->grouplist[group].Obytes > context->grouplist[group].Ibytes) ?
         context->grouplist[group].Obytes : context->grouplist[group].Ibytes;
      /* Save the offset used to compensate where to save inputs when frame returns */
      // 保存偏移量，用于在帧返回时确定输入数据的存储位置
      iomapinputoffset = context->grouplist[group].Obytes;
   }
   else
   {
      // 非重叠模式，总长度为输入输出之和
      length = context->grouplist[group].Obytes + context->grouplist[group].Ibytes;
      iomapinputoffset = 0;
   }

   // 获取组的起始逻辑地址
   LogAdr = context->grouplist[group].logstartaddr;
   if(length)
   {

      wkc = 1;
      /* LRW blocked by one or more slaves ? */
      // 检查是否有从站阻止使用LRW命令
      if(context->grouplist[group].blockLRW)
      {
         /* if inputs available generate LRD */
         // 如果有输入数据，使用LRD命令读取
         if(context->grouplist[group].Ibytes)
         {
            currentsegment = context->grouplist[group].Isegment;
            data = context->grouplist[group].inputs;
            length = context->grouplist[group].Ibytes;
            LogAdr += context->grouplist[group].Obytes;  // 跳过输出区域
            /* segment transfer if needed */
            // 分段传输循环
            do
            {
               // 计算当前段的长度
               if(currentsegment == context->grouplist[group].Isegment)
               {
                  // 第一段需要考虑偏移量
                  sublength = (uint16)(context->grouplist[group].IOsegment[currentsegment++] - context->grouplist[group].Ioffset);
               }
               else
               {
                  sublength = (uint16)context->grouplist[group].IOsegment[currentsegment++];
               }
               /* get new index */
               idx = ecx_getindex(context->port);
               w1 = LO_WORD(LogAdr);
               w2 = HI_WORD(LogAdr);
               DCO = 0;
               // 创建LRD（逻辑读）数据报
               ecx_setupdatagram(context->port, &(context->port->txbuf[idx]), EC_CMD_LRD, idx, w1, w2, sublength, data);
               if(first)
               {
                 /* FPRMW in second datagram */
                  // FRMW 0x910 触发从站时间同步
                  // 在数据报中添加FRMW命令，用于分布式时钟同步
                  DCO = ecx_adddatagram(context->port, &(context->port->txbuf[idx]), EC_CMD_FRMW, idx, FALSE,
                                           context->slavelist[context->grouplist[group].DCnext].configadr,
                                           ECT_REG_DCSYSTIME, sizeof(int64), context->DCtime);
                  first = FALSE;
               }
               /* send frame */
               // 发送帧（支持冗余）
               ecx_outframe_red(context->port, idx);
               /* push index and data pointer on stack */
               // 将索引和数据指针压栈，用于后续接收处理
               ecx_pushindex(context, idx, data, sublength, DCO);
               length -= sublength;
               LogAdr += sublength;
               data += sublength;
            } while (length && (currentsegment < context->grouplist[group].nsegments));
         }
         /* if outputs available generate LWR */
         // 如果有输出数据，使用LWR命令写入
         if(context->grouplist[group].Obytes)
         {
            data = context->grouplist[group].outputs;
            length = context->grouplist[group].Obytes;
            LogAdr = context->grouplist[group].logstartaddr;
            currentsegment = 0;
            /* segment transfer if needed */
            // 分段传输循环
            do
            {
               sublength = (uint16)context->grouplist[group].IOsegment[currentsegment++];
               if((length - sublength) < 0)
               {
                  sublength = (uint16)length;
               }
               /* get new index */
               idx = ecx_getindex(context->port);
               w1 = LO_WORD(LogAdr);
               w2 = HI_WORD(LogAdr);
               DCO = 0;
               // 创建LWR（逻辑写）数据报
               ecx_setupdatagram(context->port, &(context->port->txbuf[idx]), EC_CMD_LWR, idx, w1, w2, sublength, data);
               if(first)
               {
                  /* FPRMW in second datagram */
                  // 在数据报中添加FRMW命令
                  DCO = ecx_adddatagram(context->port, &(context->port->txbuf[idx]), EC_CMD_FRMW, idx, FALSE,
                                           context->slavelist[context->grouplist[group].DCnext].configadr,
                                           ECT_REG_DCSYSTIME, sizeof(int64), context->DCtime);
                  first = FALSE;
               }
               /* send frame */
               ecx_outframe_red(context->port, idx);
               /* push index and data pointer on stack */
               ecx_pushindex(context, idx, data, sublength, DCO);
               length -= sublength;
               LogAdr += sublength;
               data += sublength;
            } while (length && (currentsegment < context->grouplist[group].nsegments));
         }
      }
      /* LRW can be used */
      // 可以使用LRW命令（最高效的方式）
      else
      {
         // 确定数据指针：优先使用输出数据
         if (context->grouplist[group].Obytes)
         {
            data = context->grouplist[group].outputs;
         }
         else
         {
            data = context->grouplist[group].inputs;
            /* Clear offset, don't compensate for overlapping IOmap if we only got inputs */
            // 如果只有输入，清除偏移量
            iomapinputoffset = 0;
         }
         /* segment transfer if needed */
         // 分段传输循环
         do
         {
            sublength = (uint16)context->grouplist[group].IOsegment[currentsegment++];
            /* get new index */
            idx = ecx_getindex(context->port);
            w1 = LO_WORD(LogAdr);
            w2 = HI_WORD(LogAdr);
            DCO = 0;
            // 创建LRW（逻辑读写）数据报
            // LRW命令在一个帧中同时完成读和写操作，效率最高
            ecx_setupdatagram(context->port, &(context->port->txbuf[idx]), EC_CMD_LRW, idx, w1, w2, sublength, data);
            if(first)
            {
              /* FPRMW in second datagram */
               // FRMW 0x910 触发从站时间同步
               DCO = ecx_adddatagram(context->port, &(context->port->txbuf[idx]), EC_CMD_FRMW, idx, FALSE,
                                        context->slavelist[context->grouplist[group].DCnext].configadr,
                                        ECT_REG_DCSYSTIME, sizeof(int64), context->DCtime);
               first = FALSE;
            }
            /* send frame */
            ecx_outframe_red(context->port, idx);
            /* push index and data pointer on stack.
             * the iomapinputoffset compensate for where the inputs are stored
             * in the IOmap if we use an overlapping IOmap. If a regular IOmap
             * is used it should always be 0.
             */
            // 压栈时考虑重叠IO映射的偏移量
            ecx_pushindex(context, idx, (data + iomapinputoffset), sublength, DCO);
            length -= sublength;
            LogAdr += sublength;
            data += sublength;
         } while (length && (currentsegment < context->grouplist[group].nsegments));
      }
   }

   return wkc;
}

/** 向从站发送过程数据（重叠IO映射版本）。
 * 使用LRW命令，如果LRW不被允许（blockLRW）则使用LRD/LWR。
 * 输入和输出过程数据都在重叠的IOmap中发送。
 * 输出数据为实际数据，输入数据在返回帧中替换输出数据。
 * 输入数据通过接收过程数据函数收集。
 * 与基础LRW函数不同，此函数是非阻塞的。
 * 如果过程数据无法放入一个数据报中，则使用多个数据报。
 * 为了重新组合从站响应，使用了一个栈。
 *
 * @param[in]  context        = 上下文结构体
 * @param[in]  group          = 组号
 * @return >0 表示过程数据已发送
 */
int ecx_send_overlap_processdata_group(ecx_contextt *context, uint8 group)
{
   return ecx_main_send_processdata(context, group, TRUE);
}

/** 向从站发送过程数据。
 * 使用LRW命令，如果LRW不被允许（blockLRW）则使用LRD/LWR。
 * 输入和输出过程数据都被发送。输出数据为实际数据，输入数据为占位符。
 * 输入数据通过接收过程数据函数收集。
 * 与基础LRW函数不同，此函数是非阻塞的。
 * 如果过程数据无法放入一个数据报中，则使用多个数据报。
 * 为了重新组合从站响应，使用了一个栈。
 *
 * @param[in]  context        = 上下文结构体
 * @param[in]  group          = 组号
 * @return >0 表示过程数据已发送
 */
int ecx_send_processdata_group(ecx_contextt *context, uint8 group)
{
   return ecx_main_send_processdata(context, group, FALSE);
}

/** 从从站接收过程数据。
 * ec_send_processdata()的第二部分。
 * 接收到的数据报借助栈与过程数据重新组合。
 * 如果数据报包含输入过程数据，则将其复制到过程数据结构中。
 *
 * @param[in]  context        = 上下文结构体
 * @param[in]  group          = 组号
 * @param[in]  timeout        = 超时时间（微秒）
 * @return 工作计数器值
 */
int ecx_receive_processdata_group(ecx_contextt *context, uint8 group, int timeout)
{
   uint8 idx;
   int pos;
   int wkc = 0, wkc2;
   uint16 le_wkc = 0;
   int valid_wkc = 0;
   int64 le_DCtime;
   ec_idxstackT *idxstack;
   ec_bufT *rxbuf;

   /* just to prevent compiler warning for unused group */
   wkc2 = group;

   idxstack = context->idxstack;
   rxbuf = context->port->rxbuf;
   /* get first index */
   pos = ecx_pullindex(context);
   /* read the same number of frames as send */
   while (pos >= 0)
   {
      idx = idxstack->idx[pos];
      wkc2 = ecx_waitinframe(context->port, idx, timeout);
      /* check if there is input data in frame */
      if (wkc2 > EC_NOFRAME)
      {
         if((rxbuf[idx][EC_CMDOFFSET]==EC_CMD_LRD) || (rxbuf[idx][EC_CMDOFFSET]==EC_CMD_LRW))
         {
            if(idxstack->dcoffset[pos] > 0)
            {
               memcpy(idxstack->data[pos], &(rxbuf[idx][EC_HEADERSIZE]), idxstack->length[pos]);
               memcpy(&le_wkc, &(rxbuf[idx][EC_HEADERSIZE + idxstack->length[pos]]), EC_WKCSIZE);
               wkc = etohs(le_wkc);
               // 获得参考时钟时间
               memcpy(&le_DCtime, &(rxbuf[idx][idxstack->dcoffset[pos]]), sizeof(le_DCtime));
               *(context->DCtime) = etohll(le_DCtime);
            }
            else
            {
               /* copy input data back to process data buffer */
               memcpy(idxstack->data[pos], &(rxbuf[idx][EC_HEADERSIZE]), idxstack->length[pos]);
               wkc += wkc2;
            }
            valid_wkc = 1;
         }
         else if(rxbuf[idx][EC_CMDOFFSET]==EC_CMD_LWR)
         {
            if(idxstack->dcoffset[pos] > 0)
            {
               memcpy(&le_wkc, &(rxbuf[idx][EC_HEADERSIZE + idxstack->length[pos]]), EC_WKCSIZE);
               /* output WKC counts 2 times when using LRW, emulate the same for LWR */
               wkc = etohs(le_wkc) * 2;
               // 获得参考时钟时间
               memcpy(&le_DCtime, &(rxbuf[idx][idxstack->dcoffset[pos]]), sizeof(le_DCtime));
               *(context->DCtime) = etohll(le_DCtime);
            }
            else
            {
               /* output WKC counts 2 times when using LRW, emulate the same for LWR */
               wkc += wkc2 * 2;
            }
            valid_wkc = 1;
         }
      }
      /* release buffer */
      ecx_setbufstat(context->port, idx, EC_BUF_EMPTY);
      /* get next index */
      pos = ecx_pullindex(context);
   }

   ecx_clearindex(context);

   /* if no frames has arrived */
   if (valid_wkc == 0)
   {
      return EC_NOFRAME;
   }
   return wkc;
}

/** 发送过程数据（默认组0）。
 *
 * 这是ecx_send_processdata_group的便捷封装，
 * 使用默认组0（所有从站）。
 *
 * @param[in]  context = 上下文结构体
 * @return >0: 过程数据已发送
 */
int ecx_send_processdata(ecx_contextt *context)
{
   return ecx_send_processdata_group(context, 0);
}

/** 发送重叠过程数据（默认组0）。
 *
 * 这是ecx_send_overlap_processdata_group的便捷封装，
 * 使用默认组0（所有从站）。
 *
 * @param[in]  context = 上下文结构体
 * @return >0: 过程数据已发送
 */
int ecx_send_overlap_processdata(ecx_contextt *context)
{
   return ecx_send_overlap_processdata_group(context, 0);
}

/** 接收过程数据（默认组0）。
 *
 * 这是ecx_receive_processdata_group的便捷封装，
 * 使用默认组0（所有从站）。
 *
 * @param[in]  context = 上下文结构体
 * @param[in]  timeout = 超时时间（微秒）
 * @return 工作计数器值
 */
int ecx_receive_processdata(ecx_contextt *context, int timeout)
{
   return ecx_receive_processdata_group(context, 0, timeout);
}

#ifdef EC_VER1
/** EC_VER1兼容函数：将错误压入错误列表。
 * @param[in] Ec = 错误结构体指针
 */
void ec_pusherror(const ec_errort *Ec)
{
   ecx_pusherror(&ecx_context, Ec);
}

/** EC_VER1兼容函数：从错误列表弹出错误。
 * @param[out] Ec = 错误结构体指针
 * @return TRUE: 成功；FALSE: 列表为空
 */
boolean ec_poperror(ec_errort *Ec)
{
   return ecx_poperror(&ecx_context, Ec);
}

/** EC_VER1兼容函数：检查错误列表是否有条目。
 * @return TRUE: 有错误；FALSE: 无错误
 */
boolean ec_iserror(void)
{
   return ecx_iserror(&ecx_context);
}

/** EC_VER1兼容函数：报告数据包错误。
 * @param[in] Slave     = 从站号
 * @param[in] Index     = 索引
 * @param[in] SubIdx    = 子索引
 * @param[in] ErrorCode = 错误码
 */
void ec_packeterror(uint16 Slave, uint16 Index, uint8 SubIdx, uint16 ErrorCode)
{
   ecx_packeterror(&ecx_context, Slave, Index, SubIdx, ErrorCode);
}

/** 初始化库（单网卡模式）- EC_VER1兼容函数。
 * @param[in] ifname = 网络接口名称，例如 "eth0"
 * @return >0: 成功；<=0: 失败
 * @see ecx_init
 */
int ec_init(const char * ifname)
{
   return ecx_init(&ecx_context, ifname);
}

/** 初始化库（冗余网卡模式）- EC_VER1兼容函数。
 * @param[in]  ifname  = 主网卡名称，例如 "eth0"
 * @param[in]  if2name = 备用网卡名称，例如 "eth1"
 * @return >0: 成功；<=0: 失败
 * @see ecx_init_redundant
 */
int ec_init_redundant(const char *ifname, char *if2name)
{
   return ecx_init_redundant (&ecx_context, &ecx_redport, ifname, if2name);
}

/** 关闭库 - EC_VER1兼容函数。
 * @see ecx_close
 */
void ec_close(void)
{
   ecx_close(&ecx_context);
};

/** 从从站EEPROM缓存读取一个字节 - EC_VER1兼容函数。
 *  如果缓存位置为空，则向从站发送读取请求。
 *  根据从站能力，请求可以是4或8字节。
 *  @param[in] slave   = 从站号
 *  @param[in] address = EEPROM字节地址（从站使用字地址）
 *  @return 请求的字节，如果不可用则返回0xff
 * @see ecx_siigetbyte
 */
uint8 ec_siigetbyte(uint16 slave, uint16 address)
{
   return ecx_siigetbyte (&ecx_context, slave, address);
}

/** 在从站EEPROM中查找SII段头部 - EC_VER1兼容函数。
 *  @param[in] slave = 从站号
 *  @param[in] cat   = 段类别
 *  @return 段长度条目处的字节地址，如果不可用则返回0
 *  @see ecx_siifind
 */
int16 ec_siifind(uint16 slave, uint16 cat)
{
   return ecx_siifind (&ecx_context, slave, cat);
}

/** 从从站EEPROM的SII字符串段获取字符串 - EC_VER1兼容函数。
 *  @param[out] str   = 请求的字符串，如果未找到则为0x00
 *  @param[in]  slave = 从站号
 *  @param[in]  Sn    = 字符串编号
 *  @see ecx_siistring
 */
void ec_siistring(char *str, uint16 slave, uint16 Sn)
{
   ecx_siistring(&ecx_context, str, slave, Sn);
}

/** 从从站EEPROM的SII FMMU段获取FMMU数据 - EC_VER1兼容函数。
 *  @param[in]  slave = 从站号
 *  @param[out] FMMU  = 从SII获取的FMMU结构体，最多4个FMMU
 *  @return 段中定义的FMMU数量
 *  @see ecx_siiFMMU
 */
uint16 ec_siiFMMU(uint16 slave, ec_eepromFMMUt* FMMU)
{
   return ecx_siiFMMU (&ecx_context, slave, FMMU);
}

/** 从从站EEPROM的SII SM段获取SM数据 - EC_VER1兼容函数。
 *  @param[in]  slave = 从站号
 *  @param[out] SM    = 从SII获取的第一个SM结构体
 *  @return 段中定义的SM数量
 *  @see ecx_siiSM
 */
uint16 ec_siiSM(uint16 slave, ec_eepromSMt* SM)
{
   return ecx_siiSM (&ecx_context, slave, SM);
}

/** 从从站EEPROM的SII SM段获取下一个SM数据 - EC_VER1兼容函数。
 *  @param[in]  slave = 从站号
 *  @param[out] SM    = SM结构体
 *  @param[in]  n     = SM编号
 *  @return >0: 成功
 *  @see ecx_siiSMnext
 */
uint16 ec_siiSMnext(uint16 slave, ec_eepromSMt* SM, uint16 n)
{
   return ecx_siiSMnext (&ecx_context, slave, SM, n);
}

/** 从从站EEPROM的SII PDO段获取PDO数据 - EC_VER1兼容函数。
 *  @param[in]  slave = 从站号
 *  @param[out] PDO   = 从SII获取的PDO结构体
 *  @param[in]  t      = 0: RX PDO；1: TX PDO
 *  @return PDO的映射大小（位）
 *  @see ecx_siiPDO
 */
uint32 ec_siiPDO(uint16 slave, ec_eepromPDOt* PDO, uint8 t)
{
   return ecx_siiPDO (&ecx_context, slave, PDO, t);
}

/** 读取所有从站的AL状态 - EC_VER1兼容函数。
 * @warning BOOT状态实际上比INIT和PRE_OP更高（参见状态表示）。
 * @return 找到的最低状态值
 * @see ecx_readstate
 */
int ec_readstate(void)
{
   return ecx_readstate (&ecx_context);
}

/** 写入从站AL状态 - EC_VER1兼容函数。
 * 如果slave = 0，则写入所有从站。
 * 函数不检查实际状态是否已改变。
 * @param[in] slave = 从站号，0 = 所有从站
 * @return 0
 * @see ecx_writestate
 */
int ec_writestate(uint16 slave)
{
   return ecx_writestate(&ecx_context, slave);
}

/** 检查从站的实际状态 - EC_VER1兼容函数。
 * 这是一个阻塞函数。
 * 要刷新所有从站的状态，应该调用ec_readstate()函数。
 * @warning 如果用于从站0（=所有从站），所有从站的状态通过按位OR操作读取。
 * 返回值也是所有从站状态的按位OR值。
 * 这对BOOT状态有一些影响。BOOT状态表示与INIT | PRE_OP冲突，
 * 因此此函数不能用于slave = 0且reqstate = EC_STATE_BOOT的情况。
 * @param[in] slave    = 从站号，0 = 所有从站
 * @param[in] reqstate = 请求的状态
 * @param[in] timeout  = 超时值（微秒）
 * @return 请求的状态，或超时后找到的状态
 * @see ecx_statecheck
 */
uint16 ec_statecheck(uint16 slave, uint16 reqstate, int timeout)
{
   return ecx_statecheck (&ecx_context, slave, reqstate, timeout);
}

/** 检查从站输入邮箱是否为空 - EC_VER1兼容函数。
 * @param[in] slave   = 从站号
 * @param[in] timeout = 超时时间（微秒）
 * @return >0: 成功
 * @see ecx_mbxempty
 */
int ec_mbxempty(uint16 slave, int timeout)
{
   return ecx_mbxempty (&ecx_context, slave, timeout);
}

/** 向从站写入输入邮箱数据 - EC_VER1兼容函数。
 * @param[in]  slave   = 从站号
 * @param[out] mbx     = 邮箱数据
 * @param[in]  timeout = 超时时间（微秒）
 * @return 工作计数器（>0表示成功）
 * @see ecx_mbxsend
 */
int ec_mbxsend(uint16 slave,ec_mbxbuft *mbx, int timeout)
{
   return ecx_mbxsend (&ecx_context, slave, mbx, timeout);
}

/** 从从站读取输出邮箱数据 - EC_VER1兼容函数。
 * 支持邮箱链路层的重复请求机制。
 * @param[in]  slave   = 从站号
 * @param[out] mbx     = 邮箱数据
 * @param[in]  timeout = 超时时间（微秒）
 * @return 工作计数器（>0表示成功）
 * @see ecx_mbxreceive
 */
int ec_mbxreceive(uint16 slave, ec_mbxbuft *mbx, int timeout)
{
   return ecx_mbxreceive (&ecx_context, slave, mbx, timeout);
}

/** 从从站转储完整的EEPROM数据到缓冲区 - EC_VER1兼容函数。
 * @param[in]  slave  = 从站号
 * @param[out] esibuf = EEPROM数据缓冲区，确保足够大
 * @see ecx_esidump
 */
void ec_esidump(uint16 slave, uint8 *esibuf)
{
   ecx_esidump (&ecx_context, slave, esibuf);
}

/** 绕过缓存从从站读取EEPROM - EC_VER1兼容函数。
 * @param[in] slave   = 从站号
 * @param[in] eeproma = EEPROM地址（字）
 * @param[in] timeout = 超时时间（微秒）
 * @return EEPROM数据（32位）
 * @see ecx_readeeprom
 */
uint32 ec_readeeprom(uint16 slave, uint16 eeproma, int timeout)
{
   return ecx_readeeprom (&ecx_context, slave, eeproma, timeout);
}

/** 绕过缓存向从站写入EEPROM - EC_VER1兼容函数。
 * @param[in] slave   = 从站号
 * @param[in] eeproma = EEPROM地址（字）
 * @param[in] data    = 16位数据
 * @param[in] timeout = 超时时间（微秒）
 * @return >0: 成功
 * @see ecx_writeeeprom
 */
int ec_writeeeprom(uint16 slave, uint16 eeproma, uint16 data, int timeout)
{
   return ecx_writeeeprom (&ecx_context, slave, eeproma, data, timeout);
}

/** 将EEPROM控制权交给主站 - EC_VER1兼容函数。
 * 仅当之前设置为PDI控制时有效。
 * @param[in] slave = 从站号
 * @return >0: 成功
 * @see ecx_eeprom2master
 */
int ec_eeprom2master(uint16 slave)
{
   return ecx_eeprom2master(&ecx_context, slave);
}

/** 将EEPROM控制权交给PDI - EC_VER1兼容函数。
 * @param[in] slave = 从站号
 * @return >0: 成功
 * @see ecx_eeprom2pdi
 */
int ec_eeprom2pdi(uint16 slave)
{
   return ecx_eeprom2pdi(&ecx_context, slave);
}

/** 等待EEPROM不忙（自动增量地址） - EC_VER1兼容函数。
 * @param[in] aiadr   = 从站自动增量地址
 * @param[out] estat  = EEPROM状态
 * @param[in] timeout = 超时时间
 * @return EEPROM状态
 */
uint16 ec_eeprom_waitnotbusyAP(uint16 aiadr,uint16 *estat, int timeout)
{
   return ecx_eeprom_waitnotbusyAP (&ecx_context, aiadr, estat, timeout);
}

/** 绕过缓存从从站读取EEPROM（APRD方法） - EC_VER1兼容函数。
 * 使用自动增量地址读取。
 * @param[in] aiadr   = 从站自动增量地址
 * @param[in] eeproma = EEPROM地址（字）
 * @param[in] timeout = 超时时间（微秒）
 * @return EEPROM数据（64位或32位）
 */
uint64 ec_readeepromAP(uint16 aiadr, uint16 eeproma, int timeout)
{
   return ecx_readeepromAP (&ecx_context, aiadr, eeproma, timeout);
}

/** 绕过缓存向从站写入EEPROM（APWR方法） - EC_VER1兼容函数。
 * 使用自动增量地址写入。
 * @param[in] aiadr   = 从站自动增量地址
 * @param[in] eeproma = EEPROM地址（字）
 * @param[in] data    = 16位数据
 * @param[in] timeout = 超时时间（微秒）
 * @return >0: 成功
 * @see ecx_writeeepromAP
 */
int ec_writeeepromAP(uint16 aiadr, uint16 eeproma, uint16 data, int timeout)
{
   return ecx_writeeepromAP (&ecx_context, aiadr, eeproma, data, timeout);
}

/** 等待EEPROM不忙（配置地址） - EC_VER1兼容函数。
 * @param[in] configadr = 从站配置地址
 * @param[out] estat    = EEPROM状态
 * @param[in] timeout   = 超时时间
 * @return EEPROM状态
 */
uint16 ec_eeprom_waitnotbusyFP(uint16 configadr,uint16 *estat, int timeout)
{
   return ecx_eeprom_waitnotbusyFP (&ecx_context, configadr, estat, timeout);
}

/** 绕过缓存从从站读取EEPROM（FPRD方法） - EC_VER1兼容函数。
 * 使用配置地址读取。
 * @param[in] configadr = 从站配置地址
 * @param[in] eeproma   = EEPROM地址（字）
 * @param[in] timeout   = 超时时间（微秒）
 * @return EEPROM数据（64位或32位）
 * @see ecx_readeepromFP
 */
uint64 ec_readeepromFP(uint16 configadr, uint16 eeproma, int timeout)
{
   return ecx_readeepromFP (&ecx_context, configadr, eeproma, timeout);
}

/** 绕过缓存向从站写入EEPROM（FPWR方法） - EC_VER1兼容函数。
 * 使用配置地址写入。
 * @param[in] configadr = 从站配置地址
 * @param[in] eeproma   = EEPROM地址（字）
 * @param[in] data      = 16位数据
 * @param[in] timeout   = 超时时间（微秒）
 * @return >0: 成功
 * @see ecx_writeeepromFP
 */
int ec_writeeepromFP(uint16 configadr, uint16 eeproma, uint16 data, int timeout)
{
   return ecx_writeeepromFP (&ecx_context, configadr, eeproma, data, timeout);
}

/** 绕过缓存从从站读取EEPROM - EC_VER1兼容函数。
 * 并行读取步骤1：向从站发送请求。
 * @param[in] slave   = 从站号
 * @param[in] eeproma = EEPROM地址（字）
 * @see ecx_readeeprom1
 */
void ec_readeeprom1(uint16 slave, uint16 eeproma)
{
   ecx_readeeprom1 (&ecx_context, slave, eeproma);
}

/** 绕过缓存从从站读取EEPROM - EC_VER1兼容函数。
 * 并行读取步骤2：实际从从站读取数据。
 * @param[in] slave   = 从站号
 * @param[in] timeout = 超时时间（微秒）
 * @return EEPROM数据（32位）
 * @see ecx_readeeprom2
 */
uint32 ec_readeeprom2(uint16 slave, int timeout)
{
   return ecx_readeeprom2 (&ecx_context, slave, timeout);
}

/** 向从站发送过程数据 - EC_VER1兼容函数。
 * 使用LRW，如果不允许LRW则使用LRD/LWR。
 * 输入和输出过程数据都会被发送。
 * 输出包含实际数据，输入使用占位符。
 * 输入数据通过接收过程数据函数收集。
 * 与基本LRW函数不同，此函数是非阻塞的。
 * 如果过程数据无法放入一个数据报中，则使用多个数据报。
 * 为了重新组合从站响应，使用了一个栈。
 * @param[in]  group = 组号
 * @return >0: 过程数据已发送
 * @see ecx_send_processdata_group
 */
int ec_send_processdata_group(uint8 group)
{
   return ecx_send_processdata_group (&ecx_context, group);
}

/** 向从站发送重叠过程数据 - EC_VER1兼容函数。
 * 使用LRW，如果不允许LRW则使用LRD/LWR。
 * 输入和输出过程数据在重叠IOmap中发送。
 * 输出包含实际数据，输入在返回帧中替换输出数据。
 * 输入数据通过接收过程数据函数收集。
 * 与基本LRW函数不同，此函数是非阻塞的。
 * 如果过程数据无法放入一个数据报中，则使用多个数据报。
 * 为了重新组合从站响应，使用了一个栈。
 * @param[in]  group = 组号
 * @return >0: 过程数据已发送
 * @see ecx_send_overlap_processdata_group
 */
int ec_send_overlap_processdata_group(uint8 group)
{
   return ecx_send_overlap_processdata_group(&ecx_context, group);
}

/** 从从站接收过程数据 - EC_VER1兼容函数。
 * ec_send_processdata()的第二部分。
 * 接收到的数据报借助栈与过程数据重新组合。
 * 如果数据报包含输入过程数据，则将其复制到过程数据结构中。
 * @param[in]  group   = 组号
 * @param[in]  timeout = 超时时间（微秒）
 * @return 工作计数器值
 * @see ecx_receive_processdata_group
 */
int ec_receive_processdata_group(uint8 group, int timeout)
{
   return ecx_receive_processdata_group (&ecx_context, group, timeout);
}

/** 发送过程数据（默认组0） - EC_VER1兼容函数。
 * @return >0: 过程数据已发送
 */
int ec_send_processdata(void)
{
   return ec_send_processdata_group(0);
}

/** 发送重叠过程数据（默认组0） - EC_VER1兼容函数。
 * @return >0: 过程数据已发送
 */
int ec_send_overlap_processdata(void)
{
   return ec_send_overlap_processdata_group(0);
}

/** 接收过程数据（默认组0） - EC_VER1兼容函数。
 * @param[in] timeout = 超时时间（微秒）
 * @return 工作计数器值
 */
int ec_receive_processdata(int timeout)
{
   return ec_receive_processdata_group(0, timeout);
}
#endif

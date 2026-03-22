/*
 * Licensed under the GNU General Public License version 2 with exceptions. See
 * LICENSE file in the project root for full license information
 */

/**
 * \file
 * \brief
 * Main EtherCAT functions.
 *
 * Initialisation, state set and read, mailbox primitives, EEPROM primitives,
 * SII reading and processdata exchange.
 *
 * Defines ec_slave[]. All slave information is put in this structure.
 * Needed for most user interaction with slaves.
 */

#include <stdio.h>
#include <string.h>
#include "osal.h"
#include "oshw.h"
#include "ethercat.h"


/** delay in us for eeprom ready loop */
#define EC_LOCALDELAY  200

/** record for ethercat eeprom communications */
PACKED_BEGIN
typedef struct PACKED
{
   uint16 comm; // EEPROM command
   uint16 addr; // EEPROM address
   uint16 d2; // EEPROM 地址后半段？
} ec_eepromt;
PACKED_END

/** mailbox error structure */
PACKED_BEGIN
typedef struct PACKED
{
   ec_mbxheadert   MbxHeader;
   uint16          Type;
   uint16          Detail;
} ec_mbxerrort;
PACKED_END

/** emergency request structure */
PACKED_BEGIN
typedef struct PACKED
{
   ec_mbxheadert   MbxHeader;
   uint16          CANOpen;
   uint16          ErrorCode;
   uint8           ErrorReg;
   uint8           bData;
   uint16          w1,w2;
} ec_emcyt;
PACKED_END

#ifdef EC_VER1
/** Main slave data array.
 *  Each slave found on the network gets its own record.
 *  ec_slave[0] is reserved for the master. Structure gets filled
 *  in by the configuration function ec_config().
 */
ec_slavet               ec_slave[EC_MAXSLAVE];
/** number of slaves found on the network */
int                     ec_slavecount;
/** slave group structure */
ec_groupt               ec_group[EC_MAXGROUP];

/** cache for EEPROM read functions */
static uint8            ec_esibuf[EC_MAXEEPBUF];
/** bitmap for filled cache buffer bytes */
static uint32           ec_esimap[EC_MAXEEPBITMAP];
/** current slave for EEPROM cache buffer */
static ec_eringt        ec_elist;
static ec_idxstackT     ec_idxstack;

/** SyncManager Communication Type struct to store data of one slave */
static ec_SMcommtypet   ec_SMcommtype[EC_MAX_MAPT];
/** PDO assign struct to store data of one slave */
static ec_PDOassignt    ec_PDOassign[EC_MAX_MAPT];
/** PDO description struct to store data of one slave */
static ec_PDOdesct      ec_PDOdesc[EC_MAX_MAPT];

/** buffer for EEPROM SM data */
static ec_eepromSMt     ec_SM;
/** buffer for EEPROM FMMU data */
static ec_eepromFMMUt   ec_FMMU;
/** Global variable TRUE if error available in error stack */
boolean                 EcatError = FALSE;

// 参考时钟上一次时间 ns
int64                   ec_DCtime;

ecx_portt               ecx_port;
ecx_redportt            ecx_redport;

ecx_contextt  ecx_context = {
    &ecx_port,          // .port          =
    &ec_slave[0],       // .slavelist     =
    &ec_slavecount,     // .slavecount    =
    EC_MAXSLAVE,        // .maxslave      =
    &ec_group[0],       // .grouplist     =
    EC_MAXGROUP,        // .maxgroup      =
    &ec_esibuf[0],      // .esibuf        =
    &ec_esimap[0],      // .esimap        =
    0,                  // .esislave      =
    &ec_elist,          // .elist         =
    &ec_idxstack,       // .idxstack      =
    &EcatError,         // .ecaterror     =
    &ec_DCtime,         // .DCtime        =
    &ec_SMcommtype[0],  // .SMcommtype    =
    &ec_PDOassign[0],   // .PDOassign     =
    &ec_PDOdesc[0],     // .PDOdesc       =
    &ec_SM,             // .eepSM         =
    &ec_FMMU,           // .eepFMMU       =
    NULL,               // .FOEhook()
    NULL,               // .EOEhook()
    0,                  // .manualstatechange
    NULL,               // .userdata
};
#endif

/** Create list over available network adapters.
 *
 * @return First element in list over available network adapters.
 */
ec_adaptert * ec_find_adapters (void)
{
   ec_adaptert * ret_adapter;

   ret_adapter = oshw_find_adapters ();

   return ret_adapter;
}

/** Free dynamically allocated list over available network adapters.
 *
 * @param[in] adapter = Struct holding adapter name, description and pointer to next.
 */
void ec_free_adapters (ec_adaptert * adapter)
{
   oshw_free_adapters (adapter);
}

/** Pushes an error on the error list.
 *
 * @param[in] context        = context struct
 * @param[in] Ec pointer describing the error.
 */
void ecx_pusherror(ecx_contextt *context, const ec_errort *Ec)
{
   context->elist->Error[context->elist->head] = *Ec;
   context->elist->Error[context->elist->head].Signal = TRUE;
   context->elist->head++;
   if (context->elist->head > EC_MAXELIST)
   {
      context->elist->head = 0;
   }
   if (context->elist->head == context->elist->tail)
   {
      context->elist->tail++;
   }
   if (context->elist->tail > EC_MAXELIST)
   {
      context->elist->tail = 0;
   }
   *(context->ecaterror) = TRUE;
}

/** Pops an error from the list.
 *
 * @param[in] context        = context struct
 * @param[out] Ec = Struct describing the error.
 * @return TRUE if an error was popped.
 */
boolean ecx_poperror(ecx_contextt *context, ec_errort *Ec)
{
   boolean notEmpty = (context->elist->head != context->elist->tail);

   *Ec = context->elist->Error[context->elist->tail];
   context->elist->Error[context->elist->tail].Signal = FALSE;
   if (notEmpty)
   {
      context->elist->tail++;
      if (context->elist->tail > EC_MAXELIST)
      {
         context->elist->tail = 0;
      }
   }
   else
   {
      *(context->ecaterror) = FALSE;
   }
   return notEmpty;
}

/** Check if error list has entries.
 *
 * @param[in] context        = context struct
 * @return TRUE if error list contains entries.
 */
boolean ecx_iserror(ecx_contextt *context)
{
   return (context->elist->head != context->elist->tail);
}

/** Report packet error
 *
 * @param[in]  context        = context struct
 * @param[in]  Slave      = Slave number
 * @param[in]  Index      = Index that generated error
 * @param[in]  SubIdx     = Subindex that generated error
 * @param[in]  ErrorCode  = Error code
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

/** Report Mailbox Error
 *
 * @param[in]  context        = context struct
 * @param[in]  Slave        = Slave number
 * @param[in]  Detail       = Following EtherCAT specification
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

/** Report Mailbox Emergency Error
 *
 * @param[in]  context        = context struct
 * @param[in]  Slave      = Slave number
 * @param[in]  ErrorCode  = Following EtherCAT specification
 * @param[in]  ErrorReg
 * @param[in]  b1
 * @param[in]  w1
 * @param[in]  w2
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

/** Initialise lib in single NIC mode
 * @param[in]  context = context struct
 * @param[in] ifname   = Dev name, f.e. "eth0"
 * @return >0 if OK
 */
int ecx_init(ecx_contextt *context, const char * ifname)
{
   return ecx_setupnic(context->port, ifname, FALSE);
}

/** Initialise lib in redundant NIC mode
 * @param[in]  context  = context struct
 * @param[in]  redport  = pointer to redport, redundant port data
 * @param[in]  ifname   = Primary Dev name, f.e. "eth0"
 * @param[in]  if2name  = Secondary Dev name, f.e. "eth1"
 * @return >0 if OK
 */
int ecx_init_redundant(ecx_contextt *context, ecx_redportt *redport, const char *ifname, char *if2name)
{
   int rval, zbuf;
   ec_etherheadert *ehp;

   context->port->redport = redport;
   ecx_setupnic(context->port, ifname, FALSE);
   rval = ecx_setupnic(context->port, if2name, TRUE);
   /* prepare "dummy" BRD tx frame for redundant operation */
   ehp = (ec_etherheadert *)&(context->port->txbuf2);
   ehp->sa1 = oshw_htons(secMAC[0]);
   zbuf = 0;
   ecx_setupdatagram(context->port, &(context->port->txbuf2), EC_CMD_BRD, 0, 0x0000, 0x0000, 2, &zbuf);
   context->port->txbuflength2 = ETH_HEADERSIZE + EC_HEADERSIZE + EC_WKCSIZE + 2;

   return rval;
}

/** Close lib.
 * @param[in]  context        = context struct
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

/** Find SII section header in slave EEPROM.
 *  @param[in]  context        = context struct
 *  @param[in] slave   = slave number
 *  @param[in] cat     = section category
 *  @return byte address of section at section length entry, if not available then 0
 */
int16 ecx_siifind(ecx_contextt *context, uint16 slave, uint16 cat)
{
   int16 a;
   uint16 p;
   uint8 eectl = context->slavelist[slave].eep_pdi;

   a = ECT_SII_START << 1;
   /* read first SII section category */
   p = ecx_siigetbyte(context, slave, a++);
   p += (ecx_siigetbyte(context, slave, a++) << 8);
   /* traverse SII while category is not found and not EOF */
   while ((p != cat) && (p != 0xffff))
   {
      /* read section length */
      p = ecx_siigetbyte(context, slave, a++);
      p += (ecx_siigetbyte(context, slave, a++) << 8);
      /* locate next section category */
      a += p << 1;
      /* read section category */
      p = ecx_siigetbyte(context, slave, a++);
      p += (ecx_siigetbyte(context, slave, a++) << 8);
   }
   if (p != cat)
   {
      a = 0;
   }
   if (eectl)
   {
      ecx_eeprom2pdi(context, slave); /* if eeprom control was previously pdi then restore */
   }

   return a;
}

/** Get string from SII string section in slave EEPROM.
 *  @param[in]  context = context struct
 *  @param[out] str     = requested string, 0x00 if not found
 *  @param[in]  slave   = slave number
 *  @param[in]  Sn      = string number
 */
void ecx_siistring(ecx_contextt *context, char *str, uint16 slave, uint16 Sn)
{
   uint16 a,i,j,l,n,ba;
   char *ptr;
   uint8 eectl = context->slavelist[slave].eep_pdi;

   ptr = str;
   a = ecx_siifind (context, slave, ECT_SII_STRING); /* find string section */
   if (a > 0)
   {
      ba = a + 2; /* skip SII section header */
      n = ecx_siigetbyte(context, slave, ba++); /* read number of strings in section */
      if (Sn <= n) /* is req string available? */
      {
         for (i = 1; i <= Sn; i++) /* walk through strings */
         {
            l = ecx_siigetbyte(context, slave, ba++); /* length of this string */
            if (i < Sn)
            {
               ba += l;
            }
            else
            {
               ptr = str;
               for (j = 1; j <= l; j++) /* copy one string */
               {
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
         *ptr = 0; /* add zero terminator */
      }
      else
      {
         ptr = str;
         *ptr = 0; /* empty string */
      }
   }
   if (eectl)
   {
      ecx_eeprom2pdi(context, slave); /* if eeprom control was previously pdi then restore */
   }
}

/** Get FMMU data from SII FMMU section in slave EEPROM.
 *  @param[in]  context = context struct
 *  @param[in]  slave   = slave number
 *  @param[out] FMMU    = FMMU struct from SII, max. 4 FMMU's
 *  @return number of FMMU's defined in section
 */
uint16 ecx_siiFMMU(ecx_contextt *context, uint16 slave, ec_eepromFMMUt* FMMU)
{
   uint16  a;
   uint8 eectl = context->slavelist[slave].eep_pdi;

   FMMU->nFMMU = 0;
   FMMU->FMMU0 = 0;
   FMMU->FMMU1 = 0;
   FMMU->FMMU2 = 0;
   FMMU->FMMU3 = 0;
   FMMU->Startpos = ecx_siifind(context, slave, ECT_SII_FMMU);

   if (FMMU->Startpos > 0)
   {
      a = FMMU->Startpos;
      FMMU->nFMMU = ecx_siigetbyte(context, slave, a++);
      FMMU->nFMMU += (ecx_siigetbyte(context, slave, a++) << 8);
      FMMU->nFMMU *= 2;
      FMMU->FMMU0 = ecx_siigetbyte(context, slave, a++);
      FMMU->FMMU1 = ecx_siigetbyte(context, slave, a++);
      if (FMMU->nFMMU > 2)
      {
         FMMU->FMMU2 = ecx_siigetbyte(context, slave, a++);
         FMMU->FMMU3 = ecx_siigetbyte(context, slave, a++);
      }
   }
   if (eectl)
   {
      ecx_eeprom2pdi(context, slave); /* if eeprom control was previously pdi then restore */
   }

   return FMMU->nFMMU;
}

/** Get SM data from SII SM section in slave EEPROM.
 *  @param[in]  context = context struct
 *  @param[in]  slave   = slave number
 *  @param[out] SM      = first SM struct from SII
 *  @return number of SM's defined in section
 */
uint16 ecx_siiSM(ecx_contextt *context, uint16 slave, ec_eepromSMt* SM)
{
   uint16 a,w;
   uint8 eectl = context->slavelist[slave].eep_pdi;

   SM->nSM = 0;
   SM->Startpos = ecx_siifind(context, slave, ECT_SII_SM);
   if (SM->Startpos > 0)
   {
      a = SM->Startpos;
      w = ecx_siigetbyte(context, slave, a++);
      w += (ecx_siigetbyte(context, slave, a++) << 8);
      SM->nSM = (uint8)(w / 4);
      SM->PhStart = ecx_siigetbyte(context, slave, a++);
      SM->PhStart += (ecx_siigetbyte(context, slave, a++) << 8);
      SM->Plength = ecx_siigetbyte(context, slave, a++);
      SM->Plength += (ecx_siigetbyte(context, slave, a++) << 8);
      SM->Creg = ecx_siigetbyte(context, slave, a++);
      SM->Sreg = ecx_siigetbyte(context, slave, a++);
      SM->Activate = ecx_siigetbyte(context, slave, a++);
      SM->PDIctrl = ecx_siigetbyte(context, slave, a++);
   }
   if (eectl)
   {
      ecx_eeprom2pdi(context, slave); /* if eeprom control was previously pdi then restore */
   }

   return SM->nSM;
}

/** Get next SM data from SII SM section in slave EEPROM.
 *  @param[in]  context = context struct
 *  @param[in]  slave   = slave number
 *  @param[out] SM      = first SM struct from SII
 *  @param[in]  n       = SM number
 *  @return >0 if OK
 */
uint16 ecx_siiSMnext(ecx_contextt *context, uint16 slave, ec_eepromSMt* SM, uint16 n)
{
   uint16 a;
   uint16 retVal = 0;
   uint8 eectl = context->slavelist[slave].eep_pdi;

   if (n < SM->nSM)
   {
      a = SM->Startpos + 2 + (n * 8);
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
      ecx_eeprom2pdi(context, slave); /* if eeprom control was previously pdi then restore */
   }

   return retVal;
}

/** Get PDO data from SII PDO section in slave EEPROM.
 *  @param[in]  context = context struct
 *  @param[in]  slave   = slave number
 *  @param[out] PDO     = PDO struct from SII
 *  @param[in]  t       = 0=RXPDO 1=TXPDO
 *  @return mapping size in bits of PDO
 */
uint32 ecx_siiPDO(ecx_contextt *context, uint16 slave, ec_eepromPDOt* PDO, uint8 t)
{
   uint16 a , w, c, e, er, Size;
   uint8 eectl = context->slavelist[slave].eep_pdi;

   Size = 0;
   PDO->nPDO = 0;
   PDO->Length = 0;
   PDO->Index[1] = 0;
   for (c = 0 ; c < EC_MAXSM ; c++) PDO->SMbitsize[c] = 0;
   if (t > 1)
      t = 1;
   PDO->Startpos = ecx_siifind(context, slave, ECT_SII_PDO + t);
   if (PDO->Startpos > 0)
   {
      a = PDO->Startpos;
      w = ecx_siigetbyte(context, slave, a++);
      w += (ecx_siigetbyte(context, slave, a++) << 8);
      PDO->Length = w;
      c = 1;
      /* traverse through all PDOs */
      do
      {
         PDO->nPDO++;
         PDO->Index[PDO->nPDO] = ecx_siigetbyte(context, slave, a++);
         PDO->Index[PDO->nPDO] += (ecx_siigetbyte(context, slave, a++) << 8);
         PDO->BitSize[PDO->nPDO] = 0;
         c++;
         e = ecx_siigetbyte(context, slave, a++);
         PDO->SyncM[PDO->nPDO] = ecx_siigetbyte(context, slave, a++);
         a += 4;
         c += 2;
         if (PDO->SyncM[PDO->nPDO] < EC_MAXSM) /* active and in range SM? */
         {
            /* read all entries defined in PDO */
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
         else /* PDO deactivated because SM is 0xff or > EC_MAXSM */
         {
            c += 4 * e;
            a += 8 * e;
            c++;
         }
         if (PDO->nPDO >= (EC_MAXEEPDO - 1))
         {
            c = PDO->Length; /* limit number of PDO entries in buffer */
         }
      }
      while (c < PDO->Length);
   }
   if (eectl)
   {
      ecx_eeprom2pdi(context, slave); /* if eeprom control was previously pdi then restore */
   }

   return (Size);
}

#define MAX_FPRD_MULTI 64

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
   ecx_setupdatagram(port, &(port->txbuf[idx]), EC_CMD_FPRD, idx,
      *(configlst + slcnt), ECT_REG_ALSTAT, sizeof(ec_alstatust), slstatlst + slcnt);
   sldatapos[slcnt] = EC_HEADERSIZE;
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

/** Read all slave states in ec_slave.
 * @warning The BOOT state is actually higher than INIT and PRE_OP (see state representation)
 * @param[in] context = context struct
 * @return lowest state found
 */
int ecx_readstate(ecx_contextt *context)
{
   uint16 slave, fslave, lslave, configadr, lowest, rval, bitwisestate;
   ec_alstatust sl[MAX_FPRD_MULTI];
   uint16 slca[MAX_FPRD_MULTI];
   boolean noerrorflag, allslavessamestate;
   boolean allslavespresent = FALSE;
   int wkc;

   /* Try to establish the state of all slaves sending only one broadcast datagram.
    * This way a number of datagrams equal to the number of slaves will be sent only if needed.*/
   rval = 0;
   wkc = ecx_BRD(context->port, 0, ECT_REG_ALSTAT, sizeof(rval), &rval, EC_TIMEOUTRET);

   if(wkc >= *(context->slavecount))
   {
      allslavespresent = TRUE;
   }

   rval = etohs(rval);
   bitwisestate = (rval & 0x0f);

   if ((rval & EC_STATE_ERROR) == 0)
   {
      noerrorflag = TRUE;
      context->slavelist[0].ALstatuscode = 0;
   }
   else
   {
      noerrorflag = FALSE;
   }

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
      context->slavelist[0].ALstatuscode = 0;
      lowest = 0xff;
      fslave = 1;
      do
      {
         lslave = (uint16)*(context->slavecount);
         if ((lslave - fslave) >= MAX_FPRD_MULTI)
         {
            lslave = fslave + MAX_FPRD_MULTI - 1;
         }
         for (slave = fslave; slave <= lslave; slave++)
         {
            const ec_alstatust zero = { 0, 0, 0 };

            configadr = context->slavelist[slave].configadr;
            slca[slave - fslave] = configadr;
            sl[slave - fslave] = zero;
         }
         ecx_FPRD_multi(context, (lslave - fslave) + 1, &(slca[0]), &(sl[0]), EC_TIMEOUTRET3);
         for (slave = fslave; slave <= lslave; slave++)
         {
            configadr = context->slavelist[slave].configadr;
            rval = etohs(sl[slave - fslave].alstatus);
            context->slavelist[slave].ALstatuscode = etohs(sl[slave - fslave].alstatuscode);
            if ((rval & 0xf) < lowest)
            {
               lowest = (rval & 0xf);
            }
            context->slavelist[slave].state = rval;
            context->slavelist[0].ALstatuscode |= context->slavelist[slave].ALstatuscode;
         }
         fslave = lslave + 1;
      } while (lslave < *(context->slavecount));
      context->slavelist[0].state = lowest;
   }

   return lowest;
}

/** Write slave state, if slave = 0 then write to all slaves.
 * The function does not check if the actual state is changed.
 * @param[in]  context        = context struct
 * @param[in] slave    = Slave number, 0 = master
 * @return Workcounter or EC_NOFRAME
 */
int ecx_writestate(ecx_contextt *context, uint16 slave)
{
   int ret;
   uint16 configadr, slstate;

   if (slave == 0)
   {
      slstate = htoes(context->slavelist[slave].state);
      ret = ecx_BWR(context->port, 0, ECT_REG_ALCTL, sizeof(slstate),
	            &slstate, EC_TIMEOUTRET3);
   }
   else
   {
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

/** Get index of next mailbox counter value.
 * Used for Mailbox Link Layer.
 * @param[in] cnt     = Mailbox counter value [0..7]
 * @return next mailbox counter value
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

/** Clear mailbox buffer.
 * @param[out] Mbx     = Mailbox buffer to clear
 */
void ec_clearmbx(ec_mbxbuft *Mbx)
{
    memset(Mbx, 0x00, EC_MAXMBX);
}

/** Check if IN mailbox of slave is empty.
 * @param[in] context  = context struct
 * @param[in] slave    = Slave number
 * @param[in] timeout  = Timeout in us
 * @return >0 is success
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
      wkc = ecx_FPRD(context->port, configadr, ECT_REG_SM0STAT, sizeof(SMstat), &SMstat, EC_TIMEOUTRET);
      SMstat = etohs(SMstat);
      if (((SMstat & 0x08) != 0) && (timeout > EC_LOCALDELAY))
      {
         osal_usleep(EC_LOCALDELAY);
      }
   }
   while (((wkc <= 0) || ((SMstat & 0x08) != 0)) && (osal_timer_is_expired(&timer) == FALSE));

   if ((wkc > 0) && ((SMstat & 0x08) == 0))
   {
      return 1;
   }

   return 0;
}

/** Write IN mailbox to slave.
 * @param[in]  context    = context struct
 * @param[in]  slave      = Slave number
 * @param[out] mbx        = Mailbox data
 * @param[in]  timeout    = Timeout in us
 * @return Work counter (>0 is success)
 */
int ecx_mbxsend(ecx_contextt *context, uint16 slave,ec_mbxbuft *mbx, int timeout)
{
   uint16 mbxwo,mbxl,configadr;
   int wkc;

   wkc = 0;
   configadr = context->slavelist[slave].configadr;
   mbxl = context->slavelist[slave].mbx_l;
   if ((mbxl > 0) && (mbxl <= EC_MAXMBX))
   {
      if (ecx_mbxempty(context, slave, timeout))
      {
         mbxwo = context->slavelist[slave].mbx_wo;
         /* write slave in mailbox */
         wkc = ecx_FPWR(context->port, configadr, mbxwo, mbxl, mbx, EC_TIMEOUTRET3);
      }
      else
      {
         wkc = 0;
      }
   }

   return wkc;
}

/** Read OUT mailbox from slave.
 * Supports Mailbox Link Layer with repeat requests.
 * @param[in]  context    = context struct
 * @param[in]  slave      = Slave number
 * @param[out] mbx        = Mailbox data
 * @param[in]  timeout    = Timeout in us
 * @return Work counter (>0 is success)
 */
int ecx_mbxreceive(ecx_contextt *context, uint16 slave, ec_mbxbuft *mbx, int timeout)
{
   uint16 mbxro,mbxl,configadr;
   int wkc=0;
   int wkc2;
   uint16 SMstat;
   uint8 SMcontr;
   ec_mbxheadert *mbxh;
   ec_emcyt *EMp;
   ec_mbxerrort *MBXEp;

   configadr = context->slavelist[slave].configadr;
   mbxl = context->slavelist[slave].mbx_rl;
   if ((mbxl > 0) && (mbxl <= EC_MAXMBX))
   {
      osal_timert timer;

      osal_timer_start(&timer, timeout);
      wkc = 0;
      do /* wait for read mailbox available */
      {
         SMstat = 0;
         wkc = ecx_FPRD(context->port, configadr, ECT_REG_SM1STAT, sizeof(SMstat), &SMstat, EC_TIMEOUTRET);
         SMstat = etohs(SMstat);
         if (((SMstat & 0x08) == 0) && (timeout > EC_LOCALDELAY))
         {
            osal_usleep(EC_LOCALDELAY);
         }
      }
      while (((wkc <= 0) || ((SMstat & 0x08) == 0)) && (osal_timer_is_expired(&timer) == FALSE));

      if ((wkc > 0) && ((SMstat & 0x08) > 0)) /* read mailbox available ? */
      {
         mbxro = context->slavelist[slave].mbx_ro;
         mbxh = (ec_mbxheadert *)mbx;
         do
         {
            wkc = ecx_FPRD(context->port, configadr, mbxro, mbxl, mbx, EC_TIMEOUTRET); /* get mailbox */
            if ((wkc > 0) && ((mbxh->mbxtype & 0x0f) == 0x00)) /* Mailbox error response? */
            {
               MBXEp = (ec_mbxerrort *)mbx;
               ecx_mbxerror(context, slave, etohs(MBXEp->Detail));
               wkc = 0; /* prevent emergency to cascade up, it is already handled. */
            }
            else if ((wkc > 0) && ((mbxh->mbxtype & 0x0f) == ECT_MBXT_COE)) /* CoE response? */
            {
               EMp = (ec_emcyt *)mbx;
               if ((etohs(EMp->CANOpen) >> 12) == 0x01) /* Emergency request? */
               {
                  ecx_mbxemergencyerror(context, slave, etohs(EMp->ErrorCode), EMp->ErrorReg,
                          EMp->bData, etohs(EMp->w1), etohs(EMp->w2));
                  wkc = 0; /* prevent emergency to cascade up, it is already handled. */
               }
            }
            else if ((wkc > 0) && ((mbxh->mbxtype & 0x0f) == ECT_MBXT_EOE)) /* EoE response? */
            {
               ec_EOEt * eoembx = (ec_EOEt *)mbx;
               uint16 frameinfo1 = etohs(eoembx->frameinfo1);
               /* All non fragment data frame types are expected to be handled by
               * slave send/receive API if the EoE hook is set
               */
               if (EOE_HDR_FRAME_TYPE_GET(frameinfo1) == EOE_FRAG_DATA)
               {
                  if (context->EOEhook)
                  {
                     if (context->EOEhook(context, slave, eoembx) > 0)
                     {
                        /* Fragment handled by EoE hook */
                        wkc = 0;
                     }
                  }
               }
            }
            else
            {
               if (wkc <= 0) /* read mailbox lost */
               {
                  SMstat ^= 0x0200; /* toggle repeat request */
                  SMstat = htoes(SMstat);
                  wkc2 = ecx_FPWR(context->port, configadr, ECT_REG_SM1STAT, sizeof(SMstat), &SMstat, EC_TIMEOUTRET);
                  SMstat = etohs(SMstat);
                  do /* wait for toggle ack */
                  {
                     wkc2 = ecx_FPRD(context->port, configadr, ECT_REG_SM1CONTR, sizeof(SMcontr), &SMcontr, EC_TIMEOUTRET);
                   } while (((wkc2 <= 0) || ((SMcontr & 0x02) != (HI_BYTE(SMstat) & 0x02))) && (osal_timer_is_expired(&timer) == FALSE));
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
static int ecx_main_send_processdata(ecx_contextt *context, uint8 group, boolean use_overlap_io)
{
   uint32 LogAdr;
   uint16 w1, w2;
   int length;
   uint16 sublength;
   uint8 idx;
   int wkc;
   uint8* data;
   boolean first=FALSE;
   uint16 currentsegment = 0;
   uint32 iomapinputoffset;
   uint16 DCO;

   wkc = 0;
   if(context->grouplist[group].hasdc)
   {
      first = TRUE;
   }

   /* For overlapping IO map use the biggest */
   if(use_overlap_io == TRUE)
   {
      /* For overlap IOmap make the frame EQ big to biggest part */
      length = (context->grouplist[group].Obytes > context->grouplist[group].Ibytes) ?
         context->grouplist[group].Obytes : context->grouplist[group].Ibytes;
      /* Save the offset used to compensate where to save inputs when frame returns */
      iomapinputoffset = context->grouplist[group].Obytes;
   }
   else
   {
      length = context->grouplist[group].Obytes + context->grouplist[group].Ibytes;
      iomapinputoffset = 0;
   }

   LogAdr = context->grouplist[group].logstartaddr;
   if(length)
   {

      wkc = 1;
      /* LRW blocked by one or more slaves ? */
      if(context->grouplist[group].blockLRW)
      {
         /* if inputs available generate LRD */
         if(context->grouplist[group].Ibytes)
         {
            currentsegment = context->grouplist[group].Isegment;
            data = context->grouplist[group].inputs;
            length = context->grouplist[group].Ibytes;
            LogAdr += context->grouplist[group].Obytes;
            /* segment transfer if needed */
            do
            {
               if(currentsegment == context->grouplist[group].Isegment)
               {
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
               ecx_setupdatagram(context->port, &(context->port->txbuf[idx]), EC_CMD_LRD, idx, w1, w2, sublength, data);
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
               /* push index and data pointer on stack */
               ecx_pushindex(context, idx, data, sublength, DCO);
               length -= sublength;
               LogAdr += sublength;
               data += sublength;
            } while (length && (currentsegment < context->grouplist[group].nsegments));
         }
         /* if outputs available generate LWR */
         if(context->grouplist[group].Obytes)
         {
            data = context->grouplist[group].outputs;
            length = context->grouplist[group].Obytes;
            LogAdr = context->grouplist[group].logstartaddr;
            currentsegment = 0;
            /* segment transfer if needed */
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
               ecx_setupdatagram(context->port, &(context->port->txbuf[idx]), EC_CMD_LWR, idx, w1, w2, sublength, data);
               if(first)
               {
                  /* FPRMW in second datagram */
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
      else
      {
         if (context->grouplist[group].Obytes)
         {
            data = context->grouplist[group].outputs;
         }
         else
         {
            data = context->grouplist[group].inputs;
            /* Clear offset, don't compensate for overlapping IOmap if we only got inputs */
            iomapinputoffset = 0;
         }
         /* segment transfer if needed */
         do
         {
            sublength = (uint16)context->grouplist[group].IOsegment[currentsegment++];
            /* get new index */
            idx = ecx_getindex(context->port);
            w1 = LO_WORD(LogAdr);
            w2 = HI_WORD(LogAdr);
            DCO = 0;
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


int ecx_send_processdata(ecx_contextt *context)
{
   return ecx_send_processdata_group(context, 0);
}

int ecx_send_overlap_processdata(ecx_contextt *context)
{
   return ecx_send_overlap_processdata_group(context, 0);
}

int ecx_receive_processdata(ecx_contextt *context, int timeout)
{
   return ecx_receive_processdata_group(context, 0, timeout);
}

#ifdef EC_VER1
void ec_pusherror(const ec_errort *Ec)
{
   ecx_pusherror(&ecx_context, Ec);
}

boolean ec_poperror(ec_errort *Ec)
{
   return ecx_poperror(&ecx_context, Ec);
}

boolean ec_iserror(void)
{
   return ecx_iserror(&ecx_context);
}

void ec_packeterror(uint16 Slave, uint16 Index, uint8 SubIdx, uint16 ErrorCode)
{
   ecx_packeterror(&ecx_context, Slave, Index, SubIdx, ErrorCode);
}

/** Initialise lib in single NIC mode
 * @param[in] ifname   = Dev name, f.e. "eth0"
 * @return >0 if OK
 * @see ecx_init
 */
int ec_init(const char * ifname)
{
   return ecx_init(&ecx_context, ifname);
}

/** Initialise lib in redundant NIC mode
 * @param[in]  ifname   = Primary Dev name, f.e. "eth0"
 * @param[in]  if2name  = Secondary Dev name, f.e. "eth1"
 * @return >0 if OK
 * @see ecx_init_redundant
 */
int ec_init_redundant(const char *ifname, char *if2name)
{
   return ecx_init_redundant (&ecx_context, &ecx_redport, ifname, if2name);
}

/** Close lib.
 * @see ecx_close
 */
void ec_close(void)
{
   ecx_close(&ecx_context);
};

/** Read one byte from slave EEPROM via cache.
 *  If the cache location is empty then a read request is made to the slave.
 *  Depending on the slave capabillities the request is 4 or 8 bytes.
 *  @param[in] slave   = slave number
 *  @param[in] address = eeprom address in bytes (slave uses words)
 *  @return requested byte, if not available then 0xff
 * @see ecx_siigetbyte
 */
uint8 ec_siigetbyte(uint16 slave, uint16 address)
{
   return ecx_siigetbyte (&ecx_context, slave, address);
}

/** Find SII section header in slave EEPROM.
 *  @param[in] slave   = slave number
 *  @param[in] cat     = section category
 *  @return byte address of section at section length entry, if not available then 0
 *  @see ecx_siifind
 */
int16 ec_siifind(uint16 slave, uint16 cat)
{
   return ecx_siifind (&ecx_context, slave, cat);
}

/** Get string from SII string section in slave EEPROM.
 *  @param[out] str    = requested string, 0x00 if not found
 *  @param[in]  slave  = slave number
 *  @param[in]  Sn     = string number
 *  @see ecx_siistring
 */
void ec_siistring(char *str, uint16 slave, uint16 Sn)
{
   ecx_siistring(&ecx_context, str, slave, Sn);
}

/** Get FMMU data from SII FMMU section in slave EEPROM.
 *  @param[in]  slave  = slave number
 *  @param[out] FMMU   = FMMU struct from SII, max. 4 FMMU's
 *  @return number of FMMU's defined in section
 *  @see ecx_siiFMMU
 */
uint16 ec_siiFMMU(uint16 slave, ec_eepromFMMUt* FMMU)
{
   return ecx_siiFMMU (&ecx_context, slave, FMMU);
}

/** Get SM data from SII SM section in slave EEPROM.
 *  @param[in]  slave   = slave number
 *  @param[out] SM      = first SM struct from SII
 *  @return number of SM's defined in section
 *  @see ecx_siiSM
 */
uint16 ec_siiSM(uint16 slave, ec_eepromSMt* SM)
{
   return ecx_siiSM (&ecx_context, slave, SM);
}

/** Get next SM data from SII SM section in slave EEPROM.
 *  @param[in]  slave  = slave number
 *  @param[out] SM     = first SM struct from SII
 *  @param[in]  n      = SM number
 *  @return >0 if OK
 *  @see ecx_siiSMnext
 */
uint16 ec_siiSMnext(uint16 slave, ec_eepromSMt* SM, uint16 n)
{
   return ecx_siiSMnext (&ecx_context, slave, SM, n);
}

/** Get PDO data from SII PDO section in slave EEPROM.
 *  @param[in]  slave  = slave number
 *  @param[out] PDO    = PDO struct from SII
 *  @param[in]  t      = 0=RXPDO 1=TXPDO
 *  @return mapping size in bits of PDO
 *  @see ecx_siiPDO
 */
uint32 ec_siiPDO(uint16 slave, ec_eepromPDOt* PDO, uint8 t)
{
   return ecx_siiPDO (&ecx_context, slave, PDO, t);
}

/** Read all slave states in ec_slave.
 * @warning The BOOT state is actually higher than INIT and PRE_OP (see state representation).
 * @return lowest state found
 * @see ecx_readstate
 */
int ec_readstate(void)
{
   return ecx_readstate (&ecx_context);
}

/** Write slave state, if slave = 0 then write to all slaves.
 * The function does not check if the actual state is changed.
 * @param[in] slave = Slave number, 0 = master
 * @return 0
 * @see ecx_writestate
 */
int ec_writestate(uint16 slave)
{
   return ecx_writestate(&ecx_context, slave);
}

/** Check actual slave state.
 * This is a blocking function.
 * To refresh the state of all slaves ecx_readstate() should be called.
 * @warning If this is used for slave 0 (=all slaves), the state of all slaves is read by an bitwise OR operation.
 * The returned value is also the bitwise OR state of all slaves.
 * This has some implications for the BOOT state. The Boot state representation collides with INIT | PRE_OP so this
 * function cannot be used for slave = 0 and reqstate = EC_STATE_BOOT and also, if the returned state is BOOT, some
 * slaves might actually be in INIT and PRE_OP and not in BOOT.
 * @param[in] slave       = Slave number, 0 = all slaves
 * @param[in] reqstate    = Requested state
 * @param[in] timeout     = Timeout value in us
 * @return Requested state, or found state after timeout.
 * @see ecx_statecheck
 */
uint16 ec_statecheck(uint16 slave, uint16 reqstate, int timeout)
{
   return ecx_statecheck (&ecx_context, slave, reqstate, timeout);
}

/** Check if IN mailbox of slave is empty.
 * @param[in] slave    = Slave number
 * @param[in] timeout  = Timeout in us
 * @return >0 is success
 * @see ecx_mbxempty
 */
int ec_mbxempty(uint16 slave, int timeout)
{
   return ecx_mbxempty (&ecx_context, slave, timeout);
}

/** Write IN mailbox to slave.
 * @param[in]  slave      = Slave number
 * @param[out] mbx        = Mailbox data
 * @param[in]  timeout    = Timeout in us
 * @return Work counter (>0 is success)
 * @see ecx_mbxsend
 */
int ec_mbxsend(uint16 slave,ec_mbxbuft *mbx, int timeout)
{
   return ecx_mbxsend (&ecx_context, slave, mbx, timeout);
}

/** Read OUT mailbox from slave.
 * Supports Mailbox Link Layer with repeat requests.
 * @param[in]  slave      = Slave number
 * @param[out] mbx        = Mailbox data
 * @param[in]  timeout    = Timeout in us
 * @return Work counter (>0 is success)
 * @see ecx_mbxreceive
 */
int ec_mbxreceive(uint16 slave, ec_mbxbuft *mbx, int timeout)
{
   return ecx_mbxreceive (&ecx_context, slave, mbx, timeout);
}

/** Dump complete EEPROM data from slave in buffer.
 * @param[in]  slave    = Slave number
 * @param[out] esibuf   = EEPROM data buffer, make sure it is big enough.
 * @see ecx_esidump
 */
void ec_esidump(uint16 slave, uint8 *esibuf)
{
   ecx_esidump (&ecx_context, slave, esibuf);
}

/** Read EEPROM from slave bypassing cache.
 * @param[in] slave     = Slave number
 * @param[in] eeproma   = (WORD) Address in the EEPROM
 * @param[in] timeout   = Timeout in us.
 * @return EEPROM data 32bit
 * @see ecx_readeeprom
 */
uint32 ec_readeeprom(uint16 slave, uint16 eeproma, int timeout)
{
   return ecx_readeeprom (&ecx_context, slave, eeproma, timeout);
}

/** Write EEPROM to slave bypassing cache.
 * @param[in] slave     = Slave number
 * @param[in] eeproma   = (WORD) Address in the EEPROM
 * @param[in] data      = 16bit data
 * @param[in] timeout   = Timeout in us.
 * @return >0 if OK
 * @see ecx_writeeeprom
 */
int ec_writeeeprom(uint16 slave, uint16 eeproma, uint16 data, int timeout)
{
   return ecx_writeeeprom (&ecx_context, slave, eeproma, data, timeout);
}

/** Set eeprom control to master. Only if set to PDI.
 * @param[in] slave = Slave number
 * @return >0 if OK
 * @see ecx_eeprom2master
 */
int ec_eeprom2master(uint16 slave)
{
   return ecx_eeprom2master(&ecx_context, slave);
}

int ec_eeprom2pdi(uint16 slave)
{
   return ecx_eeprom2pdi(&ecx_context, slave);
}

uint16 ec_eeprom_waitnotbusyAP(uint16 aiadr,uint16 *estat, int timeout)
{
   return ecx_eeprom_waitnotbusyAP (&ecx_context, aiadr, estat, timeout);
}

/** Read EEPROM from slave bypassing cache. APRD method.
 * @param[in] aiadr       = auto increment address of slave
 * @param[in] eeproma     = (WORD) Address in the EEPROM
 * @param[in] timeout     = Timeout in us.
 * @return EEPROM data 64bit or 32bit
 */
uint64 ec_readeepromAP(uint16 aiadr, uint16 eeproma, int timeout)
{
   return ecx_readeepromAP (&ecx_context, aiadr, eeproma, timeout);
}

/** Write EEPROM to slave bypassing cache. APWR method.
 * @param[in] aiadr     = configured address of slave
 * @param[in] eeproma   = (WORD) Address in the EEPROM
 * @param[in] data      = 16bit data
 * @param[in] timeout   = Timeout in us.
 * @return >0 if OK
 * @see ecx_writeeepromAP
 */
int ec_writeeepromAP(uint16 aiadr, uint16 eeproma, uint16 data, int timeout)
{
   return ecx_writeeepromAP (&ecx_context, aiadr, eeproma, data, timeout);
}

uint16 ec_eeprom_waitnotbusyFP(uint16 configadr,uint16 *estat, int timeout)
{
   return ecx_eeprom_waitnotbusyFP (&ecx_context, configadr, estat, timeout);
}

/** Read EEPROM from slave bypassing cache. FPRD method.
 * @param[in] configadr   = configured address of slave
 * @param[in] eeproma     = (WORD) Address in the EEPROM
 * @param[in] timeout     = Timeout in us.
 * @return EEPROM data 64bit or 32bit
 * @see ecx_readeepromFP
 */
uint64 ec_readeepromFP(uint16 configadr, uint16 eeproma, int timeout)
{
   return ecx_readeepromFP (&ecx_context, configadr, eeproma, timeout);
}

/** Write EEPROM to slave bypassing cache. FPWR method.
 * @param[in] configadr   = configured address of slave
 * @param[in] eeproma     = (WORD) Address in the EEPROM
 * @param[in] data        = 16bit data
 * @param[in] timeout     = Timeout in us.
 * @return >0 if OK
 * @see ecx_writeeepromFP
 */
int ec_writeeepromFP(uint16 configadr, uint16 eeproma, uint16 data, int timeout)
{
   return ecx_writeeepromFP (&ecx_context, configadr, eeproma, data, timeout);
}

/** Read EEPROM from slave bypassing cache.
 * Parallel read step 1, make request to slave.
 * @param[in] slave       = Slave number
 * @param[in] eeproma     = (WORD) Address in the EEPROM
 * @see ecx_readeeprom1
 */
void ec_readeeprom1(uint16 slave, uint16 eeproma)
{
   ecx_readeeprom1 (&ecx_context, slave, eeproma);
}

/** Read EEPROM from slave bypassing cache.
 * Parallel read step 2, actual read from slave.
 * @param[in] slave       = Slave number
 * @param[in] timeout     = Timeout in us.
 * @return EEPROM data 32bit
 * @see ecx_readeeprom2
 */
uint32 ec_readeeprom2(uint16 slave, int timeout)
{
   return ecx_readeeprom2 (&ecx_context, slave, timeout);
}

/** Transmit processdata to slaves.
 * Uses LRW, or LRD/LWR if LRW is not allowed (blockLRW).
 * Both the input and output processdata are transmitted.
 * The outputs with the actual data, the inputs have a placeholder.
 * The inputs are gathered with the receive processdata function.
 * In contrast to the base LRW function this function is non-blocking.
 * If the processdata does not fit in one datagram, multiple are used.
 * In order to recombine the slave response, a stack is used.
 * @param[in]  group          = group number
 * @return >0 if processdata is transmitted.
 * @see ecx_send_processdata_group
 */
int ec_send_processdata_group(uint8 group)
{
   return ecx_send_processdata_group (&ecx_context, group);
}

/** Transmit processdata to slaves.
* Uses LRW, or LRD/LWR if LRW is not allowed (blockLRW).
* Both the input and output processdata are transmitted in the overlapped IOmap.
* The outputs with the actual data, the inputs replace the output data in the
* returning frame. The inputs are gathered with the receive processdata function.
* In contrast to the base LRW function this function is non-blocking.
* If the processdata does not fit in one datagram, multiple are used.
* In order to recombine the slave response, a stack is used.
* @param[in]  group          = group number
* @return >0 if processdata is transmitted.
* @see ecx_send_overlap_processdata_group
*/
int ec_send_overlap_processdata_group(uint8 group)
{
   return ecx_send_overlap_processdata_group(&ecx_context, group);
}

/** Receive processdata from slaves.
 * Second part from ec_send_processdata().
 * Received datagrams are recombined with the processdata with help from the stack.
 * If a datagram contains input processdata it copies it to the processdata structure.
 * @param[in]  group          = group number
 * @param[in]  timeout        = Timeout in us.
 * @return Work counter.
 * @see ecx_receive_processdata_group
 */
int ec_receive_processdata_group(uint8 group, int timeout)
{
   return ecx_receive_processdata_group (&ecx_context, group, timeout);
}

int ec_send_processdata(void)
{
   return ec_send_processdata_group(0);
}

int ec_send_overlap_processdata(void)
{
   return ec_send_overlap_processdata_group(0);
}

int ec_receive_processdata(int timeout)
{
   return ec_receive_processdata_group(0, timeout);
}
#endif

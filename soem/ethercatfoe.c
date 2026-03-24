/*
 * Licensed under the GNU General Public License version 2 with exceptions. See
 * LICENSE file in the project root for full license information
 */

/** \file
 * \brief
 * File over EtherCAT (FoE) 模块。
 *
 * FoE（File over EtherCAT）是EtherCAT协议中用于文件传输的协议。
 * 本模块实现了FoE协议的读写功能，允许主站与从站之间传输文件。
 * 主要应用场景包括：
 * - 固件升级
 * - 配置文件下载/上传
 * - 从站存储器数据传输
 */

#include <stdio.h>
#include <string.h>
#include "osal.h"
#include "oshw.h"
#include "ethercattype.h"
#include "ethercatbase.h"
#include "ethercatmain.h"
#include "ethercatfoe.h"

/** FoE数据包最大数据长度 */
#define EC_MAXFOEDATA 512

/** FoE数据包结构体。
 * 用于Read（读请求）、Write（写请求）、Data（数据）、Ack（确认）和Error（错误）邮箱数据包。
 *
 * FoE协议定义了以下操作码：
 * - ECT_FOE_READ (1): 读文件请求
 * - ECT_FOE_WRITE (2): 写文件请求
 * - ECT_FOE_DATA (3): 数据传输
 * - ECT_FOE_ACK (4): 确认响应
 * - ECT_FOE_ERROR (5): 错误响应
 * - ECT_FOE_BUSY (6): 忙响应
 */
PACKED_BEGIN
typedef struct PACKED
{
   ec_mbxheadert MbxHeader;     /**< 邮箱头部，包含长度、地址、优先级、类型等信息 */
   uint8         OpCode;        /**< 操作码：读/写/数据/确认/错误/忙 */
   uint8         Reserved;      /**< 保留字段 */
   union
   {
      uint32        Password;   /**< 密码（用于读/写请求） */
      uint32        PacketNumber; /**< 数据包序号（用于数据/确认） */
      uint32 ErrorCode;           /**< 错误码（用于错误响应） */
      // TODO 忙响应 （done u16，entire u16）
   };
   union
   {
      char          FileName[EC_MAXFOEDATA]; /**< 文件名（用于读/写请求） */
      uint8         Data[EC_MAXFOEDATA];     /**< 数据内容（用于数据传输） */
      char          ErrorText[EC_MAXFOEDATA]; /**< 错误描述文本（用于错误响应） */
   };
} ec_FOEt;
PACKED_END

/** 定义FoE进度回调钩子函数。
 *
 * 钩子函数会在每个数据包传输完成后被调用，可用于：
 * - 显示传输进度
 * - 计算传输速度
 * - 实现传输取消功能
 *
 * @param[in] context = 上下文结构体
 * @param[in] hook    = 钩子函数指针，函数签名：int hook(uint16 slave, int packetnumber, int datasize)
 * @return 始终返回1
 */
int ecx_FOEdefinehook(ecx_contextt *context, void *hook)
{
  context->FOEhook = hook;
  return 1;
}

/** FoE读取文件，阻塞式。
 *
 * 从从站读取文件到主站缓冲区。该函数使用FoE协议通过邮箱通信
 * 从从站下载文件数据。传输过程是分段的，每个数据包最大不超过
 * 从站邮箱大小限制。
 *
 * 传输流程：
 * 1. 主站发送FoE读请求（包含文件名和密码）
 * 2. 从站响应FoE数据包（包含数据包序号和数据）
 * 3. 主站发送FoE确认（包含数据包序号）
 * 4. 重复步骤2-3直到所有数据传输完成
 *
 * @param[in]     context   = 上下文结构体
 * @param[in]     slave     = 从站号
 * @param[in]     filename  = 要读取的文件名
 * @param[in]     password  = 密码（如果不需要可设为0）
 * @param[in,out] psize     = 输入：文件缓冲区大小（字节）；输出：实际读取的字节数
 * @param[out]    p         = 文件缓冲区指针
 * @param[in]     timeout   = 每个邮箱周期的超时时间（微秒），标准值为EC_TIMEOUTRXM
 * @return >0: 成功，工作计数器值；<0: 错误码
 *         -EC_ERR_TYPE_FOE_BUF2SMALL: 缓冲区太小
 *         -EC_ERR_TYPE_FOE_ERROR: FoE错误
 *         -EC_ERR_TYPE_PACKET_ERROR: 数据包错误
 */
int ecx_FOEread(ecx_contextt *context, uint16 slave, char *filename, uint32 password, int *psize, void *p, int timeout)
{
   ec_FOEt *FOEp, *aFOEp;       /* FOEp: 发送数据包指针, aFOEp: 接收数据包指针 */
   int wkc;                      /* 工作计数器 */
   int32 dataread = 0;           /* 已读取的数据字节数 */
   int32 buffersize;             /* 缓冲区大小 */
   int32 packetnumber;           /* 当前数据包序号 */
   int32 prevpacket = 0;         /* 上一个数据包序号，用于验证顺序 */
   uint16 fnsize;                /* 文件名长度 */
   uint16 maxdata;               /* 每个数据包最大数据长度 */
   uint16 segmentdata;           /* 当前段的数据长度 */
   ec_mbxbuft MbxIn, MbxOut;     /* 邮箱输入/输出缓冲区 */
   uint8 cnt;                    /* 邮箱计数器值 */
   boolean worktodo;             /* 是否需要继续处理 */

   buffersize = *psize;          /* 保存缓冲区大小 */

   /* 清空输入邮箱缓冲区 */
   ec_clearmbx(&MbxIn);

   /* 清空从站输出邮箱（如果有数据），超时设为0表示不等待 */
   wkc = ecx_mbxreceive(context, slave, (ec_mbxbuft *)&MbxIn, 0);

   /* 清空输出邮箱缓冲区 */
   ec_clearmbx(&MbxOut);

   /* 设置数据包指针 */
   aFOEp = (ec_FOEt *)&MbxIn;    /* 接收数据包 */
   FOEp = (ec_FOEt *)&MbxOut;    /* 发送数据包 */

   /* 计算文件名长度，限制最大值 */
   fnsize = (uint16)strlen(filename);
   if (fnsize > EC_MAXFOEDATA)
   {
      fnsize = EC_MAXFOEDATA;
   }

   /* 计算每个数据包的最大数据长度（邮箱长度减去头部开销） */
   maxdata = context->slavelist[slave].mbx_l - 12;
   if (fnsize > maxdata)
   {
      fnsize = maxdata;
   }

   /* 构建FoE读请求数据包 */
   FOEp->MbxHeader.length = htoes(0x0006 + fnsize);  /* 长度：固定部分2+4字节 + 文件名长度 */
   FOEp->MbxHeader.address = htoes(0x0000);          /* 地址：保留 */
   FOEp->MbxHeader.priority = 0x00;                   /* 优先级：保留 */

   /* 获取新的邮箱计数器值，用作会话句柄 */
   cnt = ec_nextmbxcnt(context->slavelist[slave].mbx_cnt);
   context->slavelist[slave].mbx_cnt = cnt;
   FOEp->MbxHeader.mbxtype = ECT_MBXT_FOE + MBX_HDR_SET_CNT(cnt);  /* FoE类型 + 计数器 */

   FOEp->OpCode = ECT_FOE_READ;          /* 操作码：读请求 */
   FOEp->Password = htoel(password);     /* 密码 */

   /* 将文件名复制到邮箱数据区 */
   memcpy(&FOEp->FileName[0], filename, fnsize);

   /* 发送FoE读请求到从站 */
   wkc = ecx_mbxsend(context, slave, (ec_mbxbuft *)&MbxOut, EC_TIMEOUTTXM);

   if (wkc > 0)  /* 成功发送请求？ */
   {
      do
      {
         worktodo = FALSE;

         /* 清空输入邮箱缓冲区 */
         ec_clearmbx(&MbxIn);

         /* 读取从站响应 */
         wkc = ecx_mbxreceive(context, slave, (ec_mbxbuft *)&MbxIn, timeout);

         if (wkc > 0)  /* 成功读取响应？ */
         {
            /* 检查响应是否为FoE类型 */
            if ((aFOEp->MbxHeader.mbxtype & 0x0f) == ECT_MBXT_FOE)
            {
               if(aFOEp->OpCode == ECT_FOE_DATA)  /* 收到数据包 */
               {
                  /* 计算数据段长度（总长度减去6字节头部） */
                  segmentdata = etohs(aFOEp->MbxHeader.length) - 0x0006;
                  packetnumber = etohl(aFOEp->PacketNumber);

                  /* 验证数据包序号和缓冲区空间 */
                  if ((packetnumber == ++prevpacket) && (dataread + segmentdata <= buffersize))
                  {
                     /* 复制数据到用户缓冲区 */
                     memcpy(p, &aFOEp->Data[0], segmentdata);
                     dataread += segmentdata;
                     p = (uint8 *)p + segmentdata;  /* 移动缓冲区指针 */

                     /* 如果数据段等于最大长度，可能还有更多数据 */
                     if (segmentdata == maxdata)
                     {
                        worktodo = TRUE;
                     }

                     /* 构建并发送确认包 */
                     FOEp->MbxHeader.length = htoes(0x0006);
                     FOEp->MbxHeader.address = htoes(0x0000);
                     FOEp->MbxHeader.priority = 0x00;
                     cnt = ec_nextmbxcnt(context->slavelist[slave].mbx_cnt);
                     context->slavelist[slave].mbx_cnt = cnt;
                     FOEp->MbxHeader.mbxtype = ECT_MBXT_FOE + MBX_HDR_SET_CNT(cnt);
                     FOEp->OpCode = ECT_FOE_ACK;                    /* 操作码：确认 */
                     FOEp->PacketNumber = htoel(packetnumber);      /* 确认的数据包序号 */

                     /* 发送确认到从站 */
                     wkc = ecx_mbxsend(context, slave, (ec_mbxbuft *)&MbxOut, EC_TIMEOUTTXM);
                     if (wkc <= 0)
                     {
                        worktodo = FALSE;
                     }

                     /* 调用进度钩子函数（如果已注册） */
                     if (context->FOEhook)
                     {
                        context->FOEhook(slave, packetnumber, dataread);
                     }
                  }
                  else
                  {
                     /* 缓冲区太小或数据包序号错误 */
                     wkc = -EC_ERR_TYPE_FOE_BUF2SMALL;
                  }
               }
               else
               {
                  if(aFOEp->OpCode == ECT_FOE_ERROR)  /* 从站返回错误 */
                  {
                     wkc = -EC_ERR_TYPE_FOE_ERROR;
                  }
                  else
                  {
                     /* 收到意外的邮箱响应 */
                     wkc = -EC_ERR_TYPE_PACKET_ERROR;
                  }
               }
            }
            else
            {
               /* 收到非FoE类型的邮箱响应 */
               wkc = -EC_ERR_TYPE_PACKET_ERROR;
            }
            *psize = dataread;  /* 返回已读取的字节数 */
         }
      } while (worktodo);
   }

   return wkc;
}

/** FoE写入文件，阻塞式。
 *
 * 将文件数据从主站写入到从站。该函数使用FoE协议通过邮箱通信
 * 将文件数据上传到从站。传输过程是分段的，每个数据包最大不超过
 * 从站邮箱大小限制。
 *
 * 传输流程：
 * 1. 主站发送FoE写请求（包含文件名和密码）
 * 2. 从站响应FoE确认（包含数据包序号）
 * 3. 主站发送FoE数据包（包含数据包序号和数据）
 * 4. 重复步骤2-3直到所有数据传输完成
 * 5. EOF定义为数据包大小 < 最大数据包大小
 *
 * @param[in]  context   = 上下文结构体
 * @param[in]  slave     = 从站号
 * @param[in]  filename  = 要写入的文件名
 * @param[in]  password  = 密码（如果不需要可设为0）
 * @param[in]  psize     = 文件缓冲区大小（字节）
 * @param[out] p         = 文件缓冲区指针
 * @param[in]  timeout   = 每个邮箱周期的超时时间（微秒），标准值为EC_TIMEOUTRXM
 * @return >0: 成功，工作计数器值；<0: 错误码
 *         -EC_ERR_TYPE_FOE_PACKETNUMBER: 数据包序号错误
 *         -EC_ERR_TYPE_FOE_FILE_NOTFOUND: 文件未找到
 *         -EC_ERR_TYPE_FOE_ERROR: FoE错误
 *         -EC_ERR_TYPE_PACKET_ERROR: 数据包错误
 */
int ecx_FOEwrite(ecx_contextt *context, uint16 slave, char *filename, uint32 password, int psize, void *p, int timeout)
{
   ec_FOEt *FOEp, *aFOEp;       /* FOEp: 发送数据包指针, aFOEp: 接收数据包指针 */
   int wkc;                      /* 工作计数器 */
   int32 packetnumber;           /* 接收到的数据包序号 */
   int32 sendpacket = 0;         /* 已发送的数据包序号 */
   uint16 fnsize;                /* 文件名长度 */
   uint16 maxdata;               /* 每个数据包最大数据长度 */
   int segmentdata;              /* 当前段的数据长度 */
   ec_mbxbuft MbxIn, MbxOut;     /* 邮箱输入/输出缓冲区 */
   uint8 cnt;                    /* 邮箱计数器值 */
   boolean worktodo;             /* 是否需要继续处理 */
   boolean dofinalzero;          /* 是否需要发送最终零长度包 */
   int tsize;                    /* 临时数据大小 */

   /* 清空输入邮箱缓冲区 */
   ec_clearmbx(&MbxIn);

   /* 清空从站输出邮箱（如果有数据），超时设为0表示不等待 */
   wkc = ecx_mbxreceive(context, slave, (ec_mbxbuft *)&MbxIn, 0);

   /* 清空输出邮箱缓冲区 */
   ec_clearmbx(&MbxOut);

   /* 设置数据包指针 */
   aFOEp = (ec_FOEt *)&MbxIn;    /* 接收数据包 */
   FOEp = (ec_FOEt *)&MbxOut;    /* 发送数据包 */

   dofinalzero = TRUE;           /* 初始时需要发送零长度包（如果数据正好是最大长度的倍数） */

   /* 计算文件名长度，限制最大值 */
   fnsize = (uint16)strlen(filename);
   if (fnsize > EC_MAXFOEDATA)
   {
      fnsize = EC_MAXFOEDATA;
   }

   /* 计算每个数据包的最大数据长度（邮箱长度减去头部开销） */
   maxdata = context->slavelist[slave].mbx_l - 12;
   if (fnsize > maxdata)
   {
      fnsize = maxdata;
   }

   /* 构建FoE写请求数据包 */
   FOEp->MbxHeader.length = htoes(0x0006 + fnsize);  /* 长度：固定部分6字节 + 文件名长度 */
   FOEp->MbxHeader.address = htoes(0x0000);          /* 地址：保留 */
   FOEp->MbxHeader.priority = 0x00;                   /* 优先级：保留 */

   /* 获取新的邮箱计数器值，用作会话句柄 */
   cnt = ec_nextmbxcnt(context->slavelist[slave].mbx_cnt);
   context->slavelist[slave].mbx_cnt = cnt;
   FOEp->MbxHeader.mbxtype = ECT_MBXT_FOE + MBX_HDR_SET_CNT(cnt);  /* FoE类型 + 计数器 */

   FOEp->OpCode = ECT_FOE_WRITE;         /* 操作码：写请求 */
   FOEp->Password = htoel(password);     /* 密码 */

   /* 将文件名复制到邮箱数据区 */
   memcpy(&FOEp->FileName[0], filename, fnsize);

   /* 发送FoE写请求到从站 */
   wkc = ecx_mbxsend(context, slave, (ec_mbxbuft *)&MbxOut, EC_TIMEOUTTXM);

   if (wkc > 0)  /* 成功发送请求？ */
   {
      do
      {
         worktodo = FALSE;

         /* 清空输入邮箱缓冲区 */
         ec_clearmbx(&MbxIn);

         /* 读取从站响应 */
         wkc = ecx_mbxreceive(context, slave, (ec_mbxbuft *)&MbxIn, timeout);

         if (wkc > 0)  /* 成功读取响应？ */
         {
            /* 检查响应是否为FoE类型 */
            if ((aFOEp->MbxHeader.mbxtype & 0x0f) == ECT_MBXT_FOE)
            {
               switch (aFOEp->OpCode)
               {
                  case ECT_FOE_ACK:  /* 从站确认，可以发送下一数据包 */
                  {
                     packetnumber = etohl(aFOEp->PacketNumber);

                     /* 验证数据包序号 */
                     if (packetnumber == sendpacket)
                     {
                        /* 调用进度钩子函数（如果已注册） */
                        if (context->FOEhook)
                        {
                           context->FOEhook(slave, packetnumber, psize);
                        }

                        /* 计算本次要发送的数据大小 */
                        tsize = psize;
                        if (tsize > maxdata)
                        {
                           tsize = maxdata;
                        }

                        /* 如果还有数据或需要发送最终零长度包 */
                        if(tsize || dofinalzero)
                        {
                           worktodo = TRUE;
                           dofinalzero = FALSE;
                           segmentdata = tsize;
                           psize -= segmentdata;

                           /* 如果最后一个数据包是满的，需要添加零长度包作为结束标志 */
                           /* EOF定义为数据包大小 < 最大数据包大小 */
                           if (!psize && (segmentdata == maxdata))
                           {
                              dofinalzero = TRUE;
                           }

                           /* 构建FoE数据包 */
                           FOEp->MbxHeader.length = htoes((uint16)(0x0006 + segmentdata));
                           FOEp->MbxHeader.address = htoes(0x0000);
                           FOEp->MbxHeader.priority = 0x00;

                           /* 获取新的邮箱计数器值 */
                           cnt = ec_nextmbxcnt(context->slavelist[slave].mbx_cnt);
                           context->slavelist[slave].mbx_cnt = cnt;
                           FOEp->MbxHeader.mbxtype = ECT_MBXT_FOE + MBX_HDR_SET_CNT(cnt);

                           FOEp->OpCode = ECT_FOE_DATA;              /* 操作码：数据 */
                           sendpacket++;
                           FOEp->PacketNumber = htoel(sendpacket);    /* 数据包序号 */

                           /* 复制数据到邮箱 */
                           memcpy(&FOEp->Data[0], p, segmentdata);
                           p = (uint8 *)p + segmentdata;              /* 移动缓冲区指针 */

                           /* 发送FoE数据到从站 */
                           wkc = ecx_mbxsend(context, slave, (ec_mbxbuft *)&MbxOut, EC_TIMEOUTTXM);
                           if (wkc <= 0)
                           {
                              worktodo = FALSE;
                           }
                        }
                     }
                     else
                     {
                        /* 数据包序号错误 */
                        wkc = -EC_ERR_TYPE_FOE_PACKETNUMBER;
                     }
                     break;
                  }
                  case ECT_FOE_BUSY:  /* 从站忙，需要重发上一个数据包 */
                  {
                     /* 如果之前已发送数据，则重发 */
                     /* 否则忽略 */
                     if (sendpacket)
                     {
                        if (!psize)
                        {
                           dofinalzero = TRUE;
                        }
                        psize += segmentdata;           /* 恢复剩余数据大小 */
                        p = (uint8 *)p - segmentdata;   /* 回退缓冲区指针 */
                        --sendpacket;                    /* 回退数据包序号 */
                     }
                     break;
                  }
                  case ECT_FOE_ERROR:  /* 从站返回错误 */
                  {
                     if (aFOEp->ErrorCode == 0x8001)
                     {
                        wkc = -EC_ERR_TYPE_FOE_FILE_NOTFOUND;
                     }
                     else
                     {
                        wkc = -EC_ERR_TYPE_FOE_ERROR;
                     }
                     break;
                  }
                  default:  /* 未知的操作码 */
                  {
                     wkc = -EC_ERR_TYPE_PACKET_ERROR;
                     break;
                  }
               }
            }
            else
            {
               /* 收到非FoE类型的邮箱响应 */
               wkc = -EC_ERR_TYPE_PACKET_ERROR;
            }
         }
      } while (worktodo);
   }

   return wkc;
}

#ifdef EC_VER1
/** 定义FoE进度回调钩子函数（全局上下文版本）。
 *
 * 这是EC_VER1兼容性API，使用全局ecx_context上下文。
 *
 * @param[in] hook = 钩子函数指针
 * @return 始终返回1
 * @see ecx_FOEdefinehook
 */
int ec_FOEdefinehook(void *hook)
{
   return ecx_FOEdefinehook(&ecx_context, hook);
}

/** FoE读取文件（全局上下文版本）。
 *
 * 这是EC_VER1兼容性API，使用全局ecx_context上下文。
 *
 * @param[in]     slave     = 从站号
 * @param[in]     filename  = 要读取的文件名
 * @param[in]     password  = 密码
 * @param[in,out] psize     = 输入：缓冲区大小；输出：实际读取字节数
 * @param[out]    p         = 文件缓冲区指针
 * @param[in]     timeout   = 超时时间（微秒）
 * @return >0: 成功；<0: 错误码
 * @see ecx_FOEread
 */
int ec_FOEread(uint16 slave, char *filename, uint32 password, int *psize, void *p, int timeout)
{
   return ecx_FOEread(&ecx_context, slave, filename, password, psize, p, timeout);
}

/** FoE写入文件（全局上下文版本）。
 *
 * 这是EC_VER1兼容性API，使用全局ecx_context上下文。
 *
 * @param[in]  slave     = 从站号
 * @param[in]  filename  = 要写入的文件名
 * @param[in]  password  = 密码
 * @param[in]  psize     = 文件缓冲区大小
 * @param[out] p         = 文件缓冲区指针
 * @param[in]  timeout   = 超时时间（微秒）
 * @return >0: 成功；<0: 错误码
 * @see ecx_FOEwrite
 */
int ec_FOEwrite(uint16 slave, char *filename, uint32 password, int psize, void *p, int timeout)
{
   return ecx_FOEwrite(&ecx_context, slave, filename, password, psize, p, timeout);
}
#endif

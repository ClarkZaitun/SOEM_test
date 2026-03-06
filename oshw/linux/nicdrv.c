/*
 * Licensed under the GNU General Public License version 2 with exceptions. See
 * LICENSE file in the project root for full license information
 */

/** \file
 * \brief
 * EtherCAT RAW socket driver.
 *
 * Low level interface functions to send and receive EtherCAT packets.
 * EtherCAT has the property that packets are only send by the master,
 * and the send packets always return in the receive buffer.
 * There can be multiple packets "on the wire" before they return.
 * To combine the received packets with the original send packets a buffer
 * system is installed. The identifier is put in the index item of the
 * EtherCAT header. The index is stored and compared when a frame is received.
 * If there is a match the packet can be combined with the transmit packet
 * and returned to the higher level function.
 *
 * The socket layer can exhibit a reversal in the packet order (rare).
 * If the Tx order is A-B-C the return order could be A-C-B. The indexed buffer
 * will reorder the packets automatically.
 *
 * The "redundant" option will configure two sockets and two NIC interfaces.
 * Slaves are connected to both interfaces, one on the IN port and one on the
 * OUT port. Packets are send via both interfaces. Any one of the connections
 * (also an interconnect) can be removed and the slaves are still serviced with
 * packets. The software layer will detect the possible failure modes and
 * compensate. If needed the packets from interface A are resent through interface B.
 * This layer if fully transparent for the higher layers.
 */

#include <sys/types.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <sys/socket.h>
#include <unistd.h>
#include <sys/time.h>
#include <time.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <netpacket/packet.h>
#include <pthread.h>

#include "oshw.h"
#include "osal.h"

/** Redundancy modes */
enum
{
   /** No redundancy, single NIC mode */
   ECT_RED_NONE,
   /** Double redundant NIC connection */
   ECT_RED_DOUBLE
};


/** Primary source MAC address used for EtherCAT.
 * This address is not the MAC address used from the NIC.
 * EtherCAT does not care about MAC addressing, but it is used here to
 * differentiate the route the packet traverses through the EtherCAT
 * segment. This is needed to find out the packet flow in redundant
 * configurations. */
const uint16 priMAC[3] = { 0x0101, 0x0101, 0x0101 };
/** Secondary source MAC address used for EtherCAT. */
const uint16 secMAC[3] = { 0x0404, 0x0404, 0x0404 };

/** second MAC word is used for identification */
#define RX_PRIM priMAC[1]
/** second MAC word is used for identification */
#define RX_SEC secMAC[1]

static void ecx_clear_rxbufstat(int *rxbufstat)
{
   int i;
   for(i = 0; i < EC_MAXBUF; i++)
   {
      rxbufstat[i] = EC_BUF_EMPTY;
   }
}

/**
 * ecx_setupnic - 基本设置，将 NIC 连接到 socket
 * @port: 端口上下文结构体
 * @ifname: NIC 设备名称，例如 "eth0"
 * @secondary: 如果 >0，则使用冗余网卡而不是主栈
 *
 * 此函数设置网络接口卡 (NIC) 与 socket 的连接，
 * 配置必要的 socket 选项，并初始化缓冲区。
 *
 * 返回值：
 * - 如果成功，返回 >0
 * - 如果失败，返回 0
 */
int ecx_setupnic(ecx_portt *port, const char *ifname, int secondary)
{
   int i;
   int r, rval, ifindex;
   struct timeval timeout;
   struct ifreq ifr;
   struct sockaddr_ll sll;
   int *psock;
   pthread_mutexattr_t mutexattr;

   rval = 0;  /* 默认返回失败 */

   if (secondary)
   {
      /* 检查次级端口结构体是否可用 */
      if (port->redport)
      {
         /* 使用次级 socket 时，自动设置为冗余配置 */
         psock = &(port->redport->sockhandle);
         *psock = -1;  /* 初始化 socket 句柄为 -1 */
         port->redstate                   = ECT_RED_DOUBLE;  /* 设置为双重冗余模式 */
         port->redport->stack.sock        = &(port->redport->sockhandle);
         port->redport->stack.txbuf       = &(port->txbuf);
         port->redport->stack.txbuflength = &(port->txbuflength);
         port->redport->stack.tempbuf     = &(port->redport->tempinbuf);
         port->redport->stack.rxbuf       = &(port->redport->rxbuf);
         port->redport->stack.rxbufstat   = &(port->redport->rxbufstat);
         port->redport->stack.rxsa        = &(port->redport->rxsa);
         ecx_clear_rxbufstat(&(port->redport->rxbufstat[0]));  /* 清除冗余网卡的接收缓冲区状态 */
      }
      else
      {
         /* 次级端口不可用，失败 */
         return 0;
      }
   }
   else
   {
      /* 初始化互斥锁属性 */
      pthread_mutexattr_init(&mutexattr);
      pthread_mutexattr_setprotocol(&mutexattr  , PTHREAD_PRIO_INHERIT);
      /* 初始化各种互斥锁 */
      pthread_mutex_init(&(port->getindex_mutex), &mutexattr);
      pthread_mutex_init(&(port->tx_mutex)      , &mutexattr);
      pthread_mutex_init(&(port->rx_mutex)      , &mutexattr);

      port->sockhandle        = -1;  /* 初始化 socket 句柄为 -1 */
      port->lastidx           = 0;   /* 初始化最后使用的索引为 0 */
      port->redstate          = ECT_RED_NONE;  /* 设置为非冗余模式 */
      /* 配置主栈的各种指针 */
      port->stack.sock        = &(port->sockhandle);
      port->stack.txbuf       = &(port->txbuf);
      port->stack.txbuflength = &(port->txbuflength);
      port->stack.tempbuf     = &(port->tempinbuf);
      port->stack.rxbuf       = &(port->rxbuf);
      port->stack.rxbufstat   = &(port->rxbufstat);
      port->stack.rxsa        = &(port->rxsa);
      ecx_clear_rxbufstat(&(port->rxbufstat[0]));  /* 清除主栈的接收缓冲区状态 */
      psock = &(port->sockhandle);
   }

   /* 使用 RAW 数据包 socket，数据包类型为 ETH_P_ECAT */
   *psock = socket(PF_PACKET, SOCK_RAW, htons(ETH_P_ECAT));

   /* 设置 socket 超时时间 */
   timeout.tv_sec =  0;
   timeout.tv_usec = 1;
   r = setsockopt(*psock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
   r = setsockopt(*psock, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

   /* 设置 socket 选项，禁用路由 */
   i = 1;
   r = setsockopt(*psock, SOL_SOCKET, SO_DONTROUTE, &i, sizeof(i));

   /* 通过名称连接 socket 到 NIC */
   strcpy(ifr.ifr_name, ifname);
   r = ioctl(*psock, SIOCGIFINDEX, &ifr);
   ifindex = ifr.ifr_ifindex;

   /* 配置 NIC 接口标志 */
   strcpy(ifr.ifr_name, ifname);
   ifr.ifr_flags = 0;
   /* 重置 NIC 接口的标志 */
   r = ioctl(*psock, SIOCGIFFLAGS, &ifr);
   /* 设置 NIC 接口的标志，这里设置为混杂模式和广播模式 */
   ifr.ifr_flags = ifr.ifr_flags | IFF_PROMISC | IFF_BROADCAST;
   r = ioctl(*psock, SIOCSIFFLAGS, &ifr);

   /* 将 socket 绑定到协议，这里是 RAW EtherCAT */
   sll.sll_family = AF_PACKET;
   sll.sll_ifindex = ifindex;
   sll.sll_protocol = htons(ETH_P_ECAT);
   r = bind(*psock, (struct sockaddr *)&sll, sizeof(sll));

   /* 在发送缓冲区中设置以太网头部，这样就不需要重复设置 */
   for (i = 0; i < EC_MAXBUF; i++)
   {
      ec_setupheader(&(port->txbuf[i]));
      port->rxbufstat[i] = EC_BUF_EMPTY;
   }
   ec_setupheader(&(port->txbuf2));

   if (r == 0) rval = 1;  /* 如果绑定成功，设置返回值为 1 */

   return rval;
}

/**
 * ecx_closenic - 关闭使用的 socket
 * @port: 端口上下文结构体
 *
 * 关闭主端口和冗余端口（如果存在）的 socket。
 *
 * 返回值：
 * - 始终返回 0
 */
int ecx_closenic(ecx_portt *port)
{
   /* 关闭主端口的 socket（如果已打开） */
   if (port->sockhandle >= 0)
      close(port->sockhandle);

   /* 关闭冗余端口的 socket（如果存在且已打开） */
   if ((port->redport) && (port->redport->sockhandle >= 0))
      close(port->redport->sockhandle);

   return 0;
}

/**
 * ec_setupheader - 用以太网头部结构填充缓冲区
 * @p: 缓冲区指针
 *
 * 目标 MAC 地址始终是广播地址。
 * 以太网类型始终是 ETH_P_ECAT。
 */
void ec_setupheader(void *p)
{
   ec_etherheadert *bp;
   bp = p;  /* 将缓冲区指针转换为以太网头部指针 */

   /* 设置目标 MAC 地址为广播地址 (0xffff.ffff.ffff) */
   bp->da0 = htons(0xffff);
   bp->da1 = htons(0xffff);
   bp->da2 = htons(0xffff);

   /* 设置源 MAC 地址为主要 MAC 地址 */
   bp->sa0 = htons(priMAC[0]);
   bp->sa1 = htons(priMAC[1]);
   bp->sa2 = htons(priMAC[2]);

   /* 设置以太网类型为 ETH_P_ECAT */
   bp->etype = htons(ETH_P_ECAT);
}

/**
 * ecx_getindex - 获取新的帧标识符索引并分配相应的接收缓冲区
 * @port: 端口上下文结构体
 *
 * 此函数获取一个新的帧索引，用于标识 EtherCAT 帧，并标记相应的
 * 接收缓冲区为已分配状态。如果在冗余模式下，也会标记冗余端口的
 * 相应缓冲区。
 *
 * 返回值：
 * - 新分配的帧索引
 */
uint8 ecx_getindex(ecx_portt *port)
{
   uint8 idx;  /* 帧索引 */
   uint8 cnt;  /* 计数器，用于防止无限循环 */

   pthread_mutex_lock( &(port->getindex_mutex) );  /* 加锁以保护索引分配 */

   idx = port->lastidx + 1;  /* 从最后使用的索引加 1 开始 */
   /* 索引不能大于缓冲区数组大小 */
   if (idx >= EC_MAXBUF)
   {
      idx = 0;  /* 回绕到 0 */
   }

   cnt = 0;
   /* 尝试找到未使用的索引 */
   while ((port->rxbufstat[idx] != EC_BUF_EMPTY) && (cnt < EC_MAXBUF))
   {
      idx++;
      cnt++;
      if (idx >= EC_MAXBUF)
      {
         idx = 0;  /* 回绕到 0 */
      }
   }

   port->rxbufstat[idx] = EC_BUF_ALLOC;  /* 标记缓冲区为已分配 */

   /* 如果在冗余模式下，也标记冗余端口的相应缓冲区 */
   if (port->redstate != ECT_RED_NONE)
      port->redport->rxbufstat[idx] = EC_BUF_ALLOC;

   port->lastidx = idx;  /* 更新最后使用的索引 */

   pthread_mutex_unlock( &(port->getindex_mutex) );  /* 解锁 */

   return idx;  /* 返回新分配的索引 */
}

/**
 * ecx_setbufstat - 设置接收缓冲区状态
 * @port: 端口上下文结构体
 * @idx: 缓冲区数组中的索引
 * @bufstat: 要设置的状态
 *
 * 设置指定索引的接收缓冲区状态。如果在冗余模式下，
 * 也会设置冗余端口的相应缓冲区状态。
 */
void ecx_setbufstat(ecx_portt *port, uint8 idx, int bufstat)
{
   port->rxbufstat[idx] = bufstat;  /* 设置主端口的缓冲区状态 */

   /* 如果在冗余模式下，也设置冗余端口的相应缓冲区状态 */
   if (port->redstate != ECT_RED_NONE)
      port->redport->rxbufstat[idx] = bufstat;
}

/**
 * ecx_outframe - 通过 socket 传输缓冲区（非阻塞）
 * @port: 端口上下文结构体
 * @idx: 发送缓冲区数组中的索引
 * @stacknumber: 栈编号，0=主栈，1=冗余网卡
 *
 * 通过指定的栈（主栈或冗余网卡）发送缓冲区中的数据。
 *
 * 返回值：
 * - socket 发送结果
 * - 如果发送失败，返回 -1
 */
int ecx_outframe(ecx_portt *port, uint8 idx, int stacknumber)
{
   int lp, rval;
   ec_stackT *stack;

   /* 选择正确的栈（主栈或冗余网卡） */
   if (!stacknumber)
   {
      stack = &(port->stack);
   }
   else
   {
      stack = &(port->redport->stack);
   }

   lp = (*stack->txbuflength)[idx];  /* 获取发送缓冲区长度 */
   (*stack->rxbufstat)[idx] = EC_BUF_TX;  /* 将缓冲区状态标记为已发送 */

   /* 发送数据 */
   rval = send(*stack->sock, (*stack->txbuf)[idx], lp, 0);

   /* 如果发送失败，将缓冲区状态标记为空 */
   if (rval == -1)
   {
      (*stack->rxbufstat)[idx] = EC_BUF_EMPTY;
   }

   return rval;  /* 返回发送结果 */
}

/**
 * ecx_outframe_red - 通过 socket 传输缓冲区（非阻塞，支持冗余）
 * @port: 端口上下文结构体
 * @idx: 发送缓冲区数组中的索引
 *
 * 通过主栈发送缓冲区中的数据，并在冗余模式下通过冗余网卡发送一个 dummy 帧。
 *
 * 返回值：
 * - 主栈的 socket 发送结果
 */
int ecx_outframe_red(ecx_portt *port, uint8 idx)
{
   ec_comt *datagramP;
   ec_etherheadert *ehp;
   int rval;

   ehp = (ec_etherheadert *)&(port->txbuf[idx]);
   /* 将 MAC 源地址 1 重写为主 MAC */
   ehp->sa1 = htons(priMAC[1]);
   /* 通过主 socket 传输 */
   rval = ecx_outframe(port, idx, 0);

   /* 如果在冗余模式下 */
   if (port->redstate != ECT_RED_NONE)
   {
      pthread_mutex_lock( &(port->tx_mutex) );  /* 加锁以保护发送操作 */

      ehp = (ec_etherheadert *)&(port->txbuf2);
      /* 使用 dummy 帧进行次级 socket 传输（广播） */
      datagramP = (ec_comt*)&(port->txbuf2[ETH_HEADERSIZE]);
      /* 将索引写入帧 */
      datagramP->index = idx;
      /* 将 MAC 源地址 1 重写为次级 MAC */
      ehp->sa1 = htons(secMAC[1]);
      /* 通过次级 socket 传输 */
      port->redport->rxbufstat[idx] = EC_BUF_TX;

      if (send(port->redport->sockhandle, &(port->txbuf2), port->txbuflength2 , 0) == -1)
      {
         port->redport->rxbufstat[idx] = EC_BUF_EMPTY;
      }

      pthread_mutex_unlock( &(port->tx_mutex) );  /* 解锁 */
   }

   return rval;  /* 返回主栈的发送结果 */
}

/** Non blocking read of socket. Put frame in temporary buffer.
 * @param[in] port        = port context struct
 * @param[in] stacknumber = 0=primary 1=secondary stack
 * @return >0 if frame is available and read
 */
static int ecx_recvpkt(ecx_portt *port, int stacknumber)
{
   int lp, bytesrx;
   ec_stackT *stack;

   if (!stacknumber)
   {
      stack = &(port->stack);
   }
   else
   {
      stack = &(port->redport->stack);
   }
   lp = sizeof(port->tempinbuf);
   bytesrx = recv(*stack->sock, (*stack->tempbuf), lp, 0);
   port->tempinbufs = bytesrx;

   return (bytesrx > 0);
}

/** Non blocking receive frame function. Uses RX buffer and index to combine
 * read frame with transmitted frame. To compensate for received frames that
 * are out-of-order all frames are stored in their respective indexed buffer.
 * If a frame was placed in the buffer previously, the function retrieves it
 * from that buffer index without calling ec_recvpkt. If the requested index
 * is not already in the buffer it calls ec_recvpkt to fetch it. There are
 * three options now, 1 no frame read, so exit. 2 frame read but other
 * than requested index, store in buffer and exit. 3 frame read with matching
 * index, store in buffer, set completed flag in buffer status and exit.
 *
 * @param[in] port        = port context struct
 * @param[in] idx         = requested index of frame
 * @param[in] stacknumber = 0=primary 1=secondary stack
 * @return Workcounter if a frame is found with corresponding index, otherwise
 * EC_NOFRAME or EC_OTHERFRAME.
 */
/**
 * ecx_inframe - 检查并获取指定索引的 EtherCAT 帧
 * @port: EtherCAT 端口结构体指针
 * @idx: 要查找的帧索引
 * @stacknumber: 栈编号，0 表示主栈，非 0 表示冗余端口栈
 *
 * 该函数检查指定索引的 EtherCAT 帧是否已在缓冲区中，
 * 如果不在，则从网络接口接收数据包并处理。
 *
 * 返回值：
 * - EC_NOFRAME: 没有找到指定索引的帧
 * - EC_OTHERFRAME: 收到了其他索引的帧
 * - 其他值: 找到的帧的工作计数器 (WKC)
 */
int ecx_inframe(ecx_portt *port, uint8 idx, int stacknumber)
{
   uint16  l;            /* 帧长度变量 */
   int     rval;         /* 返回值，WKC 或 EC_NOFRAME/EC_OTHERFRAME */
   uint8   idxf;         /* 收到的帧的索引 */
   ec_etherheadert *ehp; /* 以太网头部指针 */
   ec_comt *ecp;         /* EtherCAT 通信数据指针 */
   ec_stackT *stack;     /* 栈指针 */
   ec_bufT *rxbuf;       /* 接收缓冲区指针 */

   /* 选择正确的栈（主栈或冗余端口栈） */
   if (!stacknumber)
   {
      stack = &(port->stack);
   }
   else
   {
      stack = &(port->redport->stack);
   }

   rval = EC_NOFRAME; /* 默认为未找到帧 */
   rxbuf = &(*stack->rxbuf)[idx]; /* 获取指定索引的接收缓冲区 */

   /* 检查请求的索引是否已在缓冲区中且状态为已接收 */
   if ((idx < EC_MAXBUF) && ((*stack->rxbufstat)[idx] == EC_BUF_RCVD))
   {
      /* 计算工作计数器 (WKC) 的位置 */
      l = (*rxbuf)[0] + ((uint16)((*rxbuf)[1] & 0x0f) << 8);
      /* 提取并返回 WKC */
      rval = ((*rxbuf)[l] + ((uint16)(*rxbuf)[l + 1] << 8));
      /* 将缓冲区状态标记为已完成 */
      (*stack->rxbufstat)[idx] = EC_BUF_COMPLETE;
   }
   else
   {
      /* 加锁以保护接收操作 */
      pthread_mutex_lock(&(port->rx_mutex));

      /* 非阻塞调用从 socket 接收数据包 */
      if (ecx_recvpkt(port, stacknumber))
      {
         rval = EC_OTHERFRAME; /* 默认为收到其他帧 */
         ehp =(ec_etherheadert*)(stack->tempbuf); /* 指向以太网头部 */

         /* 检查是否是 EtherCAT 帧 */
         if (ehp->etype == htons(ETH_P_ECAT))
         {
            /* 指向 EtherCAT 通信数据部分 */
            ecp =(ec_comt*)(&(*stack->tempbuf)[ETH_HEADERSIZE]);
            /* 提取 EtherCAT 帧长度 */
            l = etohs(ecp->elength) & 0x0fff;
            /* 提取帧索引 */
            idxf = ecp->index;

            /* 检查收到的帧索引是否与请求的索引匹配 */
            if (idxf == idx)
            {
               /* 匹配，将数据放入缓冲区（去除以太网头部） */
               memcpy(rxbuf, &(*stack->tempbuf)[ETH_HEADERSIZE], (*stack->txbuflength)[idx] - ETH_HEADERSIZE);
               /* 提取并返回 WKC */
               rval = ((*rxbuf)[l] + ((uint16)((*rxbuf)[l + 1]) << 8));
               /* 将缓冲区状态标记为已完成 */
               (*stack->rxbufstat)[idx] = EC_BUF_COMPLETE;
               /* 存储 MAC 源地址字 1 用于冗余路由信息 */
               (*stack->rxsa)[idx] = ntohs(ehp->sa1);
            }
            else
            {
               /* 不匹配，检查是否存在该索引且有人在等待 */
               if (idxf < EC_MAXBUF && (*stack->rxbufstat)[idxf] == EC_BUF_TX)
               {
                  /* 指向该索引的接收缓冲区 */
                  rxbuf = &(*stack->rxbuf)[idxf];
                  /* 将数据放入缓冲区（去除以太网头部） */
                  memcpy(rxbuf, &(*stack->tempbuf)[ETH_HEADERSIZE], (*stack->txbuflength)[idxf] - ETH_HEADERSIZE);
                  /* 将缓冲区状态标记为已接收 */
                  (*stack->rxbufstat)[idxf] = EC_BUF_RCVD;
                  /* 存储 MAC 源地址字 1 用于冗余路由信息 */
                  (*stack->rxsa)[idxf] = ntohs(ehp->sa1);
               }
               else
               {
                  /* 发生了奇怪的事情，不做处理 */
               }
            }
         }
      }

      /* 解锁接收操作 */
      pthread_mutex_unlock( &(port->rx_mutex) );

   }

   /* 如果找到匹配的帧，返回 WKC，否则返回状态码 */
   return rval;
}

/**
 * ecx_waitinframe_red - 阻塞式冗余接收帧函数
 * @port: EtherCAT 端口结构体指针
 * @idx: 请求的帧索引
 * @timer: 绝对超时时间
 *
 * 如果冗余模式未激活，则跳过冗余网卡和冗余功能。
 * 在冗余模式下，等待主栈和冗余网卡的帧到达。
 * 根据数据包的路由和可能的丢失情况，决定如何重新路由原始数据包以在其他尝试中获取数据。
 *
 * 返回值：
 * - 如果找到具有对应索引的帧，返回工作计数器 (WKC)
 * - 否则返回 EC_NOFRAME
 */
static int ecx_waitinframe_red(ecx_portt *port, uint8 idx, osal_timert *timer)
{
   osal_timert timer2;     /* 用于重发操作的定时器 */
   int wkc  = EC_NOFRAME;  /* 主栈的工作计数器 */
   int wkc2 = EC_NOFRAME;  /* 冗余网卡的工作计数器 */
   int primrx, secrx;      /* 主栈和冗余网卡接收到的 MAC 源地址 */

   /* 如果不在冗余模式下，始终假设冗余网卡正常 */
   if (port->redstate == ECT_RED_NONE)
      wkc2 = 0;

   do
   {
      /* 只有在尚未接收到帧时才读取 */
      if (wkc <= EC_NOFRAME)
         wkc  = ecx_inframe(port, idx, 0);  /* 从主栈接收帧 */

      /* 只有在冗余模式下才尝试冗余网卡 */
      if (port->redstate != ECT_RED_NONE)
      {
         /* 只有在尚未接收到帧时才读取 */
         if (wkc2 <= EC_NOFRAME)
            wkc2 = ecx_inframe(port, idx, 1);  /* 从冗余网卡接收帧 */
      }
   /* 等待两个帧到达或超时 */
   } while (((wkc <= EC_NOFRAME) || (wkc2 <= EC_NOFRAME)) && !osal_timer_is_expired(timer));

   /* 只有在冗余模式下才执行冗余功能 */
   if (port->redstate != ECT_RED_NONE)
   {
      /* 主栈接收到的 MAC 源地址 */
      primrx = 0;
      if (wkc > EC_NOFRAME) primrx = port->rxsa[idx];

      /* 冗余网卡接收到的 MAC 源地址 */
      secrx = 0;
      if (wkc2 > EC_NOFRAME) secrx = port->redport->rxsa[idx];

      /* 主栈接收到次级帧，冗余网卡接收到主帧 */
      /* 冗余模式下的正常情况 */
      if ( ((primrx == RX_SEC) && (secrx == RX_PRIM)) )
      {
         /* 将次级缓冲区复制到主缓冲区 */
         memcpy(&(port->rxbuf[idx]), &(port->redport->rxbuf[idx]), port->txbuflength[idx] - ETH_HEADERSIZE);
         wkc = wkc2;  /* 使用冗余网卡的工作计数器 */
      }

      /* 主栈未接收到帧或接收到主帧，且冗余网卡接收到次级帧 */
      /* 需要重新发送 TX 数据包 */
      if ( ((primrx == 0) && (secrx == RX_SEC)) ||
           ((primrx == RX_PRIM) && (secrx == RX_SEC)) )
      {
         /* 如果主栈和冗余网卡都有部分连接，通过次级套接字重新发送主栈接收到的帧 */
         /* 次级接收到的帧结果是一个按标准顺序遍历所有从站的组合帧 */
         if ( (primrx == RX_PRIM) && (secrx == RX_SEC) )
         {
            /* 将主栈接收数据复制到发送缓冲区 */
            memcpy(&(port->txbuf[idx][ETH_HEADERSIZE]), &(port->rxbuf[idx]), port->txbuflength[idx] - ETH_HEADERSIZE);
         }

         osal_timer_start (&timer2, EC_TIMEOUTRET);  /* 启动重发超时定时器 */
         /* 重新发送冗余网卡数据包 */
         ecx_outframe(port, idx, 1);

         do
         {
            /* 获取帧 */
            wkc2 = ecx_inframe(port, idx, 1);
         } while ((wkc2 <= EC_NOFRAME) && !osal_timer_is_expired(&timer2));

         if (wkc2 > EC_NOFRAME)
         {
            /* 将冗余网卡结果复制到主接收缓冲区 */
            memcpy(&(port->rxbuf[idx]), &(port->redport->rxbuf[idx]), port->txbuflength[idx] - ETH_HEADERSIZE);
            wkc = wkc2;  /* 使用冗余网卡的工作计数器 */
         }
      }
   }

   /* 返回 WKC 或 EC_NOFRAME */
   return wkc;
}

/**
 * ecx_waitinframe - 阻塞式接收帧函数
 * @port: EtherCAT 端口结构体指针
 * @idx: 请求的帧索引
 * @timeout: 超时时间（微秒）
 *
 * 调用 ecx_waitinframe_red() 函数来等待指定索引的帧到达。
 *
 * 返回值：
 * - 如果找到具有对应索引的帧，返回工作计数器 (WKC)
 * - 否则返回 EC_NOFRAME
 */
int ecx_waitinframe(ecx_portt *port, uint8 idx, int timeout)
{
   int wkc;         /* 工作计数器 */
   osal_timert timer;  /* 超时定时器 */

   osal_timer_start (&timer, timeout);  /* 启动超时定时器 */
   wkc = ecx_waitinframe_red(port, idx, &timer);  /* 调用冗余接收函数 */

   return wkc;  /* 返回工作计数器或 EC_NOFRAME */
}

/**
 * ecx_srconfirm - 阻塞式发送和接收帧函数
 * @port: EtherCAT 端口结构体指针
 * @idx: 帧的索引
 * @timeout: 超时时间（微秒）
 *
 * 用于非过程数据帧。数据报被构建到帧中并通过此函数传输。
 * 它等待应答并返回工作计数器。如果时间充足且结果为 WKC=0 或未接收到帧，函数会重试。
 *
 * 该函数调用 ecx_outframe_red() 和 ecx_waitinframe_red()。
 *
 * 返回值：
 * - 工作计数器 (WKC)
 * - 或 EC_NOFRAME
 */
int ecx_srconfirm(ecx_portt *port, uint8 idx, int timeout)
{
   int wkc = EC_NOFRAME;      /* 工作计数器 */
   osal_timert timer1, timer2; /* 定时器，timer1 用于总超时，timer2 用于单次接收超时 */

   osal_timer_start (&timer1, timeout);  /* 启动总超时定时器 */
   do
   {
      /* 在主栈上发送帧，如果在冗余模式下，在冗余网卡上发送一个 dummy 帧 */
      ecx_outframe_red(port, idx);

      if (timeout < EC_TIMEOUTRET)
      {
         osal_timer_start (&timer2, timeout);  /* 如果总超时小于默认重试超时，使用总超时 */
      }
      else
      {
         /* 通常使用部分超时进行接收 */
         osal_timer_start (&timer2, EC_TIMEOUTRET);  /* 使用默认重试超时 */
      }

      /* 从主栈获取帧，或在冗余模式下可能从冗余网卡获取 */
      wkc = ecx_waitinframe_red(port, idx, &timer2);
   /* 等待 WKC>=0 的应答，否则重试直到超时 */
   } while ((wkc <= EC_NOFRAME) && !osal_timer_is_expired (&timer1));

   return wkc;  /* 返回工作计数器或 EC_NOFRAME */
}

#ifdef EC_VER1
int ec_setupnic(const char *ifname, int secondary)
{
   return ecx_setupnic(&ecx_port, ifname, secondary);
}

int ec_closenic(void)
{
   return ecx_closenic(&ecx_port);
}

uint8 ec_getindex(void)
{
   return ecx_getindex(&ecx_port);
}

void ec_setbufstat(uint8 idx, int bufstat)
{
   ecx_setbufstat(&ecx_port, idx, bufstat);
}

int ec_outframe(uint8 idx, int stacknumber)
{
   return ecx_outframe(&ecx_port, idx, stacknumber);
}

int ec_outframe_red(uint8 idx)
{
   return ecx_outframe_red(&ecx_port, idx);
}

int ec_inframe(uint8 idx, int stacknumber)
{
   return ecx_inframe(&ecx_port, idx, stacknumber);
}

int ec_waitinframe(uint8 idx, int timeout)
{
   return ecx_waitinframe(&ecx_port, idx, timeout);
}

int ec_srconfirm(uint8 idx, int timeout)
{
   return ecx_srconfirm(&ecx_port, idx, timeout);
}
#endif

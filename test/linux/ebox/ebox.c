/** \file
 * \brief Example code for Simple Open EtherCAT master
 *
 * Usage : ebox [ifname] [cycletime]
 * ifname is NIC interface, f.e. eth0
 * cycletime in us, f.e. 500
 *
 * This test is specifically build for the E/BOX.
 *
 * (c)Arthur Ketels 2011
 */

#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <unistd.h>
#include <sched.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include <pthread.h>
#include <math.h>

#include "ethercat.h"

/** 定义纳秒到秒的转换常量 */
#define NSEC_PER_SEC 1000000000

/**
 * E/BOX 设备的输入数据结构体（标准模式）
 * 用于 EtherCAT 通信中的过程数据映射（PDO）
 */
typedef struct PACKED
{
   uint8         status;      /**< 设备状态 */
   uint8         counter;     /**< 计数器值 */
   uint8         din;         /**< 数字输入 */
   int32         ain[2];      /**< 模拟输入通道 */
   uint32        tsain;       /**< 模拟输入时间戳 */
   int32         enc[2];      /**< 编码器输入 */
} in_EBOXt;

/**
 * E/BOX 设备的输入数据结构体（流模式）
 * 用于高速数据采集
 */
typedef struct PACKED
{
   uint8         counter;     /**< 计数器值，用于检测数据更新 */
   int16         stream[100]; /**< 流数据缓冲区，存储高速采集的数据 */
} in_EBOX_streamt;

/**
 * E/BOX 设备的输出数据结构体（标准模式）
 * 用于 EtherCAT 通信中的过程数据映射（PDO）
 */
typedef struct PACKED
{
   uint8         control;     /**< 控制命令 */
   uint8         dout;        /**< 数字输出 */
   int16         aout[2];     /**< 模拟输出通道 */
   uint16        pwmout[2];   /**< PWM输出通道 */
} out_EBOXt;

/**
 * E/BOX 设备的输出数据结构体（流模式）
 * 用于高速数据采集模式下的控制
 */
typedef struct PACKED
{
   uint8         control;     /**< 控制命令 */
} out_EBOX_streamt;

/** 定义要采集的总采样数 */
#define MAXSTREAM 200000
/**
 * 采样间隔，单位为纳秒
 * 此处为 8 微秒 -> 125kHz 采样率
 * E/BOX v1.0.1 的最大数据速率约为 150kHz
 */
#define SYNC0TIME 8000

/** 调度参数结构体 */
struct sched_param schedp;
/** IO映射缓冲区，用于EtherCAT过程数据映射 */
char IOmap[4096];
/** 实时线程ID */
pthread_t thread1;
/** 时间结构体，用于测量时间 */
struct timeval tv,t1,t2;
/** 运行标志，控制EtherCAT通信线程 */
int dorun = 0;
/** 时间差和最大时间差 */
int deltat, tmax=0;
/** 偏移时间（纳秒），用于时间同步调整 */
int64 toff;
/** DC时间差 */
int DCdiff;
/** 变量大小 */
int os;
/** 通用32位变量 */
uint32 ob;
/** 通用16位变量 */
int16 ob2;
/** 通用8位变量 */
uint8 ob3;
/** 条件变量，用于线程同步 */
pthread_cond_t  cond  = PTHREAD_COND_INITIALIZER;
/** 互斥锁，用于保护共享资源 */
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
/** 时间同步算法积分值，用于PI控制器 */
int64 integral=0;
/** 循环计数器 */
uint32 cyclecount;
/** 指向E/BOX输入数据的指针（流模式） */
in_EBOX_streamt  *in_EBOX;
/** 指向E/BOX输出数据的指针（流模式） */
out_EBOX_streamt *out_EBOX;
/** 模拟输入平均值 */
double     ain[2];
/** 模拟输入计数 */
int        ainc;
/** 流数据位置指针 */
int        streampos;
/** 流数据通道1缓冲区 */
int16      stream1[MAXSTREAM];
/** 流数据通道2缓冲区 */
int16      stream2[MAXSTREAM];

/**
 * 将采集的流数据输出到CSV文件
 * @param fname 文件名
 * @param length 数据长度
 * @return 成功返回1，失败返回0
 */
int output_cvs(char *fname, int length)
{
   FILE *fp;
   int  i;

   // 打开文件
   fp = fopen(fname, "w");
   if(fp == NULL)
      return 0;

   // 写入数据到文件
   for (i = 0; i < length; i++)
   {
      fprintf(fp, "%d %d %d\n", i, stream1[i], stream2[i]);
   }

   // 关闭文件
   fclose(fp);

   return 1;
}

/**
 * E/BOX测试主函数
 * 初始化EtherCAT通信，配置从站，执行数据采集
 * @param ifname 网络接口名称
 */
void eboxtest(char *ifname)
{
   int cnt, i;

   printf("Starting E/BOX test\n");

   /* 绑定socket到指定网络接口 */
   if (ec_init(ifname))
   {
      printf("ec_init on %s succeeded.\n",ifname);

      /* 查找并自动配置从站 */
      if ( ec_config_init(FALSE) > 0 )
      {
         printf("%d slaves found and configured.\n",ec_slavecount);

         // 检查第一个从站是否为E/BOX
         if (( ec_slavecount >= 1 ) &&
             (strcmp(ec_slave[1].name,"E/BOX") == 0))
         {
            // 重新编程PDO映射，将从站设置为流模式
            // 这只能在PRE-OP状态下完成
            os=sizeof(ob2); ob2 = 0x1601; // 配置输入PDO映射
            ec_SDOwrite(1,0x1c12,01,FALSE,os,&ob2,EC_TIMEOUTRXM);
            os=sizeof(ob2); ob2 = 0x1a01; // 配置输出PDO映射
            ec_SDOwrite(1,0x1c13,01,FALSE,os,&ob2,EC_TIMEOUTRXM);
         }

         // 配置PDO映射
         ec_config_map(&IOmap);

         // 配置DC（分布式时钟）
         ec_configdc();

         /* 等待所有从站达到SAFE_OP状态 */
         ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE);

         /* 为列表中找到的每个支持DC的从站配置DC选项 */
         printf("DC capable : %d\n",ec_configdc());

         /* 检查配置 */
         if (( ec_slavecount >= 1 ) &&
             (strcmp(ec_slave[1].name,"E/BOX") == 0)
            )
         {
            printf("E/BOX found.\n");

            /* 将结构体指针连接到从站I/O指针 */
            in_EBOX = (in_EBOX_streamt*) ec_slave[1].inputs;
            out_EBOX = (out_EBOX_streamt*) ec_slave[1].outputs;

            /* 读取各个从站状态并存储在ec_slave[]中 */
            ec_readstate();
            for(cnt = 1; cnt <= ec_slavecount ; cnt++)
            {
               printf("Slave:%d Name:%s Output size:%3dbits Input size:%3dbits State:%2d delay:%d.%d\n",
                     cnt, ec_slave[cnt].name, ec_slave[cnt].Obits, ec_slave[cnt].Ibits,
                     ec_slave[cnt].state, (int)ec_slave[cnt].pdelay, ec_slave[cnt].hasdc);
            }
            printf("Request operational state for all slaves\n");

            /* 发送一个过程数据周期以初始化从站中的SM */
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);

            // 设置所有从站为OPERATIONAL状态
            ec_slave[0].state = EC_STATE_OPERATIONAL;
            /* 请求所有从站进入OP状态 */
            ec_writestate(0);
            /* 等待所有从站达到OP状态 */
            ec_statecheck(0, EC_STATE_OPERATIONAL,  EC_TIMEOUTSTATE);

            if (ec_slave[0].state == EC_STATE_OPERATIONAL )
            {
               printf("Operational state reached for all slaves.\n");
               ain[0] = 0;
               ain[1] = 0;
               ainc = 0;
               dorun = 1; // 启动EtherCAT通信线程

               // 等待Linux与DC同步
               usleep(100000);

               // 在从站1上启用SYNC0，设置同步时间为SYNC0TIME
               ec_dcsync0(1, TRUE, SYNC0TIME, 0);

               /* 20ms的非周期循环 */
               for(i = 1; i <= 200; i++)
               {
                  /* 读取从站2的DC差异寄存器 */
   //               ec_FPRD(ec_slave[1].configadr, ECT_REG_DCSYSDIFF, sizeof(DCdiff), &DCdiff, EC_TIMEOUTRET);
   //               if(DCdiff<0) { DCdiff = - (int32)((uint32)DCdiff & 0x7ffffff); }

                  // 打印过程数据
                  printf("PD cycle %5d DCtime %12lld Cnt:%3d Data: %6d %6d %6d %6d %6d %6d %6d %6d \n",
                        cyclecount, ec_DCtime, in_EBOX->counter, in_EBOX->stream[0], in_EBOX->stream[1],
                         in_EBOX->stream[2], in_EBOX->stream[3], in_EBOX->stream[4], in_EBOX->stream[5],
                         in_EBOX->stream[98], in_EBOX->stream[99]);

                  // 等待20ms
                  usleep(20000);
               }

               dorun = 0; // 停止EtherCAT通信线程
   //            printf("\nCnt %d : Ain0 = %f  Ain2 = %f\n", ainc, ain[0] / ainc, ain[1] / ainc);
            }
            else
            {
               printf("Not all slaves reached operational state.\n");
            }
         }
         else
         {
            printf("E/BOX not found in slave configuration.\n");
         }

         // 关闭SYNC0
         ec_dcsync0(1, FALSE, 8000, 0);

         // 请求所有从站进入SAFE_OP状态
         printf("Request safe operational state for all slaves\n");
         ec_slave[0].state = EC_STATE_SAFE_OP;
         ec_writestate(0);
         ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE);

         // 请求所有从站进入PRE_OP状态
         ec_slave[0].state = EC_STATE_PRE_OP;
         ec_writestate(0);
         ec_statecheck(0, EC_STATE_PRE_OP,  EC_TIMEOUTSTATE);

         // 恢复PDO到标准模式
         if (( ec_slavecount >= 1 ) &&
             (strcmp(ec_slave[1].name,"E/BOX") == 0))
         {
            // 恢复PDO到标准模式
            // 这只能在pre-op状态下完成
            os=sizeof(ob2); ob2 = 0x1600; // 恢复输入PDO映射
            ec_SDOwrite(1,0x1c12,01,FALSE,os,&ob2,EC_TIMEOUTRXM);
            os=sizeof(ob2); ob2 = 0x1a00; // 恢复输出PDO映射
            ec_SDOwrite(1,0x1c13,01,FALSE,os,&ob2,EC_TIMEOUTRXM);
         }

         // 输出流数据位置和数据
         printf("Streampos %d\n", streampos);
         output_cvs("stream.txt", streampos);
      }
      else
      {
         printf("No slaves found!\n");
      }

      printf("End E/BOX, close socket\n");
      /* 停止SOEM，关闭socket */
      ec_close();
   }
   else
   {
      printf("No socket connection on %s\nExcecute as root\n",ifname);
   }
}

/**
 * 向timespec结构体添加纳秒时间
 * @param ts timespec结构体指针
 * @param addtime 要添加的纳秒数
 */
void add_timespec(struct timespec *ts, int64 addtime)
{
   int64 sec, nsec;

   // 计算秒和纳秒部分
   nsec = addtime % NSEC_PER_SEC;
   sec = (addtime - nsec) / NSEC_PER_SEC;

   // 更新timespec结构体
   ts->tv_sec += sec;
   ts->tv_nsec += nsec;

   // 处理纳秒溢出
   if ( ts->tv_nsec >= NSEC_PER_SEC )
   {
      nsec = ts->tv_nsec % NSEC_PER_SEC;
      ts->tv_sec += (ts->tv_nsec - nsec) / NSEC_PER_SEC;
      ts->tv_nsec = nsec;
   }
}

/**
 * PI控制器计算，使Linux时间与DC时间同步
 * @param reftime 从站参考时钟时间（DC时间），单位为纳秒
 * @param cycletime 循环周期时间，单位为纳秒
 * @param offsettime 输出参数，用于存储计算得到的时间偏移量，单位为纳秒
 */
void ec_sync(int64 reftime, int64 cycletime , int64 *offsettime)
{
   int64 delta; // 存储时间差值

   /* 设置Linux同步点比DC同步点晚50us，仅作为示例 */
   // 计算参考时间减去50微秒后相对于循环周期的余数
   delta = (reftime - 50000) % cycletime;

   // 将delta调整到[-cycletime/2, cycletime/2]范围内，确保相位差最小
   if(delta> (cycletime /2)) { delta= delta - cycletime; }

   // 积分项累加：如果delta为正，积分增加；如果delta为负，积分减少
   // 这是PI控制器的积分部分，用于消除稳态误差
   if(delta>0){ integral++; }
   if(delta<0){ integral--; }

   // 计算输出的偏移时间，包含比例项和积分项
   // -(delta / 100) 是比例项，系数为1/100
   // (integral /20) 是积分项，系数为1/20
   *offsettime = -(delta / 100) - (integral /20);
}

/**
 * 实时EtherCAT线程
 * 负责周期性发送和接收EtherCAT过程数据
 * @param ptr 指向周期时间的指针（微秒）
 */
void ecatthread( void *ptr )
{
   struct timespec   ts;     // 用于指定线程唤醒时间的结构体，精确到纳秒
   struct timeval    tp;     // 主站当前时间结构体
   int ht;                   // 用于存储毫秒值
   int i;                    // 循环计数器
   int pcounter = 0;         // 主站内的计数器，用于跟踪EBOX计数器的变化
   int64 cycletime;          // 循环周期时间（纳秒）

   // 锁定互斥量，保护共享资源
   pthread_mutex_lock(&mutex);

   // 获取当前的系统时间，精确到微秒级别
   gettimeofday(&tp, NULL);

   /* 将timeval格式转换为timespec格式 */
   ts.tv_sec  = tp.tv_sec;
   // 将微秒转换为毫秒并向上取整
   ht = (tp.tv_usec / 1000) + 1; /* round to nearest ms */
   // 转换为纳秒
   ts.tv_nsec = ht * 1000000;

   // 将周期时间从微秒转换为纳秒
   cycletime = *(int*)ptr * 1000; /* cycletime in ns */

   // 重置时间偏移量
   toff = 0;
   // 重置运行标志
   dorun = 0;

   // 主循环
   while(1)
   {
      // 计算下一个周期开始时间（累加时间 ts 与周期时间 cycletime，同步算法调整值 toff 之和）
      /* calculate next cycle start */
      add_timespec(&ts, cycletime + toff);

      // 等待到计算出的周期开始时间
      // 条件变量定时等待：让当前线程等待条件变量 cond 被信号唤醒，或者直到指定的时间 ts 到达
      pthread_cond_timedwait(&cond, &mutex, &ts);

      // 检查是否需要执行EtherCAT通信
      if (dorun>0)
      {
         // 再次获取当前时间
         gettimeofday(&tp, NULL);

         // 发送Group 0过程数据到EtherCAT从站
         ec_send_processdata();

         // 接收来自EtherCAT从站的过程数据
         ec_receive_processdata(EC_TIMEOUTRET);

         // 增加循环计数器
         cyclecount++;

         // 检查是否有新的流数据可用且缓冲区未满
         if((in_EBOX->counter != pcounter) && (streampos < (MAXSTREAM - 1)))
         {
            // 检查主站是否存在定时问题
            // 如果是，则覆盖流数据以便在图表中明显显示
            if(in_EBOX->counter > (pcounter + 1))
            {
               // 检测到主站定时问题，记录异常值
               for(i = 0 ; i < 50 ; i++)
               {
                  // 使用明显的极值标记定时问题
                  stream1[streampos]   = 20000;
                  stream2[streampos++] = -20000;
               }
            }
            else
            {
               // 正常情况：复制流数据到缓冲区
               for(i = 0 ; i < 50 ; i++)
               {
                  // 交替复制两个通道的数据
                  stream1[streampos]   = in_EBOX->stream[i * 2];
                  stream2[streampos++] = in_EBOX->stream[(i * 2) + 1];
               }
            }
            // 更新上次的计数器值
            pcounter = in_EBOX->counter;
         }

         /* 计算时间偏移量以同步Linux时间与DC时间 */
         ec_sync(ec_DCtime, cycletime, &toff);
      }
   }
}

/**
 * 主函数
 * @param argc 命令行参数数量
 * @param argv 命令行参数数组
 * @return 0表示成功
 */
int main(int argc, char *argv[])
{
   int ctime;                // 周期时间（微秒）
   struct sched_param    param; // 线程调度参数
   int                   policy = SCHED_OTHER; // 调度策略

   printf("SOEM (Simple Open EtherCAT Master)\nE/BOX test\n");

   // 初始化调度参数
   memset(&schedp, 0, sizeof(schedp));
   /* 不要将优先级设置高于49，否则socket会被饿死 */
   schedp.sched_priority = 30;
   // 设置当前线程为FIFO调度策略
   sched_setscheduler(0, SCHED_FIFO, &schedp);

   // 等待dorun标志变为0
   do
   {
      usleep(1000);
   }
   while (dorun);

   // 检查命令行参数
   if (argc > 1)
   {
      dorun = 1;
      // 获取周期时间参数
      if( argc > 2)
         ctime = atoi(argv[2]);
      else
         ctime = 1000; // 默认1ms周期时间

      /* 创建实时线程 */
      pthread_create( &thread1, NULL, (void *) &ecatthread, (void*) &ctime);

      // 设置线程优先级
      memset(&param, 0, sizeof(param));
      /* 优先级 40 */
      param.sched_priority = 40;
      pthread_setschedparam(thread1, policy, &param);

      /* 启动非周期部分 */
      eboxtest(argv[1]);
   }
   else
   {
      // 显示用法
      printf("Usage: ebox ifname [cycletime]\nifname = eth0 for example\ncycletime in us\n");
   }

   // 恢复默认调度策略
   schedp.sched_priority = 0;
   sched_setscheduler(0, SCHED_OTHER, &schedp);

   printf("End program\n");

   return (0);
}

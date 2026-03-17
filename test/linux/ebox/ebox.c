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

#define NSEC_PER_SEC 1000000000

// E/BOX 设备的输入输出数据结构体，用于 EtherCAT 通信中的过程数据映射（PDO）
typedef struct PACKED
{
   uint8         status;
   uint8         counter;
   uint8         din;
   int32         ain[2];
   uint32        tsain;
   int32         enc[2];
} in_EBOXt;

typedef struct PACKED
{
   uint8         counter;
   int16         stream[100];
} in_EBOX_streamt;

typedef struct PACKED
{
   uint8         control;
   uint8         dout;
   int16         aout[2];
   uint16        pwmout[2];
} out_EBOXt;

typedef struct PACKED
{
   uint8         control;
} out_EBOX_streamt;

// 要采集的总采样数
#define MAXSTREAM 200000
// 采样间隔，单位为纳秒，此处为 8 微秒 -> 125kHz
// E/BOX v1.0.1 的最大数据速率约为 150kHz
#define SYNC0TIME 8000

struct sched_param schedp;
char IOmap[4096];
pthread_t thread1;
struct timeval tv,t1,t2;
int dorun = 0;
int deltat, tmax=0;
int64 toff;  // 偏移时间（纳秒）
int DCdiff;
int os;
uint32 ob;
int16 ob2;
uint8 ob3;
pthread_cond_t  cond  = PTHREAD_COND_INITIALIZER;
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
// 时间同步算法积分值
int64 integral=0;
uint32 cyclecount;
in_EBOX_streamt  *in_EBOX;
out_EBOX_streamt *out_EBOX;
double     ain[2];
int        ainc;
int        streampos;
int16      stream1[MAXSTREAM];
int16      stream2[MAXSTREAM];

int output_cvs(char *fname, int length)
{
   FILE *fp;

   int  i;

   fp = fopen(fname, "w");
   if(fp == NULL)
      return 0;
   for (i = 0; i < length; i++)
   {
      fprintf(fp, "%d %d %d\n", i, stream1[i], stream2[i]);
   }
   fclose(fp);

   return 1;
}

// 初始化流程
void eboxtest(char *ifname)
{
   int cnt, i;

   printf("Starting E/BOX test\n");

   /* initialise SOEM, bind socket to ifname */
   if (ec_init(ifname))
   {
      printf("ec_init on %s succeeded.\n",ifname);
      /* find and auto-config slaves */
      if ( ec_config_init(FALSE) > 0 )
      {
         printf("%d slaves found and configured.\n",ec_slavecount);

         // check if first slave is an E/BOX
         if (( ec_slavecount >= 1 ) &&
             (strcmp(ec_slave[1].name,"E/BOX") == 0))
         {
            // reprogram PDO mapping to set slave in stream mode
            // this can only be done in pre-OP state
            os=sizeof(ob2); ob2 = 0x1601;
            ec_SDOwrite(1,0x1c12,01,FALSE,os,&ob2,EC_TIMEOUTRXM);
            os=sizeof(ob2); ob2 = 0x1a01;
            ec_SDOwrite(1,0x1c13,01,FALSE,os,&ob2,EC_TIMEOUTRXM);
         }

         ec_config_map(&IOmap);

         // 配置 DC
         ec_configdc();

         /* wait for all slaves to reach SAFE_OP state */
         ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE);

         /* configure DC options for every DC capable slave found in the list */
         printf("DC capable : %d\n",ec_configdc());

         /* check configuration */
         if (( ec_slavecount >= 1 ) &&
             (strcmp(ec_slave[1].name,"E/BOX") == 0)
            )
         {
            printf("E/BOX found.\n");

            /* connect struct pointers to slave I/O pointers */
            in_EBOX = (in_EBOX_streamt*) ec_slave[1].inputs;
            out_EBOX = (out_EBOX_streamt*) ec_slave[1].outputs;

            /* read indevidual slave state and store in ec_slave[] */
            ec_readstate();
            for(cnt = 1; cnt <= ec_slavecount ; cnt++)
            {
               printf("Slave:%d Name:%s Output size:%3dbits Input size:%3dbits State:%2d delay:%d.%d\n",
                     cnt, ec_slave[cnt].name, ec_slave[cnt].Obits, ec_slave[cnt].Ibits,
                     ec_slave[cnt].state, (int)ec_slave[cnt].pdelay, ec_slave[cnt].hasdc);
            }
            printf("Request operational state for all slaves\n");

            /* send one processdata cycle to init SM in slaves */
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);

            ec_slave[0].state = EC_STATE_OPERATIONAL;
            /* request OP state for all slaves */
            ec_writestate(0);
            /* wait for all slaves to reach OP state */
            ec_statecheck(0, EC_STATE_OPERATIONAL,  EC_TIMEOUTSTATE);
            if (ec_slave[0].state == EC_STATE_OPERATIONAL )
            {
               printf("Operational state reached for all slaves.\n");
               ain[0] = 0;
               ain[1] = 0;
               ainc = 0;
               dorun = 1;
               usleep(100000); // wait for linux to sync on DC
               ec_dcsync0(1, TRUE, SYNC0TIME, 0); // SYNC0 on slave 1
               /* acyclic loop 20ms */
               for(i = 1; i <= 200; i++)
               {
                  /* read DC difference register for slave 2 */
   //               ec_FPRD(ec_slave[1].configadr, ECT_REG_DCSYSDIFF, sizeof(DCdiff), &DCdiff, EC_TIMEOUTRET);
   //               if(DCdiff<0) { DCdiff = - (int32)((uint32)DCdiff & 0x7ffffff); }
                  printf("PD cycle %5d DCtime %12lld Cnt:%3d Data: %6d %6d %6d %6d %6d %6d %6d %6d \n",
                        cyclecount, ec_DCtime, in_EBOX->counter, in_EBOX->stream[0], in_EBOX->stream[1],
                         in_EBOX->stream[2], in_EBOX->stream[3], in_EBOX->stream[4], in_EBOX->stream[5],
                         in_EBOX->stream[98], in_EBOX->stream[99]);
                  usleep(20000);
               }
               dorun = 0;
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
         ec_dcsync0(1, FALSE, 8000, 0); // SYNC0 off
         printf("Request safe operational state for all slaves\n");
         ec_slave[0].state = EC_STATE_SAFE_OP;
         /* request SAFE_OP state for all slaves */
         ec_writestate(0);
         /* wait for all slaves to reach state */
         ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE);
         ec_slave[0].state = EC_STATE_PRE_OP;
         /* request SAFE_OP state for all slaves */
         ec_writestate(0);
         /* wait for all slaves to reach state */
         ec_statecheck(0, EC_STATE_PRE_OP,  EC_TIMEOUTSTATE);
         if (( ec_slavecount >= 1 ) &&
             (strcmp(ec_slave[1].name,"E/BOX") == 0))
         {
            // restore PDO to standard mode
            // this can only be done is pre-op state
            os=sizeof(ob2); ob2 = 0x1600;
            ec_SDOwrite(1,0x1c12,01,FALSE,os,&ob2,EC_TIMEOUTRXM);
            os=sizeof(ob2); ob2 = 0x1a00;
            ec_SDOwrite(1,0x1c13,01,FALSE,os,&ob2,EC_TIMEOUTRXM);
         }
         printf("Streampos %d\n", streampos);
         output_cvs("stream.txt", streampos);
      }
      else
      {
         printf("No slaves found!\n");
      }
      printf("End E/BOX, close socket\n");
      /* stop SOEM, close socket */
      ec_close();
   }
   else
   {
      printf("No socket connection on %s\nExcecute as root\n",ifname);
   }
}

/* add ns to timespec */
void add_timespec(struct timespec *ts, int64 addtime)
{
   int64 sec, nsec;

   nsec = addtime % NSEC_PER_SEC;
   sec = (addtime - nsec) / NSEC_PER_SEC;
   ts->tv_sec += sec;
   ts->tv_nsec += nsec;
   if ( ts->tv_nsec >= NSEC_PER_SEC )
   {
      nsec = ts->tv_nsec % NSEC_PER_SEC;
      ts->tv_sec += (ts->tv_nsec - nsec) / NSEC_PER_SEC;
      ts->tv_nsec = nsec;
   }
}

/* PI calculation to get linux time synced to DC time */
// 实现PI控制器，使Linux时间与DC时间同步
// 输入：
// reftime：从站参考时钟时间（DC时间），单位为纳秒
// cycletime：循环周期时间，单位为纳秒
// offsettime：输出参数，用于存储计算得到的时间偏移量，单位为纳秒
// 输出：
// offsettime：根据PI控制器计算得到的时间偏移量，单位为纳秒
void ec_sync(int64 reftime, int64 cycletime , int64 *offsettime)
{
   int64 delta; // 存储时间差值
   /* set linux sync point 50us later than DC sync, just as example */
   // 计算参考时间减去50微秒后相对于循环周期的余数
   // 50000ns = 50us，这里设置了Linux同步点比DC同步点晚50微秒
   // TODO 50us的来源是什么？
   delta = (reftime - 50000) % cycletime;

   // 将delta调整到[-cycletime/2, cycletime/2]范围内，确保相位差最小
   // 如果delta大于半周期，则减去一个完整周期，使其成为负值
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

/* RT EtherCAT thread */
// 参数为指向周期时间的指针
void ecatthread( void *ptr )
{
   struct timespec   ts; // 收发帧累计时间结构体，用于指定线程唤醒时间，从 tp 计算得到，精确到纳秒
   struct timeval    tp; // 主站当前时间结构体
   int ht;                         // 用于存储毫秒值
   int i;                          // 循环计数器
   int pcounter = 0;               // 主站内的计数器，用于跟踪EBOX计数器的变化
   int64 cycletime;                // 循环周期时间（纳秒）

   // 锁定互斥量，保护共享资源
   pthread_mutex_lock(&mutex);
   // 获取当前的系统时间，精确到微秒级别。
   // osal_current_time() 函数获取当前系统时间里面的时间也是（秒和微秒）
   gettimeofday(&tp, NULL);

    /* Convert from timeval to timespec */
   // 将timeval格式转换为timespec格式
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
      // 条件变量定时等待：让当前线程等待条件变量 cond 被信号唤醒，或者直到指定的时间 ts 到达。
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

         /* calulate toff to get linux time and DC synced */
         // 计算时间偏移量以同步Linux时间与DC时间
         ec_sync(ec_DCtime, cycletime, &toff);
      }
   }
}

int main(int argc, char *argv[])
{
   int ctime;
   struct sched_param    param;
   int                   policy = SCHED_OTHER;

   printf("SOEM (Simple Open EtherCAT Master)\nE/BOX test\n");

   memset(&schedp, 0, sizeof(schedp));
   /* do not set priority above 49, otherwise sockets are starved */
   schedp.sched_priority = 30;
   sched_setscheduler(0, SCHED_FIFO, &schedp);

   do
   {
      usleep(1000);
   }
   while (dorun);

   if (argc > 1)
   {
      dorun = 1;
      if( argc > 2)
         ctime = atoi(argv[2]);
      else
         ctime = 1000; // 1ms cycle time
      /* create RT thread */
      pthread_create( &thread1, NULL, (void *) &ecatthread, (void*) &ctime);
      memset(&param, 0, sizeof(param));
      /* give it higher priority */
      param.sched_priority = 40;
      pthread_setschedparam(thread1, policy, &param);

      /* start acyclic part */
      eboxtest(argv[1]);
   }
   else
   {
      printf("Usage: ebox ifname [cycletime]\nifname = eth0 for example\ncycletime in us\n");
   }

   schedp.sched_priority = 0;
   sched_setscheduler(0, SCHED_OTHER, &schedp);

   printf("End program\n");

   return (0);
}

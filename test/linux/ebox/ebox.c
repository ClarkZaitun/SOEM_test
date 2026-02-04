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

// total samples to capture
#define MAXSTREAM 200000
// sample interval in ns, here 8us -> 125kHz
// maximum data rate for E/BOX v1.0.1 is around 150kHz
#define SYNC0TIME 8000

struct sched_param schedp;
char IOmap[4096];
pthread_t thread1;
struct timeval tv,t1,t2;
int dorun = 0;
int deltat, tmax=0;
int64 toff;
int DCdiff;
int os;
uint32 ob;
int16 ob2;
uint8 ob3;
pthread_cond_t  cond  = PTHREAD_COND_INITIALIZER;
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
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
void ec_sync(int64 reftime, int64 cycletime , int64 *offsettime)
{
   int64 delta; // 存储时间差值
   /* set linux sync point 50us later than DC sync, just as example */
   // 计算参考时间减去50微秒后相对于循环周期的余数
   // 50000ns = 50us，这里设置了Linux同步点比DC同步点晚50微秒
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
   struct timespec   ts;
   struct timeval    tp;
   int ht;
   int i;
   int pcounter = 0;
   int64 cycletime;

   pthread_mutex_lock(&mutex);
   gettimeofday(&tp, NULL);

    /* Convert from timeval to timespec */
   ts.tv_sec  = tp.tv_sec;
   ht = (tp.tv_usec / 1000) + 1; /* round to nearest ms */
   ts.tv_nsec = ht * 1000000;
   cycletime = *(int*)ptr * 1000; /* cycletime in ns */
   toff = 0;
   dorun = 0;
   while(1)
   {
      // 计算下一个周期开始时间
      /* calculate next cycle start */
      add_timespec(&ts, cycletime + toff);
      // 等待到周期开始时间
      pthread_cond_timedwait(&cond, &mutex, &ts);
      if (dorun>0)
      {
         gettimeofday(&tp, NULL);

         ec_send_processdata();

         ec_receive_processdata(EC_TIMEOUTRET);

         cyclecount++;


         if((in_EBOX->counter != pcounter) && (streampos < (MAXSTREAM - 1)))
         {
            // check if we have timing problems in master
            // if so, overwrite stream data so it shows up clearly in plots.
            if(in_EBOX->counter > (pcounter + 1))
            {
               for(i = 0 ; i < 50 ; i++)
               {
                  stream1[streampos]   = 20000;
                  stream2[streampos++] = -20000;
               }
            }
            else
            {
               for(i = 0 ; i < 50 ; i++)
               {
                  stream1[streampos]   = in_EBOX->stream[i * 2];
                  stream2[streampos++] = in_EBOX->stream[(i * 2) + 1];
               }
            }
            pcounter = in_EBOX->counter;
         }

         /* calulate toff to get linux time and DC synced */
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

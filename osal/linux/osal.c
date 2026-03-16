/*
 * Licensed under the GNU General Public License version 2 with exceptions. See
 * LICENSE file in the project root for full license information
 */

#include <time.h>
#include <sys/time.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <osal.h>

#define USECS_PER_SEC     1000000

int osal_usleep (uint32 usec)
{
   struct timespec ts;
   ts.tv_sec = usec / USECS_PER_SEC;
   ts.tv_nsec = (usec % USECS_PER_SEC) * 1000;
   /* usleep is deprecated, use nanosleep instead */
   return nanosleep(&ts, NULL);
}

ec_timet osal_current_time(void)
{
   struct timespec current_time;
   ec_timet return_value;

   //  CLOCK_REALTIME（实时时间）
   //  含义：
   //  表示系统的实际日期和时间（即墙钟时间），与 Unix 时间戳一致。它会受到系统时间调整的影响（如 NTP 同步、手动修改时间、时区变更等）。
   //  时间起点：
   //  1970-01-01 00:00:00 UTC（Unix 纪元时间）。
   //  返回值表示从该起点开始经过的秒数和纳秒数（类似 time_t的高精度版本）。
   //  特点：
   //      可被用户或系统管理员修改（向前/向后跳变）。
   //      可能受闰秒调整影响。
   //      适用于需要真实世界时间的场景（如日志记录、定时任务）。
   clock_gettime(CLOCK_REALTIME, &current_time);
   // CLOCK_MONOTONIC（单调时间）
   //  含义：
   //  表示一个单调递增的时钟，不受系统时间修改的影响。它始终向前推进（即使系统时间被回调或 NTP 调整）。
   //  时间起点：
   //  系统启动时的某个未指定时间点（通常接近开机时刻）。
   //  具体起点值不暴露给用户空间，且不同启动周期的值无关联（重启后重置）。
   //  特点：
   //      永不减少（即使系统挂起/休眠也可能暂停，取决于实现）。
   //      适合测量时间间隔（如性能分析、超时计算）。
   //      不代表真实日期时间（无法直接转换为日历时间）。
   return_value.sec = current_time.tv_sec;
   return_value.usec = current_time.tv_nsec / 1000;

   return return_value;
}

void osal_time_diff(ec_timet *start, ec_timet *end, ec_timet *diff)
{
   if (end->usec < start->usec) {
      diff->sec = end->sec - start->sec - 1;
      diff->usec = end->usec + 1000000 - start->usec;
   }
   else {
      diff->sec = end->sec - start->sec;
      diff->usec = end->usec - start->usec;
   }
}

/* Returns time from some unspecified moment in past,
 * strictly increasing, used for time intervals measurement. */
static void osal_getrelativetime(struct timeval *tv)
{
   struct timespec ts;

   /* Use clock_gettime to prevent possible live-lock.
    * Gettimeofday uses CLOCK_REALTIME that can get NTP timeadjust.
    * If this function preempts timeadjust and it uses vpage it live-locks.
    * Also when using XENOMAI, only clock_gettime is RT safe */
   clock_gettime(CLOCK_MONOTONIC, &ts);
   tv->tv_sec = ts.tv_sec;
   tv->tv_usec = ts.tv_nsec / 1000;
}

void osal_timer_start(osal_timert * self, uint32 timeout_usec)
{
   struct timeval start_time;
   struct timeval timeout;
   struct timeval stop_time;

   osal_getrelativetime(&start_time);
   timeout.tv_sec = timeout_usec / USECS_PER_SEC;
   timeout.tv_usec = timeout_usec % USECS_PER_SEC;
   timeradd(&start_time, &timeout, &stop_time);

   self->stop_time.sec = stop_time.tv_sec;
   self->stop_time.usec = stop_time.tv_usec;
}

boolean osal_timer_is_expired (osal_timert * self)
{
   struct timeval current_time;
   struct timeval stop_time;
   int is_not_yet_expired;

   osal_getrelativetime(&current_time);
   stop_time.tv_sec = self->stop_time.sec;
   stop_time.tv_usec = self->stop_time.usec;
   is_not_yet_expired = timercmp(&current_time, &stop_time, <);

   return is_not_yet_expired == FALSE;
}

void *osal_malloc(size_t size)
{
   return malloc(size);
}

void osal_free(void *ptr)
{
   free(ptr);
}

int osal_thread_create(void *thandle, int stacksize, void *func, void *param)
{
   int                  ret;
   pthread_attr_t       attr;
   pthread_t            *threadp;

   threadp = thandle;
   pthread_attr_init(&attr);
   pthread_attr_setstacksize(&attr, stacksize);
   ret = pthread_create(threadp, &attr, func, param);
   if(ret < 0)
   {
      return 0;
   }
   return 1;
}

int osal_thread_create_rt(void *thandle, int stacksize, void *func, void *param)
{
   int                  ret;
   pthread_attr_t       attr;
   struct sched_param   schparam;
   pthread_t            *threadp;

   threadp = thandle;
   pthread_attr_init(&attr);
   pthread_attr_setstacksize(&attr, stacksize);
   ret = pthread_create(threadp, &attr, func, param);
   pthread_attr_destroy(&attr);
   if(ret < 0)
   {
      return 0;
   }
   memset(&schparam, 0, sizeof(schparam));
   schparam.sched_priority = 40;
   ret = pthread_setschedparam(*threadp, SCHED_FIFO, &schparam);
   if(ret < 0)
   {
      return 0;
   }

   return 1;
}

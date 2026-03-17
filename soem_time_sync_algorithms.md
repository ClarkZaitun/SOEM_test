# SOEM 时间同步算法详细说明

本文档详细说明 SOEM (Simple Open EtherCAT Master) 仓库中实现的各种时间同步相关算法。

---

## 目录

1. [ec_sync - PI控制器时间同步](#1-ec_sync---pi控制器时间同步)
2. [add_timespec - 时间累加](#2-add_timespec---时间累加)
3. [ecx_dcsync0 - 从站SYNC0配置](#3-ecx_dcsync0---从站sync0配置)
4. [ecx_dcsync01 - 从站SYNC0和SYNC1配置](#4-ecx_dcsync01---从站sync0和sync1配置)
5. [ecx_configdc - DC配置和传播延迟计算](#5-ecx_configdc---dc配置和传播延迟计算)
6. [osal时间操作函数](#6-osal时间操作函数)

---

## 1. ec_sync - PI控制器时间同步

### 1.1 算法概述

`ec_sync` 函数实现了一个 **PI控制器（比例-积分控制器）**，用于将Linux系统时间与EtherCAT从站的DC（Distributed Clock，分布式时钟）时间进行同步。

这是 EtherCAT 实时通信中的关键技术，通过PI控制器的闭环调节，使主站（Linux系统）能够精确跟踪从站的参考时钟。

### 1.2 函数原型

```c
void ec_sync(int64 reftime, int64 cycletime, int64 *offsettime);
```

### 1.3 输入参数

| 参数 | 类型 | 含义 | 单位 |
|------|------|------|------|
| `reftime` | `int64` | 从站参考时钟时间（DC时间） | 纳秒(ns) |
| `cycletime` | `int64` | 循环周期时间（通信周期） | 纳秒(ns) |

### 1.4 输出参数

| 参数 | 类型 | 含义 | 单位 |
|------|------|------|------|
| `offsettime` | `int64*` | 计算得到的时间偏移量，用于调整帧发送时间 | 纳秒(ns) |

### 1.5 算法详解

```c
// 全局积分变量（需在函数外声明）
static int64 integral = 0;

void ec_sync(int64 reftime, int64 cycletime, int64 *offsettime)
{
    int64 delta;
    
    // 步骤1: 计算相位差
    delta = (reftime - 50000) % cycletime;
    
    // 步骤2: 相位调整到 [-cycletime/2, cycletime/2]
    if(delta > cycletime / 2) { delta = delta - cycletime; }
    
    // 步骤3: 积分累积
    if(delta > 0) { integral++; }
    if(delta < 0) { integral--; }
    
    // 步骤4: PI计算输出
    *offsettime = -(delta / 100) - (integral / 20);
}
```

### 1.6 核心概念

#### 为什么要提前50µs发送？

EtherCAT从站内置精确的时钟（DC），从站在精确的时钟同步下工作。但主站运行在Linux系统上，存在多种延迟：

| 延迟类型 | 原因 |
|---------|------|
| 调度延迟 | 线程切换、CPU竞争 |
| 网卡驱动延迟 | 驱动处理、发送队列 |
| 中断延迟 | 网卡中断处理 |
| 软件延迟 | 应用层到内核层的传递 |

**50µs** 是一个经验值，表示主站需要**比DC同步点提前50µs**发送帧，这样帧经过网络传输延迟后，恰好在DC同步点到达从站。

### 1.7 调节目标

- **最终目标**：帧的发送时间 = DC时间 - 50µs
- **表现形式**：delta 在 0 附近波动（不可能恒等于0，因为DC时间在不断变化）

---

## 2. add_timespec - 时间累加

### 2.1 函数概述

`add_timespec` 函数将纳秒级时间增量添加到 `timespec` 结构体中，用于精确的时间调度。

### 2.2 函数原型

```c
void add_timespec(struct timespec *ts, int64 addtime);
```

### 2.3 参数说明

| 参数 | 类型 | 含义 |
|------|------|------|
| `ts` | `struct timespec*` | 输入/输出，当前时间 |
| `addtime` | `int64` | 要添加的时间增量（纳秒） |

### 2.4 算法实现

```c
void add_timespec(struct timespec *ts, int64 addtime)
{
    int64 sec, nsec;
    
    // 分离秒和纳秒
    nsec = addtime % NSEC_PER_SEC;
    sec = (addtime - nsec) / NSEC_PER_SEC;
    
    // 累加到timespec
    ts->tv_sec += sec;
    ts->tv_nsec += nsec;
    
    // 处理溢出（纳秒 >= 1秒）
    if (ts->tv_nsec >= NSEC_PER_SEC)
    {
        nsec = ts->tv_nsec % NSEC_PER_SEC;
        ts->tv_sec += (ts->tv_nsec - nsec) / NSEC_PER_SEC;
        ts->tv_nsec = nsec;
    }
}
```

### 2.5 理论依据

- **纳秒转秒**：1秒 = 10^9 纳秒
- **溢出处理**：当 `tv_nsec >= 10^9` 时，需要将多余的纳秒转换为秒进位

### 2.6 使用场景

在EtherCAT主循环中，用于计算下一次帧发送的时间点：

```c
struct timespec ts;
// 获取当前时间
clock_gettime(CLOCK_REALTIME, &ts);
// 添加周期时间 + PI调节偏移
add_timespec(&ts, cycletime + toff);
// 休眠到指定时间
pthread_cond_timedwait(&cond, &mutex, &ts);
```

---

## 3. ecx_dcsync0 - 从站SYNC0配置

### 3.1 函数概述

`ecx_dcsync0` 函数配置从站的DC（分布式时钟）同步功能，设置从站以指定周期触发SYNC0信号。

### 3.2 函数原型

```c
void ecx_dcsync0(ecx_contextt *context, uint16 slave, boolean act, 
                 uint32 CyclTime, int32 CyclShift);
```

### 3.3 参数说明

| 参数 | 类型 | 含义 |
|------|------|------|
| `context` | `ecx_contextt*` | EtherCAT主站上下文 |
| `slave` | `uint16` | 从站编号 |
| `act` | `boolean` | TRUE=激活, FALSE=停用 |
| `CyclTime` | `uint32` | 循环周期（纳秒） |
| `CyclShift` | `int32` | 循环偏移（纳秒，可为负） |

### 3.4 算法详解

```c
#define SyncDelay ((int32)100000000)  // 100ms

void ecx_dcsync0(ecx_contextt *context, uint16 slave, boolean act, 
                 uint32 CyclTime, int32 CyclShift)
{
    uint8 h, RA;
    uint16 slaveh;
    int64 t, t1;
    int32 tc;
    
    slaveh = context->slavelist[slave].configadr;
    RA = 0;
    
    // 停止当前循环操作
    (void)ecx_FPWR(context->port, slaveh, ECT_REG_DCSYNCACT, sizeof(RA), &RA, EC_TIMEOUTRET);
    
    if (act)
    {
        RA = 1 + 2;  // 激活sync0
    }
    
    h = 0;
    (void)ecx_FPWR(context->port, slaveh, ECT_REG_DCCUC, sizeof(h), &h, EC_TIMEOUTRET);
    
    // 读取从站本地时间
    (void)ecx_FPRD(context->port, slaveh, ECT_REG_DCSYSTIME, sizeof(t1), &t1, EC_TIMEOUTRET);
    t1 = etohll(t1);
    
    // 计算首次触发时间
    if (CyclTime > 0)
    {
        // 确保触发时间是CyclTime的整数倍（向上舍入）
        t = ((t1 + SyncDelay) / CyclTime) * CyclTime + CyclTime + CyclShift;
    }
    else
    {
        t = t1 + SyncDelay + CyclShift;
    }
    
    // 写入SYNC0启动时间
    t = htoell(t);
    (void)ecx_FPWR(context->port, slaveh, ECT_REG_DCSTART0, sizeof(t), &t, EC_TIMEOUTRET);
    
    // 写入SYNC0循环周期
    tc = htoel(CyclTime);
    (void)ecx_FPWR(context->port, slaveh, ECT_REG_DCCYCLE0, sizeof(tc), &tc, EC_TIMEOUTRET);
    
    // 激活循环操作
    (void)ecx_FPWR(context->port, slaveh, ECT_REG_DCSYNCACT, sizeof(RA), &RA, EC_TIMEOUTRET);
    
    // 更新从站状态
    context->slavelist[slave].DCactive = (uint8)act;
    context->slavelist[slave].DCshift = CyclShift;
    context->slavelist[slave].DCcycle = CyclTime;
}
```

### 3.5 首次触发时间计算

```
t = ((t1 + SyncDelay) / CyclTime) * CyclTime + CyclTime + CyclShift
```

其中：
- `t1`：从站当前本地时间
- `SyncDelay = 100ms`：延迟触发，等待系统稳定
- `CyclTime`：循环周期
- `CyclShift`：用户指定的偏移量

**理论依据**：
1. `t1 + SyncDelay`：延迟100ms后开始同步
2. `((t1 + SyncDelay) / CyclTime) * CyclTime`：向下取整到周期边界
3. `+ CyclTime`：向上舍入到下一个周期
4. `+ CyclShift`：添加用户偏移

这样确保多个从站的SYNC0信号在同一时刻触发（相同CyclTime时）。

---

## 4. ecx_dcsync01 - 从站SYNC0和SYNC1配置

### 4.1 函数概述

`ecx_dcsync01` 函数配置从站同时触发SYNC0和SYNC1两个同步信号，SYNC1可以相对于SYNC0延迟触发。

### 4.2 函数原型

```c
void ecx_dcsync01(ecx_contextt *context, uint16 slave, boolean act,
                  uint32 CyclTime0, uint32 CyclTime1, int32 CyclShift);
```

### 4.3 参数说明

| 参数 | 类型 | 含义 |
|------|------|------|
| `context` | `ecx_contextt*` | EtherCAT主站上下文 |
| `slave` | `uint16` | 从站编号 |
| `act` | `boolean` | TRUE=激活, FALSE=停用 |
| `CyclTime0` | `uint32` | SYNC0循环周期（纳秒） |
| `CyclTime1` | `uint32` | SYNC1相对于SYNC0的延迟（纳秒），0表示同时触发 |
| `CyclShift` | `int32` | 循环偏移（纳秒，可为负） |

### 4.4 算法详解

```c
void ecx_dcsync01(ecx_contextt *context, uint16 slave, boolean act,
                  uint32 CyclTime0, uint32 CyclTime1, int32 CyclShift)
{
    uint8 h, RA;
    uint16 slaveh;
    int64 t, t1;
    int32 tc;
    uint32 TrueCyclTime;
    
    // 计算实际循环周期（SYNC1作为SYNC0的倍数时）
    TrueCyclTime = ((CyclTime1 / CyclTime0) + 1) * CyclTime0;
    
    slaveh = context->slavelist[slave].configadr;
    RA = 0;
    
    // 停止循环操作
    (void)ecx_FPWR(context->port, slaveh, ECT_REG_DCSYNCACT, sizeof(RA), &RA, EC_TIMEOUTRET);
    
    if (act)
    {
        RA = 1 + 2 + 4;  // 激活sync0 + sync1
    }
    
    h = 0;
    (void)ecx_FPWR(context->port, slaveh, ECT_REG_DCCUC, sizeof(h), &h, EC_TIMEOUTRET);
    
    // 读取从站本地时间
    (void)ecx_FPRD(context->port, slaveh, ECT_REG_DCSYSTIME, sizeof(t1), &t1, EC_TIMEOUTRET);
    t1 = etohll(t1);
    
    // 计算首次触发时间
    if (CyclTime0 > 0)
    {
        t = ((t1 + SyncDelay) / TrueCyclTime) * TrueCyclTime + TrueCyclTime + CyclShift;
    }
    else
    {
        t = t1 + SyncDelay + CyclShift;
    }
    
    // ... 写入寄存器 ...
}
```

### 4.5 SYNC0和SYNC1的关系

```
时间轴：
|---------- CyclTime0 ----------|
|<----- CyclTime1 (延迟) ------>|

SYNC0 ──────────────────────────▶
             SYNC0 + CyclTime1 ─▶ SYNC1
```

- SYNC0：主同步信号
- SYNC1：辅助同步信号，可用于需要双触发的应用（如双缓冲）

---

## 5. ecx_configdc - DC配置和传播延迟计算

### 5.1 函数概述

`ecx_configdc` 函数遍历所有从站，检测支持DC功能的从站，并计算它们之间的信号传播延迟。这是实现精确时间同步的关键步骤。

### 5.2 函数原型

```c
boolean ecx_configdc(ecx_contextt *context);
```

### 5.3 返回值

| 返回值 | 含义 |
|--------|------|
| `TRUE` | 找到支持DC功能的从站 |
| `FALSE` | 未找到支持DC功能的从站 |

### 5.4 算法详解

```c
boolean ecx_configdc(ecx_contextt *context)
{
    uint16 i, slaveh, parent, child;
    uint16 prevDCslave = 0;
    int32 ht, dt1, dt2, dt3;
    int64 hrt;
    uint8 entryport;
    int8 nlist, plist[4];
    int32 tlist[4];
    ec_timet mastertime;
    uint64 mastertime64;
    
    // 初始化
    context->slavelist[0].hasdc = FALSE;
    context->grouplist[0].hasdc = FALSE;
    ht = 0;
    
    // 清除DC时间寄存器
    ecx_BWR(context->port, 0, ECT_REG_DCTIME0, sizeof(ht), &ht, EC_TIMEOUTRET);
    
    // 获取主站时间并转换为EtherCAT时间格式
    mastertime = osal_current_time();
    // EtherCAT使用2000年1月1日作为纪元（不是1970年）
    mastertime.sec -= 946684800UL;
    mastertime64 = (((uint64)mastertime.sec * 1000000) + 
                    (uint64)mastertime.usec) * 1000;
    
    // 遍历所有从站
    for (i = 1; i <= *(context->slavecount); i++)
    {
        context->slavelist[i].consumedports = context->slavelist[i].activeports;
        
        if (context->slavelist[i].hasdc)
        {
            // DC从站链表管理
            if (!context->slavelist[0].hasdc)
            {
                context->slavelist[0].hasdc = TRUE;
                context->slavelist[0].DCnext = i;
                context->slavelist[i].DCprevious = 0;
            }
            else
            {
                context->slavelist[prevDCslave].DCnext = i;
                context->slavelist[i].DCprevious = prevDCslave;
            }
            prevDCslave = i;
            
            slaveh = context->slavelist[i].configadr;
            
            // 读取各端口的接收时间戳
            (void)ecx_FPRD(context->port, slaveh, ECT_REG_DCTIME0, sizeof(ht), &ht, EC_TIMEOUTRET);
            context->slavelist[i].DCrtA = etohl(ht);
            // ... DCrtB, DCrtC, DCrtD ...
            
            // 计算并设置系统时间偏移
            (void)ecx_FPRD(context->port, slaveh, ECT_REG_DCSOF, sizeof(hrt), &hrt, EC_TIMEOUTRET);
            hrt = htoell(-etohll(hrt) + mastertime64);
            (void)ecx_FPWR(context->port, slaveh, ECT_REG_DCSYSOFFSET, sizeof(hrt), &hrt, EC_TIMEOUTRET);
            
            // 构建端口时间戳列表
            nlist = 0;
            if (context->slavelist[i].activeports & PORTM0) { ... }
            // ... 其他端口 ...
            
            // 计算传播延迟
            // ...
        }
    }
    
    return context->slavelist[0].hasdc;
}
```

### 5.5 EtherCAT时间格式

```
EtherCAT时间纪元：2000年1月1日 00:00:00 UTC
Unix时间纪元：   1970年1月1日 00:00:00 UTC
差值：          946684800 秒

时间格式转换：
EtherCAT时间(ns) = (Unix时间秒 - 946684800) * 10^6 + 微秒 * 10^3
```

### 5.6 传播延迟计算

EtherCAT网络是线性的，信号依次经过各从站。传播延迟计算：

```
       主站
         |
         |--- 端口0延迟 ---|--- 端口1延迟 ---|
         |                 |                 |
       从站A ──────────> 从站B ──────────> 从站C
         
每个从站记录帧到达各端口的时间戳，通过时间戳差计算延迟。
```

---

## 6. osal时间操作函数

### 6.1 osal_usleep - 微秒级休眠

```c
int osal_usleep(uint32 usec)
{
    struct timespec ts;
    ts.tv_sec = usec / USECS_PER_SEC;
    ts.tv_nsec = (usec % USECS_PER_SEC) * 1000;
    return nanosleep(&ts, NULL);
}
```

### 6.2 osal_current_time - 获取当前时间

```c
ec_timet osal_current_time(void)
{
    struct timespec current_time;
    ec_timet return_value;
    
    // 使用CLOCK_REALTIME获取实时时间
    clock_gettime(CLOCK_REALTIME, &current_time);
    
    return_value.sec = current_time.tv_sec;
    return_value.usec = current_time.tv_nsec / 1000;
    
    return return_value;
}
```

**两种时钟源的区别**：
- `CLOCK_REALTIME`：系统实时时间，可被NTP或管理员修改
- `CLOCK_MONOTONIC`：单调递增时间，不受系统时间调整影响，适合测量时间间隔

### 6.3 osal_time_diff - 计算时间差

```c
void osal_time_diff(ec_timet *start, ec_timet *end, ec_timet *diff)
{
    if (end->usec < start->usec)
    {
        diff->sec = end->sec - start->sec - 1;
        diff->usec = end->usec + 1000000 - start->usec;
    }
    else
    {
        diff->sec = end->sec - start->sec;
        diff->usec = end->usec - start->usec;
    }
}
```

### 6.4 osal_timer_start / osal_timer_is_expired - 定时器

```c
void osal_timer_start(osal_timert *self, uint32 timeout_usec)
{
    struct timeval start_time, timeout, stop_time;
    
    osal_getrelativetime(&start_time);
    timeout.tv_sec = timeout_usec / USECS_PER_SEC;
    timeout.tv_usec = timeout_usec % USECS_PER_SEC;
    timeradd(&start_time, &timeout, &stop_time);
    
    self->stop_time.sec = stop_time.tv_sec;
    self->stop_time.usec = stop_time.tv_usec;
}

boolean osal_timer_is_expired(osal_timert *self)
{
    struct timeval current_time, stop_time;
    
    osal_getrelativetime(&current_time);
    stop_time.sec = self->stop_time.sec;
    stop_time.usec = self->stop_time.usec;
    
    // 返回 current_time >= stop_time
    return timercmp(&current_time, &stop_time, <) == FALSE;
}
```

---

## 算法关系图

```
                    ┌─────────────────────────────────────────┐
                    │           主站应用层                     │
                    │                                         │
                    │  ┌─────────────────────────────────┐   │
                    │  │     主循环                      │   │
                    │  │                                │   │
                    │  │  1. add_timespec() 计算下次    │   │
                    │  │     唤醒时间 (cycletime+toff)   │   │
                    │  │                                │   │
                    │  │  2. pthread_cond_timedwait()   │   │
                    │  │     休眠等待                    │   │
                    │  │                                │   │
                    │  │  3. ec_send_processdata()      │   │
                    │  │     发送帧                      │   │
                    │  │                                │   │
                    │  │  4. ec_sync() ←───────────┐    │   │
                    │  │     PI控制器计算toff  │    │   │
                    │  └─────────────────────────────────┘   │
                    └────────────────┬──────────────────────┘
                                     │
                    ┌────────────────▼──────────────────────┐
                    │         ecx_configdc()                 │
                    │   配置DC+计算传播延迟                    │
                    └────────────────┬──────────────────────┘
                                     │
                    ┌────────────────▼──────────────────────┐
                    │       ecx_dcsync0() / dcsync01()       │
                    │    配置从站SYNC0/SYNC1触发              │
                    └───────────────────────────────────────┘
                                     │
                    ┌────────────────▼──────────────────────┐
                    │           EtherCAT从站                  │
                    │    DC分布式时钟同步                     │
                    └───────────────────────────────────────┘
```

---

## 参考资料

- EtherCAT Technology Group: EtherCAT Slave Information
- IEC 61158-12: Industrial communications networks - Fieldbus specifications
- IEEE 1588: Precision Clock Synchronization Protocol
- Linux man pages: clock_gettime, nanosleep, pthread_cond_timedwait

#include "types.h"
#include "param.h"
#include "defs.h"
#include "sysinfo.h"
#include "proc.h"
#include "defs.h"

#include "types.h"
#include "riscv.h"
#include "defs.h"
#include "param.h"
#include "stat.h"
#include "spinlock.h"
#include "proc.h"
#include "fs.h"
#include "sleeplock.h"
#include "file.h"
#include "fcntl.h"


struct sysinfo sysinfo;


uint64
sys_sysinfo(void)
{
    //计算活跃线程和空闲内存
    int proCount;
    int memCount;
    struct proc *p = myproc();
    uint64 info_addr;
    sysinfo.nproc = countActiceProcess();
    sysinfo.freemem = countFreeMem();

    //从a1寄存器中获取sys_info结构体地址
    if(argaddr(0,&info_addr)<0)
        return -1;
    //将sysinfo信息返回指定地址的用户态
    if(copyout(p->pagetable,info_addr,(char *)&info_addr,sizeof(sysinfo))<0)
        return -1;
    return 0;
}
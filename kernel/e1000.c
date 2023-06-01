#include "types.h"
#include "param.h"
#include "memlayout.h"
#include "riscv.h"
#include "spinlock.h"
#include "proc.h"
#include "defs.h"
#include "e1000_dev.h"
#include "net.h"

// 传输环形队列定义
// sz
#define TX_RING_SIZE 16
// 传输环形队列结构体，每个元素都描述了对应的数据包信息（以及指向它们的指针）
static struct tx_desc tx_ring[TX_RING_SIZE] __attribute__((aligned(16)));
// 临时数据包数组，对应下标存放对应队列桶中要传输的数据包
static struct mbuf *tx_mbufs[TX_RING_SIZE];

#define RX_RING_SIZE 16
static struct rx_desc rx_ring[RX_RING_SIZE] __attribute__((aligned(16)));
static struct mbuf *rx_mbufs[RX_RING_SIZE];

// remember where the e1000's registers live.
static volatile uint32 *regs;

struct spinlock e1000_lock;

// called by pci_init().
// xregs is the memory address at which the
// e1000's registers are mapped. e1000的寄存器映射的内存地址。
void
e1000_init(uint32 *xregs)
{
  int i;

  initlock(&e1000_lock, "e1000");

  regs = xregs;

  // Reset the device
  regs[E1000_IMS] = 0; // disable interrupts
  regs[E1000_CTL] |= E1000_CTL_RST;
  regs[E1000_IMS] = 0; // redisable interrupts
  __sync_synchronize();

  // [E1000 14.5] Transmit initialization
  memset(tx_ring, 0, sizeof(tx_ring));
  for (i = 0; i < TX_RING_SIZE; i++) {
    tx_ring[i].status = E1000_TXD_STAT_DD;
    tx_mbufs[i] = 0;
  }
  regs[E1000_TDBAL] = (uint64) tx_ring;
  if(sizeof(tx_ring) % 128 != 0)
    panic("e1000");
  regs[E1000_TDLEN] = sizeof(tx_ring);
  regs[E1000_TDH] = regs[E1000_TDT] = 0;
  
  // [E1000 14.4] Receive initialization
  memset(rx_ring, 0, sizeof(rx_ring));
  for (i = 0; i < RX_RING_SIZE; i++) {
    rx_mbufs[i] = mbufalloc(0);
    if (!rx_mbufs[i])
      panic("e1000");
    rx_ring[i].addr = (uint64) rx_mbufs[i]->head;
  }
  regs[E1000_RDBAL] = (uint64) rx_ring;
  if(sizeof(rx_ring) % 128 != 0)
    panic("e1000");
  regs[E1000_RDH] = 0;
  regs[E1000_RDT] = RX_RING_SIZE - 1;
  regs[E1000_RDLEN] = sizeof(rx_ring);

  // filter by qemu's MAC address, 52:54:00:12:34:56
  regs[E1000_RA] = 0x12005452;
  regs[E1000_RA+1] = 0x5634 | (1<<31);
  // multicast table
  for (int i = 0; i < 4096/32; i++)
    regs[E1000_MTA + i] = 0;

  // transmitter control bits.
  regs[E1000_TCTL] = E1000_TCTL_EN |  // enable
    E1000_TCTL_PSP |                  // pad short packets
    (0x10 << E1000_TCTL_CT_SHIFT) |   // collision stuff
    (0x40 << E1000_TCTL_COLD_SHIFT);
  regs[E1000_TIPG] = 10 | (8<<10) | (6<<20); // inter-pkt gap

  // receiver control bits.
  regs[E1000_RCTL] = E1000_RCTL_EN | // enable receiver
    E1000_RCTL_BAM |                 // enable broadcast
    E1000_RCTL_SZ_2048 |             // 2048-byte rx buffers
    E1000_RCTL_SECRC;                // strip CRC
  
  // ask e1000 for receive interrupts.
  regs[E1000_RDTR] = 0; // interrupt after every received packet (no timer)
  regs[E1000_RADV] = 0; // interrupt after every packet (no timer)
  regs[E1000_IMS] = (1 << 7); // RXDW -- Receiver Descriptor Write Back
}

int
e1000_transmit(struct mbuf *m)
{
  //
  // Your code here.
  //
  // the mbuf contains an ethernet frame; program it into
  // the TX descriptor ring so that the e1000 sends it. Stash
  // a pointer so that it can be freed after sending.
  //
  acquire(&e1000_lock);

  uint32 npIdx = regs[E1000_TDT];
  //如果使用了已经发送完毕的数据包缓冲桶就退出
  if(!(tx_ring[npIdx].status & E1000_TXD_STAT_DD)){
    release(&e1000_lock);
    return -1;
  }
  // 查看桶中是否存有上一轮发送的数据包缓冲,如果有则清空缓冲桶
  if(tx_mbufs[npIdx]){
    mbuffree(tx_mbufs[npIdx]);
    tx_mbufs[npIdx] = 0;
  }
  tx_ring[npIdx].addr = (uint64)m->head;
  tx_ring[npIdx].length = m->len;
  tx_ring[npIdx].cmd = E1000_TXD_CMD_RS | E1000_TXD_CMD_EOP;
  tx_ring[npIdx].special = 0x0b;

  // 将指针记录，并在下轮传输中进行清除
  tx_mbufs[npIdx] = m;

  regs[E1000_TDT] = (npIdx+1) % TX_RING_SIZE;

  release(&e1000_lock);

  return 0;
}

static void
e1000_recv(void)
{
  //
  // Your code here.
  //
  // Check for packets that have arrived from the e1000
  // Create and deliver an mbuf for each packet (using net_rx()).
  //
  // acquire(&e1000_lock);
  // 如果没有空闲的桶可以接受缓存
  while(1){

    uint32 npIdx = (regs[E1000_RDT] + 1)% RX_RING_SIZE;

    if(!(rx_ring[npIdx].status & E1000_RXD_STAT_DD)){
      return;
    }
    rx_mbufs[npIdx]->len = rx_ring[npIdx].length;
    // 发起中断让e1000去将数据报发送到网络协议堆栈中，
    // 识别各种协议的数据包，并发送到相应的网络堆栈中
    net_rx(rx_mbufs[npIdx]);
    // 指定新的缓冲区来接受下一轮的数据包
    struct mbuf *n_mbuf = mbufalloc(0);
    if(n_mbuf==0){
      // release(&e1000_lock);
      printf("e1000_recv: mbufalloc fail");
      return;
    }
    rx_mbufs[npIdx] = n_mbuf;
    rx_ring[npIdx].addr = (uint64)n_mbuf->head;
    rx_ring[npIdx].status = 0;

    regs[E1000_RDT] = npIdx;
  }
}


// static void
// e1000_recv(void)
// {
//   while(1) { // 每次 recv 可能接收多个包

//     uint32 ind = (regs[E1000_RDT] + 1) % RX_RING_SIZE;
    
//     struct rx_desc *desc = &rx_ring[ind];
//     // 如果需要接收的包都已经接收完毕，则退出
//     if(!(desc->status & E1000_RXD_STAT_DD)) {
//       return;
//     }

//     rx_mbufs[ind]->len = desc->length;
    
//     net_rx(rx_mbufs[ind]); // 传递给上层网络栈。上层负责释放 mbuf

//     // 分配并设置新的 mbuf，供给下一次轮到该下标时使用
//     rx_mbufs[ind] = mbufalloc(0); 
//     desc->addr = (uint64)rx_mbufs[ind]->head;
//     desc->status = 0;

//     regs[E1000_RDT] = ind;
//   }

// }

void
e1000_intr(void)
{
  // tell the e1000 we've seen this interrupt;
  // without this the e1000 won't raise any
  // further interrupts.
  regs[E1000_ICR] = 0xffffffff;

  e1000_recv();
}

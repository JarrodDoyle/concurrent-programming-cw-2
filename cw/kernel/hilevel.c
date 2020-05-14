/* Copyright (C) 2017 Daniel Page <csdsp@bristol.ac.uk>
 *
 * Use of this source code is restricted per the CC BY-NC-ND license, a copy of 
 * which can be found via http://creativecommons.org (and should be included as 
 * LICENSE.txt within the associated archive or repository).
 */

#include "hilevel.h"

pcb_t procTab[MAX_PROCS];
pcb_t *executing = NULL;

int prioTab[MAX_PROCS];
int agesTab[MAX_PROCS];

// Changes currently executing program
void dispatch(ctx_t *ctx, pcb_t *prev, pcb_t *next)
{
  char prev_pid = '?', next_pid = '?';

  if (NULL != prev)
  {
    memcpy(&prev->ctx, ctx, sizeof(ctx_t)); // preserve execution context of P_{prev}
    prev_pid = '0' + prev->pid;
  }
  if (NULL != next)
  {
    memcpy(ctx, &next->ctx, sizeof(ctx_t)); // restore  execution context of P_{next}
    next_pid = '0' + next->pid;
  }

  PL011_putc(UART0, '[', true);
  PL011_putc(UART0, prev_pid, true);
  PL011_putc(UART0, '-', true);
  PL011_putc(UART0, '>', true);
  PL011_putc(UART0, next_pid, true);
  PL011_putc(UART0, ']', true);

  executing = next; // update   executing process to P_{next}

  return;
}

// Priority+Age, 1 <= n <= N , scheduler
void schedule(ctx_t *ctx)
{
  int current = NULL;
  int next = NULL;
  uint32_t highestPrio = 0;
  for (int i = 0; i < MAX_PROCS; i++)
  {
    if (procTab[i].status != STATUS_INVALID)
    {
      if (procTab[i].pid == executing->pid)
      {
        current = i;
      }
      else
      {
        agesTab[i] += 1;
      }

      if (prioTab[i] + agesTab[i] > highestPrio)
      {
        highestPrio = prioTab[i] + agesTab[i];
        next = i;
      }
    }
  }

  // Dispatch only if executing process has changed
  if (current != next)
  {
    dispatch(ctx, &procTab[current], &procTab[next]);
    procTab[current].status = STATUS_READY;
    procTab[next].status = STATUS_EXECUTING;
    agesTab[next] = 0;
  }

  return;
}

extern void main_console();

void hilevel_handler_rst(ctx_t *ctx)
{
  /* Invalidate all entries in the process table, so it's clear they are not
   * representing valid (i.e., active) processes.
   */

  for (int i = 0; i < MAX_PROCS; i++)
  {
    procTab[i].status = STATUS_INVALID;
  }

  /* Configure the mechanism for interrupt handling by
   *
   * - configuring timer st. it raises a (periodic) interrupt for each 
   *   timer tick,
   * - configuring GIC st. the selected interrupts are forwarded to the 
   *   processor via the IRQ interrupt signal, then
   * - enabling IRQ interrupts.
   */

  TIMER0->Timer1Load = 0x00100000;  // select period = 2^20 ticks ~= 1 sec
  TIMER0->Timer1Ctrl = 0x00000002;  // select 32-bit   timer
  TIMER0->Timer1Ctrl |= 0x00000040; // select periodic timer
  TIMER0->Timer1Ctrl |= 0x00000020; // enable          timer interrupt
  TIMER0->Timer1Ctrl |= 0x00000080; // enable          timer

  GICC0->PMR = 0x000000F0;         // unmask all            interrupts
  GICD0->ISENABLER1 |= 0x00000010; // enable timer          interrupt
  GICC0->CTLR = 0x00000001;        // enable GIC interface
  GICD0->CTLR = 0x00000001;        // enable GIC distributor

  int_enable_irq();

  /* Automatically execute the user program console by setting the fields
   * in the associated PCB.  Note in each case that
   *
   * - the CPSR value of 0x50 means the processor is switched into USR mode,
   *   with IRQ interrupts enabled, and
   * - the PC and SP values match the entry point and top of stack.
   */

  int *tos = malloc(1000);
  memset(&procTab[0], 0, sizeof(pcb_t));
  procTab[0].pid = 0;
  procTab[0].status = STATUS_READY;
  procTab[0].tos = (uint32_t)(tos);
  procTab[0].ctx.cpsr = 0x50;
  procTab[0].ctx.pc = (uint32_t)(&main_console);
  procTab[0].ctx.sp = procTab[0].tos;
  prioTab[0] = 0;
  agesTab[0] = 0;

  /* Once the PCBs are initialised, we arbitrarily select the 0-th PCB to be 
   * executed: there is no need to preserve the execution context, since it 
   * is invalid on reset (i.e., no process was previously executing).
   */

  dispatch(ctx, NULL, &procTab[0]);

  return;
}

void hilevel_handler_irq(ctx_t *ctx)
{
  // Step 2: read  the interrupt identifier so we know the source.

  uint32_t id = GICC0->IAR;

  // Step 4: handle the interrupt, then clear (or reset) the source.

  if (id == GIC_SOURCE_TIMER0)
  {
    // PL011_putc(UART0, 'T', true);
    schedule(ctx);
    TIMER0->Timer1IntClr = 0x01;
  }

  // Step 5: write the interrupt identifier to signal we're done.

  GICC0->EOIR = id;

  return;
}

void hilevel_handler_svc(ctx_t *ctx, uint32_t id)
{
  /* Based on the identifier (i.e., the immediate operand) extracted from the
   * svc instruction, 
   *
   * - read  the arguments from preserved usr mode registers,
   * - perform whatever is appropriate for this system call, then
   * - write any return value back to preserved usr mode registers.
   */

  switch (id)
  {
  case 0x00:
  { // 0x00 => yield()
    schedule(ctx);

    break;
  }

  case 0x01:
  { // 0x01 => write(fd, x, n)
    int fd = (int)(ctx->gpr[0]);
    char *x = (char *)(ctx->gpr[1]);
    int n = (int)(ctx->gpr[2]);

    for (int i = 0; i < n; i++)
    {
      PL011_putc(UART0, *x++, true);
    }

    ctx->gpr[0] = n;

    break;
  }

  case 0x02:
  { // 0x02 => read(fx, x, n)
    int fd = (int)(ctx->gpr[0]);
    char *x = (char *)(ctx->gpr[1]);
    int n = (int)(ctx->gpr[2]);

    for (int i = 0; i < n; i++)
    {
      *x++ = PL011_getc(UART0, true);
    }

    ctx->gpr[0] = n;

    break;
  }

  case 0x03:
  { // 0x03 => fork()
    // Create child pcb_t with unique pid_t
    int child_pid = -1;
    for (int i = 0; i < MAX_PROCS; i++)
    {
      if (procTab[i].status == STATUS_INVALID)
      {
        // Replicate parent state in child
        child_pid = i;
        int *tos_child = malloc(1000) + 1000;

        // Replicate parent process
        memset(&procTab[i], 0, sizeof(pcb_t));
        procTab[i].pid = i;
        procTab[i].status = STATUS_READY;
        procTab[i].tos = (uint32_t)(tos_child);
        procTab[i].ctx.cpsr = ctx->cpsr;
        procTab[i].ctx.pc = ctx->pc;
        procTab[i].ctx.sp = procTab[i].tos - (executing->tos - ctx->sp);
        procTab[i].ctx.lr = ctx->lr;
        for (int j = 0; j < 13; j++)
        {
          procTab[i].ctx.gpr[j] = ctx->gpr[j];
        }

        // Copy stack
        memcpy(tos_child - 1000, (int *)(executing->tos) - 1000, 1000);
        prioTab[i] = prioTab[executing->pid];
        agesTab[i] = agesTab[executing->pid];

        break;
      }
    }

    PL011_putc(UART0, '[', true);
    PL011_putc(UART0, 'F', true);
    PL011_putc(UART0, 'O', true);
    PL011_putc(UART0, 'R', true);
    PL011_putc(UART0, 'K', true);
    PL011_putc(UART0, ']', true);

    // Return child pid to parent and 0 to child
    ctx->gpr[0] = child_pid;
    if (child_pid != -1)
    {
      procTab[child_pid].ctx.gpr[0] = 0;
    }

    break;
  }

  case 0x04:
  { // 0x04 => exit(x)
  }

  case 0x05:
  { // 0x05 => exec(x)
    PL011_putc(UART0, '[', true);
    PL011_putc(UART0, 'E', true);
    PL011_putc(UART0, 'X', true);
    PL011_putc(UART0, 'E', true);
    PL011_putc(UART0, 'C', true);
    PL011_putc(UART0, ']', true);

    int *x = (int *)(ctx->gpr[0]);

    // int pid = executing->pid;
    // procTab[pid].status = STATUS_READY;
    // procTab[pid].ctx.cpsr = 0x50;
    // procTab[pid].ctx.pc = (uint32_t)(x);
    // prioTab[pid] = 10;
    // agesTab[pid] = 0;
    // memset((int *)(procTab[pid].ctx.sp), 0, 1000);

    // Update pcb and ctx
    int pid = executing->pid;
    procTab[pid].status = STATUS_READY;
    procTab[pid].ctx.cpsr = 0x50;
    procTab[pid].ctx.pc = (uint32_t)(x);
    procTab[pid].ctx.sp = procTab[pid].tos;
    procTab[pid].ctx.lr = 0;
    for (int i = 0; i < 13; i++)
    {
      procTab[pid].ctx.gpr[i] = 0;
    }

    // Clear stack
    memset((int *)(procTab[pid].tos) - 1000, 0, 1000);

    break;
  }

  default:
  { // 0x?? => unknown/unsupported
    break;
  }
  }

  return;
}

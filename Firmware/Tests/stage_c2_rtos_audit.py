#!/usr/bin/env python3
"""Stage C2: static STM32H745 dual-FreeRTOS task and IRQ contracts.
Not physical WCET, ISR jitter, runtime stack watermark or worst-case heap.
"""
import argparse
import json
from pathlib import Path
import re
import sys

ROOT=Path(__file__).resolve().parents[2]
FW=ROOT/"Firmware/STM32H745"

def pick(pattern,text,label):
    matches=re.findall(pattern,text,re.M)
    if len(matches)!=1:
        raise ValueError("unexpected source shape: "+label)
    return int(matches[0])

def config(core):
    text=(FW/core/"Core/Inc/FreeRTOSConfig.h").read_text()
    return {
        "tick_hz":pick(r"^\s*#define\s+configTICK_RATE_HZ\s+\(\(TickType_t\)(\d+)\)",text,"tick"),
        "max_priorities":pick(r"^\s*#define\s+configMAX_PRIORITIES\s+\(\s*(\d+)\s*\)",text,"max priority"),
        "heap_bytes_configured":pick(r"^\s*#define\s+configTOTAL_HEAP_SIZE\s+\(\(size_t\)(\d+)\)",text,"heap"),
        "max_syscall_irq_priority":pick(r"^\s*#define\s+configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY\s+(\d+)",text,"irq ceiling"),
    }

def audit():
    cfg={name:config(name) for name in ("CM7","CM4")}
    if any(c["tick_hz"]!=1000 or c["max_syscall_irq_priority"]!=5 for c in cfg.values()):
        raise ValueError("CM7/CM4 RTOS clock or ISR API ceiling changed")
    sup=(ROOT/"StateMachine/supervisor_task.c").read_text()
    m4=(FW/"CM4/Core/Src/application_validation_service.c").read_text()
    m7_main=(FW/"CM7/Core/Src/main.c").read_text()
    m4_main=(FW/"CM4/Core/Src/main.c").read_text()
    msp=(FW/"CM7/Core/Src/stm32h7xx_hal_msp.c").read_text()
    notify=(FW/"Common/Src/intercore_notifications.c").read_text()
    app=(FW/"CM7/Core/Src/application_supervisor.c").read_text()
    stacks={
        "M7 Supervisor":4*pick(r"^\s*#define SUPERVISOR_TASK_STACK_WORDS\s+(\d+)U",sup,"supervisor stack"),
        "M7 validation worker":4*pick(r"^\s*#define VALIDATION_TASK_STACK_WORDS\s+(\d+)U",sup,"M7 validation stack"),
        "M4 NOR worker (only with physical provider)":4*pick(r"^\s*#define GP_M4_VALIDATION_STACK_WORDS\s+(\d+)U",m4,"M4 flash stack"),
        "M7 CubeMX defaultTask":pick(r"\.stack_size\s*=\s*(\d+)\s*\*\s*(\d+)",m7_main,"M7 default") * 4,
        "M4 CubeMX defaultTask":pick(r"\.stack_size\s*=\s*(\d+)\s*\*\s*(\d+)",m4_main,"M4 default") * 4,
    }
    irq={}
    for name in ("TIM2","TIM3"):
        irq[name]=pick(r"HAL_NVIC_SetPriority\("+name+r"_IRQn,\s*(\d+)\s*,\s*0\s*\)",msp,name+" IRQ")
    irq["EXTI M7/M4"]=pick(r"HAL_NVIC_SetPriority\(IPC_RX_IRQn,\s*(\d+)U,\s*0U\)",notify,"EXTI")
    if irq["TIM3"]<cfg["CM7"]["max_syscall_irq_priority"] or irq["EXTI M7/M4"]<5:
        raise ValueError("ISR that uses FromISR API exceeds FreeRTOS priority ceiling")
    if not re.search(r"supervisor_task_start\(4U\)",app):
        raise ValueError("M7 Supervisor priority changed")
    if not re.search(r"GP_M4_VALIDATION_PRIORITY\s+\(tskIDLE_PRIORITY\s*\+\s*1U\)",m4):
        raise ValueError("M4 worker priority changed")
    if "xTaskCreateStatic(" not in sup or "xTaskCreateStatic(" not in m4:
        raise ValueError("expected static worker allocation absent")
    if not re.search(r"osPriorityNormal",m7_main) or not re.search(r"osPriorityNormal",m4_main):
        raise ValueError("generated defaultTask priorities changed")
    if not re.search(r"htim3.Init.Period\s*!=\s*1999U",app):
        raise ValueError("2ms timer guard changed")
    return {
        "rtos_config":cfg,"known_static_stack_bytes":stacks,
        "known_static_stack_total_bytes":{
           "CM7":stacks["M7 Supervisor"]+stacks["M7 validation worker"],
           "CM4":stacks["M4 NOR worker (only with physical provider)"]
        },
        "interrupt_preemption_priorities":irq,
        "M7_supervisor_task_priority":4,
        "M4_NOR_worker_priority":1,
        "current_fault_gates":["M7 TIM3 missed-period and overrun host extraction",
                              "HAL1 host bus-off and FIFO fault mock",
                              "M4 status freshness and fault policy",
                              "M4 storage CRC/abort/readback host test"],
        "warnings":[
          "M7/M4 CubeMX defaultTask stack is 512 bytes each; must measure during board bring-up",
          "CMSIS-RTOS2 osPriorityNormal may preempt FreeRTOS priority-4 Supervisor; inspect latency on hardware",
          "configTOTAL_HEAP_SIZE is provisioned heap, not observed heap use",
          "Stack arrays in BSS are not peak stack or dynamic task context",
          "Task WCET and noncacheable SRAM4 sharing remain unmeasured on physical hardware",
          "M4 worker 768-word stack is unproven and absent when NOR provider is not commissioned"
        ]
    }

def selftest():
    a=audit()
    assert a["known_static_stack_bytes"]["M7 Supervisor"]==16384
    assert a["known_static_stack_bytes"]["M4 NOR worker (only with physical provider)"]==3072
    assert a["interrupt_preemption_priorities"]["TIM3"]==5
    assert a["interrupt_preemption_priorities"]["EXTI M7/M4"]==6
    print("PASS: Stage C2 FreeRTOS priorities/ISR/stack source audit selftest")

def main():
    p=argparse.ArgumentParser()
    p.add_argument("--self-test",action="store_true")
    p.add_argument("--out",type=Path)
    args=p.parse_args()
    if args.self_test:
        selftest()
        return
    if args.out is None: p.error("--out required")
    result=audit()
    args.out.mkdir(parents=True,exist_ok=True)
    (args.out/"rtos_static_audit.json").write_text(json.dumps(result,indent=2)+"\n")
    lines=["# Stage C2 static FreeRTOS / IRQ review","",
           "**Source checks, NOT measured 500Hz CPU WCET, scheduling jitter or stack usage.**","",
           "| Task | Statical allocated stack (bytes, provisioned) |",
           "|---|---:|"]
    for k,v in result["known_static_stack_bytes"].items():
        lines.append("| "+k+" | "+str(v)+" |")
    lines+=["","| Interrupt | HAL NVIC preemption priority |","|---|---:|"]
    for k,v in result["interrupt_preemption_priorities"].items():
        lines.append("| "+k+" | "+str(v)+" |")
    lines+=["","All FreeRTOS FromISR IRQs checked against syscall ceiling 5.","",
            "## Configured heap vs used heap",""]
    for k,v in result["rtos_config"].items():
        lines.append("- "+k+": configured heap "+str(v["heap_bytes_configured"])+" bytes; not observed allocation.")
    lines+=["","## Unresolved hardware / timing limitations",""]
    for item in result["warnings"]:
        lines.append("- "+item)
    (args.out/"rtos_static_audit.md").write_text("\n".join(lines)+"\n")
    print("PASS: Stage C2 source-based ISR/task/static-stack configuration audit")
    print("WARNING: 512-byte CubeMX defaultTask stacks and M7 defaultTask priority remain hardware-WCET blockers")

if __name__=="__main__":
    try: main()
    except (ValueError,AssertionError,OSError) as e:
        print("FAIL: C2 FreeRTOS contract:",e,file=sys.stderr)
        sys.exit(1)


;***************************************************************************************
; Amount of memory (in bytes) allocated for Stack and Heap
; Tailor those values to your application needs          
;***************************************************************************************
Stack_Size   EQU     0x200
Heap_Size    EQU     0x200

;*******************************************************************************
; Allocate space for the Stack
;*******************************************************************************
  AREA    STACK, NOINIT, READWRITE, ALIGN=3

Stack
    SPACE   Stack_Size

;*******************************************************************************
; Allocate space for the Heap
;*******************************************************************************
  AREA    HEAP, NOINIT, READWRITE, ALIGN=3

Heap
    SPACE   Heap_Size

;********************************************************************************
;*  Declarations for the interrupt handlers that are used by the application.                                                                     
;********************************************************************************
  		IMPORT  __main

  		IMPORT  OSPendSV
  		IMPORT  SysTickHandler

        IMPORT     WWDG_IRQHandler           ; Window Watchdog
        IMPORT     PVD_IRQHandler            ; PVD through EXTI Line detect
        IMPORT     TAMPER_IRQHandler         ; Tamper
        IMPORT     RTC_IRQHandler            ; RTC
        IMPORT     FLASH_IRQHandler          ; Flash
        IMPORT     RCC_IRQHandler            ; RCC
        IMPORT     EXTI0_IRQHandler          ; EXTI Line 0
        IMPORT     EXTI1_IRQHandler          ; EXTI Line 1
        IMPORT     EXTI2_IRQHandler          ; EXTI Line 2
        IMPORT     EXTI3_IRQHandler          ; EXTI Line 3
        IMPORT     EXTI4_IRQHandler          ; EXTI Line 4
        IMPORT     DMA1_Channel1_IRQHandler  ; DMA1 Channel 1
        IMPORT     DMA1_Channel2_IRQHandler  ; DMA1 Channel 2
        IMPORT     DMA1_Channel3_IRQHandler  ; DMA1 Channel 3
        IMPORT     DMA1_Channel4_IRQHandler  ; DMA1 Channel 4
        IMPORT     DMA1_Channel5_IRQHandler  ; DMA1 Channel 5
        IMPORT     DMA1_Channel6_IRQHandler  ; DMA1 Channel 6
        IMPORT     DMA1_Channel7_IRQHandler  ; DMA1 Channel 7
        IMPORT     ADC1_2_IRQHandler         ; ADC1 & ADC2
        IMPORT     USB_HP_CAN1_TX_IRQHandler  ; USB High Priority or CAN1 TX
        IMPORT     USB_LP_CAN1_RX0_IRQHandler ; USB Low  Priority or CAN1 RX0
        IMPORT     CAN1_RX1_IRQHandler       ; CAN1 RX1
        IMPORT     CAN1_SCE_IRQHandler       ; CAN1 SCE
        IMPORT     EXTI9_5_IRQHandler        ; EXTI Line 9..5
        IMPORT     TIM1_BRK_IRQHandler       ; TIM1 Break
        IMPORT     TIM1_UP_IRQHandler        ; TIM1 Update
        IMPORT     TIM1_TRG_COM_IRQHandler   ; TIM1 Trigger and Commutation
        IMPORT     TIM1_CC_IRQHandler        ; TIM1 Capture Compare
        IMPORT     TIM2_IRQHandler           ; TIM2
        IMPORT     TIM3_IRQHandler           ; TIM3
        IMPORT     TIM4_IRQHandler           ; TIM4
        IMPORT     I2C1_EV_IRQHandler        ; I2C1 Event
        IMPORT     I2C1_ER_IRQHandler        ; I2C1 Error
        IMPORT     I2C2_EV_IRQHandler        ; I2C2 Event
        IMPORT     I2C2_ER_IRQHandler        ; I2C2 Error
        IMPORT     SPI1_IRQHandler           ; SPI1
        IMPORT     SPI2_IRQHandler           ; SPI2
        IMPORT     USART1_IRQHandler         ; USART1
        IMPORT     USART2_IRQHandler         ; USART2
        IMPORT     USART3_IRQHandler         ; USART3
        IMPORT     EXTI15_10_IRQHandler      ; EXTI Line 15..10
        IMPORT     RTCAlarm_IRQHandler       ; RTC Alarm through EXTI Line
        IMPORT     USBWakeUp_IRQHandler      ; USB Wakeup from suspend
        IMPORT     TIM8_BRK_IRQHandler       ; TIM8 Break
        IMPORT     TIM8_UP_IRQHandler        ; TIM8 Update
        IMPORT     TIM8_TRG_COM_IRQHandler   ; TIM8 Trigger and Commutation
        IMPORT     TIM8_CC_IRQHandler        ; TIM8 Capture Compare
        IMPORT     ADC3_IRQHandler           ; ADC3
        IMPORT     FSMC_IRQHandler           ; FSMC
        IMPORT     SDIO_IRQHandler           ; SDIO
        IMPORT     TIM5_IRQHandler           ; TIM5
        IMPORT     SPI3_IRQHandler           ; SPI3
        IMPORT     UART4_IRQHandler          ; UART4
        IMPORT     UART5_IRQHandler          ; UART5
        IMPORT     TIM6_IRQHandler           ; TIM6
        IMPORT     TIM7_IRQHandler           ; TIM7
        IMPORT     DMA2_Channel1_IRQHandler  ; DMA2 Channel1
        IMPORT     DMA2_Channel2_IRQHandler  ; DMA2 Channel2
        IMPORT     DMA2_Channel3_IRQHandler  ; DMA2 Channel3
        IMPORT     DMA2_Channel4_5_IRQHandler ; DMA2 Channel4 & Channel5

  
          
		PRESERVE8

;**********************************************************************************
;*  Reset code section.                                                                                                           
;**********************************************************************************
        AREA    RESET, CODE, READONLY
        THUMB

;*******************************************************************************
; Fill-up the Vector Table entries with the exceptions ISR address
;*******************************************************************************
    	EXPORT  __Vectors
__Vectors                      
    	DCD  Stack + Stack_Size            ; Top of Stack
    	DCD  Reset_Handler
    	DCD  NMIException
    	DCD  HardFaultException
    	DCD  MemManageException
    	DCD  BusFaultException
    	DCD  UsageFaultException
    	DCD  0                 ; Reserved
    	DCD  0                 ; Reserved
    	DCD  0                 ; Reserved
    	DCD  0                 ; Reserved
    	DCD  SVCHandler
    	DCD  DebugMonitor
    	DCD  0                 ; Reserved
    	DCD  OSPendSV
    	DCD  SysTickHandler

        ; External Interrupts
        DCD     WWDG_IRQHandler           ; Window Watchdog
        DCD     PVD_IRQHandler            ; PVD through EXTI Line detect
        DCD     TAMPER_IRQHandler         ; Tamper
        DCD     RTC_IRQHandler            ; RTC
        DCD     FLASH_IRQHandler          ; Flash
        DCD     RCC_IRQHandler            ; RCC
        DCD     EXTI0_IRQHandler          ; EXTI Line 0
        DCD     EXTI1_IRQHandler          ; EXTI Line 1
        DCD     EXTI2_IRQHandler          ; EXTI Line 2
        DCD     EXTI3_IRQHandler          ; EXTI Line 3
        DCD     EXTI4_IRQHandler          ; EXTI Line 4
        DCD     DMA1_Channel1_IRQHandler  ; DMA1 Channel 1
        DCD     DMA1_Channel2_IRQHandler  ; DMA1 Channel 2
        DCD     DMA1_Channel3_IRQHandler  ; DMA1 Channel 3
        DCD     DMA1_Channel4_IRQHandler  ; DMA1 Channel 4
        DCD     DMA1_Channel5_IRQHandler  ; DMA1 Channel 5
        DCD     DMA1_Channel6_IRQHandler  ; DMA1 Channel 6
        DCD     DMA1_Channel7_IRQHandler  ; DMA1 Channel 7
        DCD     ADC1_2_IRQHandler         ; ADC1 & ADC2
        DCD     USB_HP_CAN1_TX_IRQHandler  ; USB High Priority or CAN1 TX
        DCD     USB_LP_CAN1_RX0_IRQHandler ; USB Low  Priority or CAN1 RX0
        DCD     CAN1_RX1_IRQHandler       ; CAN1 RX1
        DCD     CAN1_SCE_IRQHandler       ; CAN1 SCE
        DCD     EXTI9_5_IRQHandler        ; EXTI Line 9..5
        DCD     TIM1_BRK_IRQHandler       ; TIM1 Break
        DCD     TIM1_UP_IRQHandler        ; TIM1 Update
        DCD     TIM1_TRG_COM_IRQHandler   ; TIM1 Trigger and Commutation
        DCD     TIM1_CC_IRQHandler        ; TIM1 Capture Compare
        DCD     TIM2_IRQHandler           ; TIM2
        DCD     TIM3_IRQHandler           ; TIM3
        DCD     TIM4_IRQHandler           ; TIM4
        DCD     I2C1_EV_IRQHandler        ; I2C1 Event
        DCD     I2C1_ER_IRQHandler        ; I2C1 Error
        DCD     I2C2_EV_IRQHandler        ; I2C2 Event
        DCD     I2C2_ER_IRQHandler        ; I2C2 Error
        DCD     SPI1_IRQHandler           ; SPI1
        DCD     SPI2_IRQHandler           ; SPI2
        DCD     USART1_IRQHandler         ; USART1
        DCD     USART2_IRQHandler         ; USART2
        DCD     USART3_IRQHandler         ; USART3
        DCD     EXTI15_10_IRQHandler      ; EXTI Line 15..10
        DCD     RTCAlarm_IRQHandler       ; RTC Alarm through EXTI Line
        DCD     USBWakeUp_IRQHandler      ; USB Wakeup from suspend
        DCD     TIM8_BRK_IRQHandler       ; TIM8 Break
        DCD     TIM8_UP_IRQHandler        ; TIM8 Update
        DCD     TIM8_TRG_COM_IRQHandler   ; TIM8 Trigger and Commutation
        DCD     TIM8_CC_IRQHandler        ; TIM8 Capture Compare
        DCD     ADC3_IRQHandler           ; ADC3
        DCD     FSMC_IRQHandler           ; FSMC
        DCD     SDIO_IRQHandler           ; SDIO
        DCD     TIM5_IRQHandler           ; TIM5
        DCD     SPI3_IRQHandler           ; SPI3
        DCD     UART4_IRQHandler          ; UART4
        DCD     UART5_IRQHandler          ; UART5
        DCD     TIM6_IRQHandler           ; TIM6
        DCD     TIM7_IRQHandler           ; TIM7
        DCD     DMA2_Channel1_IRQHandler  ; DMA2 Channel1
        DCD     DMA2_Channel2_IRQHandler  ; DMA2 Channel2
        DCD     DMA2_Channel3_IRQHandler  ; DMA2 Channel3
        DCD     DMA2_Channel4_5_IRQHandler ; DMA2 Channel4 & Channel5
;******************************************************************************************
;*  Reset entry
;******************************************************************************************
        EXPORT  Reset_Handler
Reset_Handler
        IMPORT  __main
        LDR     R0, =__main
        BX      R0


;******************************************************************************************
;*  NMI exception handler. 
;*  It simply enters an infinite loop.
;******************************************************************************************
NMIException
        B       NMIException


;******************************************************************************************
;*  Fault interrupt handler. 
;*  It simply enters an infinite loop.
;******************************************************************************************
HardFaultException
        B       HardFaultException

;******************************************************************************************
;*  MemManage interrupt handler. 
;*  It simply enters an infinite loop.
;******************************************************************************************
MemManageException
        B       MemManageException

;******************************************************************************************
;*  Bus Fault interrupt handler. 
;*  It simply enters an infinite loop.
;******************************************************************************************
BusFaultException
        B       BusFaultException

;******************************************************************************************
;*  UsageFault interrupt handler. 
;*  It simply enters an infinite loop.
;******************************************************************************************
UsageFaultException
        B       UsageFaultException

;******************************************************************************************
;*  DebugMonitor interrupt handler. 
;*  It simply enters an infinite loop.
;******************************************************************************************
DebugMonitor
        B       DebugMonitor

;******************************************************************************************
;*  SVCall interrupt handler. 
;*  It simply enters an infinite loop.
;******************************************************************************************
SVCHandler
        B       SVCHandler



;*******************************************************************************************
;*  Make sure the end of this section is aligned.
;*******************************************************************************************
        ALIGN


;********************************************************************************************
;*  Code section for initializing the heap and stack                                                                                                          
;********************************************************************************************
		AREA    |.text|, CODE, READONLY


;********************************************************************************************
;*  The function expected of the C library startup 
;*  code for defining the stack and heap memory locations. 
;********************************************************************************************
        IMPORT  __use_two_region_memory
        EXPORT  __user_initial_stackheap 
__user_initial_stackheap
        LDR     R0, =Heap
        LDR     R1, =(Stack + Stack_Size)
        LDR     R2, =(Heap + Heap_Size)
        LDR     R3, =Stack
        BX      LR

;******************************************************************************************
;*  Make sure the end of this section is aligned.
;******************************************************************************************
        ALIGN


;*******************************************************************************************
;*  End Of File                                                     
;*******************************************************************************************
        END


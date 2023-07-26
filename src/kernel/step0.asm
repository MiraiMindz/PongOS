; This is the first step of the booting process, it's limited to the first
; sector of the disk, (512 bytes), so here I will need to make the CPU load
; the sub-sequential sectors in order to have more space to work, essentially
; creating a multi-step boot process similar to how GRUB does it.

; MACROS
%define MEMOFFSET 0x0
%define ENDL 0x0D, 0x0A

; NASM Directives
org         MEMOFFSET   ; Declares the memory offset
bits        16          ; Tells the assembler to emit 16 bits compatible code

; "Variables" Section [00] {{{

msg: db "Hello From the KERNEL", ENDL, 0
;MEMOFFSET   dw 0x7C00

; [00] }}}

start:
    jmp main            ; Makes the procedures be before main & prevent errors

; Procedures [01] {{{
;
; Prints a String into the Screen.
;   Parameters:
;       * DS:SI - Points to the String
;
puts:
    ; Preserve Registers
    push si
    push ax

.chloop:
    lodsb   ; Load a byte from DS:SI into the A Register (AL/AX/EAX/RAX) and \
            ; them increments SI by the number of bytes loaded.
            ; Read More in:
            ;   asmdude.github.io/x86doc/html/LODS_LODSB_LODSW_LODSD_LODSQ.html
    or al, al   ; Sets the ZeroFlag if AL is Zero.
    jz  .chdone
    sti             ; Enables interrupts
    mov ah, 0x0E    ; Set the Interrupt mode to: "Write Character in TTY Mode"
    mov bh, 0       ; Set the PAGE number (text mode) to 0  
    int 0x10        ; 

    jmp .chloop

.chdone:
    cli     ; Disables BIOS Interupts again
    ; Restores the Registers
    pop ax
    pop si
    ret
; [01] }}}

; Start of program
main:
    mov si, msg
    call puts

    hlt                 ; Halt the CPU (Hang)

.halt:
    jmp     .halt       ; Fallback halting, in case the CPU continues executing



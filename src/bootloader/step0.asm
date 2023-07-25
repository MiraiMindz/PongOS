; This is the first step of the booting process, it's limited to the first
; sector of the disk, (512 bytes), so here I will need to make the CPU load
; the sub-sequential sectors in order to have more space to work, essentially
; creating a multi-step boot process similar to how GRUB does it.

; MACROS
%define MEMOFFSET 0x7C00
%define ENDL 0x0D, 0x0A

; NASM Directives
org         MEMOFFSET   ; Declares the memory offset
bits        16          ; Tells the assembler to emit 16 bits compatible code

; FAT 12 HEADERS
jmp short start
nop

bdb_oem:                    db  'MSWIN4.1'              ; 8 bytes
bdb_bytes:                  dw  0x200                   ; 512 bytes
bdb_sectors_per_cluster:    db  1                       ; 1 Sector Per Cluster
bdb_reserved_sectors:       dw  1                       ; 1 Reserved Sector
bdb_fat_count:              db  2                       ; Common Value
bdb_dir_entries_count:      dw  0x11                    ; Number of Dir Entries
bdb_total_sectors:          dw  2000                    ; 512 * 2000 = 2MiB
bdb_media_descriptor_type:  db  0xF8                    ; F8 = Hard Disk
bdb_sectors_per_fat:        dw  9                       ; 9 Sectors/Fat
bdb_sectors_per_track:      dw  18                      ; Sectors Per Fat * Head Counts
bdb_heads:                  dw  2                       ; Number of heads
bdb_hidden_sectors:         dd  0                       ; Hidden Sectors Count
bdb_large_sector_count:     dd  0                       ; Number of Large Sectors

; Extended Boot Record (EBR)
ebr_driver_number:          db  0x80                    ; HDD
                            db  0                       ; Reserved Byte
ebr_signature:              db 0x28                     ; Either 0x28 or 0x29
ebr_volume_id:              db 0x12, 0x34, 0x56, 0x78   ; Serial Number
ebr_volume_label:           db 'PINGPONG OS'            ; 11 Bytes
ebr_system_id:              db 'FAT12   '               ; 8 Bytes



; "Variables" Section [00] {{{

msg:                db  "Hello World",          ENDL,   0
msg_disk_error:     db  "READ DISK FAILED.",    ENDL,   0
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

; DISK UTILS

; Converts a LBA Address to the CHS Scheme Address
;   Parameters:
;       - AX:                           LBA Address
;   Returns:
;       - CX (Lower) [00 - 05 bits]:    Sector Number
;       - CX (Upper) [06 - 15 bits]:    Cylinder
;       - DH:                           Head
;
lba_to_chs:
    ; Preserve the used Registers
    push    ax
    push    dx

    ; First Step/Division
    xor     dx, dx                              ; Sets DX to Zero
    div     word    [bdb_sectors_per_track]     ; AX = LBA / Secs Per Track
                                                ; AX = Result
                                                ; DX = LBA % Secs Per Track
    inc     dx                                  ; DX = LBA % Secs Per Track + 1
    mov     cx, dx                              ; CX = Sectors

    ; Second Step/Division
    xor     dx, dx              ; Sets DX to Zero
    div     word    [bdb_heads] ; AX = (LBA / Secs Per Track) / Heads = Cylinder
                                ; DX = (LBA / Secs Per Track) % Heads = Head
    mov     dh, dl              ; DH = Head
    mov     ch, al              ; CH (Lower 8 Bits)
    shl     ah, 6               ; Left Shifts 6 bits from AH
    or      cl, ah              ; Sets the 2 upper bits of cylinder

    ; Restores the used Registers
    pop     ax
    mov     dl, al              ; Restores DL
    pop     ax
    ret

; Reads the Sectors from the disk
;   Parameters:
;       - AX:       LBA Address
;       - CL:       Numbers of Sectors to READ (MAX VAL: 128)
;       - CL:       Drive Number
;       - ES:BX:    Memory Address to store read data.
;
read_from_disk:
    push    ax
    push    bx
    push    cx
    push    dx
    push    di

    push    cx          ; Preserves CL (Lower Half)
    call    lba_to_chs  ; Computes CHS
    pop     ax          ; AL = number of Sectors to READ

    ; BIOS INT 0x13, AH = 2 (0x13;2) READ DISK SECTOR FUNCION
    mov     ah, 0x2     ; READ DISK MODE
    mov     di, 5       ; Number of Retrys

.read_from_disk_retrys:    
    pusha                           ; Preserve all Registers
    stc                             ; set Carry Flag (some BIOS'es don't set it)
    int     0x13                    ; BIOS INTERRUPT (CF cleared = Success)
    jnc     .read_from_disk_done
    
    ; Failed
    popa
    call    disk_reset
    dec     di
    test    di, di
    jnz     .read_from_disk_retrys

.read_from_disk_fail
    jmp     disk_error


.read_from_disk_done:
    popa                            ; Restore Registers
    pop     di
    pop     dx
    pop     cx
    pop     bx
    pop     ax
    ret

; Resets disk controller
;   Parameters:
;       - DL:   Drive Number
disk_reset:
    pusha
    mov     ah, 0
    stc
    int     0x13
    jc      disk_error
    popa
    ret

; Start of program
main:
    cli                 ; Disables interrupts
    
    ; Set Registers to default values [02] {{{
    
    ; Data Registers
    xor ax, ax          ; set AX to Zero
    mov ds, ax
    mov es, ax

    ; Stack Registers
    mov ss, ax
    mov sp, MEMOFFSET

    ; [02] }}}

    ; READS from DISK
    mov [ebr_driver_number], dl
    mov ax, 1
    mov cl, 1
    mov bx, 0x7E00      ; Data should be after the bootloader
    call read_from_disk

    mov si, msg
    call puts

    hlt                 ; Halt the CPU (Hang)

disk_error:
    mov     si, msg_disk_error
    call    puts
    jmp     wait_key_reboot

wait_key_reboot:
    mov     ah, 0
    int     0x16        ; Waits for KeyPress
    jmp     0xFFFF:0    ; Jumps to beginning of BIOS (Rebooting)

.halt:
    cli
    jmp     .halt       ; Fallback halting, in case the CPU continues executing

padsec:
    times   510-($-$$)  db  0   ; Fills the sectior with zeros
    dw      0xAA55              ; Declares the boot signature

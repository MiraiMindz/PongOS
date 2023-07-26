; This is the first step of the booting process, it's limited to the first
; sector of the disk, (512 bytes), so here I will need to make the CPU load
; the sub-sequential sectors in order to have more space to work, essentially
; creating a multi-step boot process similar to how GRUB does it.

; MACROS
%define MEMOFFSET               0x7C00
%define ENDL                    0x0D,           0x0A
%define BOOT_SIGNATURE          0xAA55
%define KERNEL_BIN_FILENAME     "KERNEL  BIN"
%define FAT_BYTES_PER_SECTOR    0x200
%define FAT_TOTAL_OF_SECTORS    2000
%define EBR_VOLUME_LABEL        "PINGPONG OS"
%define FAT_FILENAME_SIZE       11
%define DIRECTORY_ENTRY_SIZE    32
%define KERNEL_LOAD_SEGMENT     0x2600
%define KERNEL_LOAD_OFFSET      0

; NASM Directives
org         MEMOFFSET   ; Declares the memory offset
bits        16          ; Tells the assembler to emit 16 bits compatible code

; FAT 12 HEADERS
jmp short start
nop

bdb_oem:                    db  'MSWIN4.1'              ; 8 bytes
bdb_bytes:                  dw  FAT_BYTES_PER_SECTOR    ; 512 bytes
bdb_sectors_per_cluster:    db  1                       ; 1 Sector Per Cluster
bdb_reserved_sectors:       dw  1                       ; 1 Reserved Sector
bdb_fat_count:              db  2                       ; Common Value
bdb_dir_entries_count:      dw  0xE0                    ; Number of Dir Entries
bdb_total_sectors:          dw  FAT_TOTAL_OF_SECTORS    ; 512 * 2000 = 2MiB
bdb_media_descriptor_type:  db  0xF8                    ; F8 = Hard Disk
bdb_sectors_per_fat:        dw  9                       ; 9 Sectors/Fat
bdb_sectors_per_track:      dw  18                      ; Sectors Per Fat * Head Counts
bdb_heads:                  dw  2                       ; Number of heads
bdb_hidden_sectors:         dd  0                       ; Hidden Sectors Count
bdb_large_sector_count:     dd  0                       ; Number of Large Sectors

; Extended Boot Record (EBR)
ebr_driver_number:          db  0x80                    ; HDD
                            db  0                       ; Reserved Byte
ebr_signature:              db  0x28                    ; Either 0x28 or 0x29
ebr_volume_id:              db  0x12, 0x34, 0x56, 0x78  ; Serial Number
ebr_volume_label:           db  EBR_VOLUME_LABEL        ; 11 Bytes
ebr_system_id:              db  'FAT12   '              ; 8 Bytes

; "Variables" Section [00] {{{
; MESSAGES
msg_disk_error:                 db      "DSK ERR",      ENDL,   0
msg_kernel_was_not_found_error: db      "KRN ERR",      ENDL,   0
; OTHER VARIABLES
kernel_bin_file_name:           db  KERNEL_BIN_FILENAME
kernel_cluster:                 dw  0
buffer:
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
    mov ah, 0x0E    ; Set the Interrupt mode to: "Write Character in TTY Mode"
    mov bh, 0       ; Set the PAGE number (text mode) to 0  
    int 0x10        ; 

    jmp .chloop

.chdone:
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

.read_from_disk_fail:
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
    ; Enables video mode
    mov ah, 0x00        ; Function 00h - Set Video Mode
    mov al, 0x13        ; Video Mode 13h (320x200, 256 colors)
    int 0x10            ; BIOS video services interrupt
    
    ; Set Registers to default values [02] {{{
    
    ; Data Registers
    xor ax, ax          ; set AX to Zero
    mov ds, ax
    mov es, ax

    ; Stack Registers
    mov ss, ax
    mov sp, MEMOFFSET
    ; [02] }}}
    
    push es
    push word .after

.after: 
    ; Reads the Driver Parameters using the BIOS
    sti
    push    es
    mov     ah, 0x08
    int     0x13
    jc      disk_error
    pop     es

    and     cl, 0x3F
    xor     ch, ch
    mov     [bdb_sectors_per_track], cx

    inc     dh
    mov     [bdb_heads], dh

    ; Reads FAT ROOT DIRECTORY
    mov     ax, [bdb_sectors_per_fat]
    mov     bl, [bdb_fat_count]
    xor     bh, bh
    mul     bx                          ; AX = (FATS * Sectors Per FAT)
    add     ax, [bdb_reserved_sectors]  ; AX = LBA of ROOT DIRECTORY

    ; Gets size of ROOT DIRECTORY
    mov     ax, [bdb_sectors_per_fat]
    shl     ax, 5
    xor     dx, dx
    div     word [bdb_bytes]

    test    dx, dx              ; If dx != 0; then add 1
    jz      .fat_root_dir_after
    inc     ax                  ; If division remainder != 0; then add 1 \
                                ; this means we have only a single sector

.fat_root_dir_after:
    mov     cl, al
    pop     ax
    mov     dl, [ebr_driver_number]
    mov     bx, buffer       ; es:bx = buffer
    call    read_from_disk
    
    ; Searchs for KERNEL.BIN file
    xor bx, bx
    mov di, buffer

.search_for_kernel:
    mov     si, kernel_bin_file_name
    mov     cx, FAT_FILENAME_SIZE
    push    di
    repe    cmpsb       ; Repeates While Equals the Compare String Bytes
    pop     di
    je      .kernel_was_found
    add     di, DIRECTORY_ENTRY_SIZE    ; Moves to the next Dir Entry
    inc     bx
    cmp     bx, [bdb_dir_entries_count]
    jl      .search_for_kernel

    ; Kernel Was Not Found
    jmp     kernel_was_not_found_error

.kernel_was_found:
    mov     ax, [di + 26]           ; FAT specs says that the offset of \ 
                                    ; the first logical cluster is of 26 bits.
    mov     [kernel_cluster], ax
    
    ; Reads the FAT (File Allocation Table) from the DISK into the MEMORY.
    mov     ax, [bdb_reserved_sectors]
    mov     bx, buffer
    mov     cl, [bdb_sectors_per_fat]
    mov     dl, [ebr_driver_number]
    call    read_from_disk
    
    ; Checks if kernel is greater than 64KB
    mov     ax, [di + 28]   ; Gets the FILE SIZE
    cmp     ax, 0xFFFF      ; 65535 Bytes [MAX 16bit Register Value]
    jbe     .preloads_kernel
    inc     word [bdb_total_sectors]

.preloads_kernel:
    ; Reads the KERNEL and process any FAT CHAIN.
    mov     bx,     KERNEL_LOAD_SEGMENT
    mov     es,     bx
    mov     bx,     KERNEL_LOAD_OFFSET

.load_kernel:
    ; Here I'll need to calculate the First Cluster Number
    ; The formula is:
    ;   First Cluster = (Kernel Cluster - 2) * Sectors Per Cluster + ( \
    ;       (Fat Count * Sectors per fat) + ((Dir entry count * Dir size + \
    ;           Bytes Per Sec - 1) / Bytes Per Sec) + RESERVED SECTORS)

    mov     ax,     [kernel_cluster]            ; Gets the Kernel Cluster
    sub     ax,     2                           ; Subtract 2 from the Kernel Cluster
    mul     word    [bdb_sectors_per_cluster]   ; Multiply the result by Sectors Per Cluster
    add     ax,     [bdb_reserved_sectors]      ; Add RESERVED SECTORS to the result

    ; Calculate the size of the root directory in sectors
    mov     ax,     [bdb_dir_entries_count]
    mul     word    [DIRECTORY_ENTRY_SIZE]
    add     ax,     [bdb_bytes]
    ;dec     ax
    div     word    [bdb_bytes]
    add     ax,     [bdb_reserved_sectors]      ; Add size of the root directory in sectors
    add     ax,     [bdb_sectors_per_fat]       ; Add (Fat Count * Sectors per fat)

    mov     cl,     1
    mov     dl,     [ebr_driver_number]
    call    read_from_disk

    add     bx,     [bdb_bytes]
    
    ; Computes the location of the next cluster
    mov     ax,     [kernel_cluster]
    mov     cx,     3
    mul     cx
    mov     cx,     2
    div     cx                      ; AX = Index entry in FAT, DX = Cluster % 2

    mov     si,     buffer
    add     si,     ax
    mov     ax,     [ds:si]         ; read entry from FAT into AX
    or      dx,     dx
    jz      .even

.odd:
    shr     ax, 4
    jmp     .after_next_cluster

.even:
    and     ax, 0x0FFF

.after_next_cluster:
    cmp     ax, 0x0FF8              ; END OF FILE CHAIN
    jae     .read_file_finished
    
    mov     [kernel_cluster], ax
    jmp     .load_kernel

.read_file_finished:
    ; Goes to our Kernel
    mov     dl,     [ebr_driver_number]         ; Boot device is stored in DL
    mov     ax,     KERNEL_LOAD_SEGMENT   ; Set Segments register
    mov     ds,     ax
    mov     es,     ax

    jmp     KERNEL_LOAD_SEGMENT:KERNEL_LOAD_OFFSET

    jmp     wait_key_reboot                     ; In case of failure

    hlt                                         ; Halt the CPU (Hang)

; ERRORS SECTION
kernel_was_not_found_error:
    mov     si, msg_kernel_was_not_found_error
    call    puts
    jmp     wait_key_reboot

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
    dw      BOOT_SIGNATURE      ; Declares the boot signature


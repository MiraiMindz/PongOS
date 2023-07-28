; This is the first part of the Booting process, limited to only 512 bytes \
; I'll merely switch to graphical mode and load the next sector in the disk \
; before loading my kernel.

; MACROS ======================================================================
%define BOOTLOADER_OFFSET               0x7C00
%define ENDL                            0x0D,       0x0A
%define BOOT_SIGNATURE                  0xAA55
%define BIOS_DATA_OEM                   "MSWIN4.1"          ; 8 Bytes string
%define BIOS_BYTES_PER_SECTOR           0x200               ; 512 Bytes
%define BIOS_SECTORS_PER_CLUSTER        0x1                 ; 1 Sector
%define BIOS_RESERVED_SECTORS           0x1                 ; 1 Sector
%define BIOS_FAT_COUNT                  0x2                 ; 2 FATs
%define BIOS_DIR_ENTRIES_COUNT          0xE0                ; 14 Dirs
%define BIOS_TOTAL_SECTORS              0xB40               ; 2880 Sectors
%define BIOS_MEDIA_DESCRIPTOR_TYPE      0xF0                ; 3.5" Floppy
%define BIOS_SECTORS_PER_FAT            0x9                 ; 9 Sectors
%define BIOS_SECTORS_PER_TRACK          0x12                ; 18 Secs per Track
%define BIOS_HEADS                      0x2                 ; 2 Heads
%define BIOS_HIDDEN_SECTORS             0x0                 ; 0
%define BIOS_LARGE_SECTORS_COUNT        0x0                 ; 0
%define BOOT_RECORD_DRIVE_NUMBER        0x0                 ; 0 = Floppy
%define BOOT_RECORD_RESERVED_BYTE       0x0                 ; Reserved
%define BOOT_RECORD_SIGNATURE           0x29                ; 0x28 || 0x29
%define BOOT_RECORD_VOLUME_ID           0x1, 0x2, 0x3, 0x4  ; Random 4 Bytes
%define BOOT_RECORD_VOLUME_LABEL        "PINGPONG OS"       ; 11 bytes
%define BOOT_RECORD_SYSTEM_ID           "FAT12   "          ; 8 bytes
%define NUMBER_OF_READ_TRYS             0x8                 ; 8 Trys
%define BIOS_BEGINNING                  0xFFFF:0            ; Start of BIOS
%define FAT_DIR_SIZE                    0x20                ; 32
%define NEXT_STEP_FILENAME              "STEPONE BIN"       ; 11 Bytes String
%define FAT_FILE_NAME_SIZE              0x0B                ; 11
%define NEXT_STEP_LOAD_SEGMENT          0x2000              ; ~ 380KiB of Mem.
%define NEXT_STEP_LOAD_OFFSET           0x0                 ; None offset.
%define NEXT_STEP_MAX_SIZE              0xFFFF              ; < 64KiB 

; DIRECTIVES ==================================================================
org     BOOTLOADER_OFFSET                   ; Bootloader offset
bits    16                                  ; Tells the assembler to emit \
                                            ; 16 bits machine code.

; HEADERS =====================================================================
; -- FAT 12 -------------------------------------------------------------------
; ------ BIOS PARAMETER BLOCK (BPD) -------------------------------------------
jmp     short       main
nop

bdb_oem:                        db          BIOS_DATA_OEM
bdb_bytes_per_sector:           dw          BIOS_BYTES_PER_SECTOR
bdb_sectors_per_cluster:        db          BIOS_SECTORS_PER_CLUSTER
bdb_reserved_sectors:           dw          BIOS_RESERVED_SECTORS
bdb_fat_count:                  db          BIOS_FAT_COUNT
bdb_dir_entries_count:          dw          BIOS_DIR_ENTRIES_COUNT
bdb_total_sectors:              dw          BIOS_TOTAL_SECTORS
bdb_media_descriptor_type:      db          BIOS_MEDIA_DESCRIPTOR_TYPE
bdb_sectors_per_fat:            dw          BIOS_SECTORS_PER_FAT
bdb_sectors_per_track:          dw          BIOS_SECTORS_PER_TRACK
bdb_heads:                      dw          BIOS_HEADS
bdb_hidden_sectors:             dd          BIOS_HIDDEN_SECTORS
bdb_large_sectors_count:        dd          BIOS_LARGE_SECTORS_COUNT

; ------ EXTENDED BOOT RECORD (EBR) -------------------------------------------
ebr_drive_number:               db          BOOT_RECORD_DRIVE_NUMBER
                                db          BOOT_RECORD_RESERVED_BYTE
ebr_signature:                  db          BOOT_RECORD_SIGNATURE
ebr_volume_id:                  db          BOOT_RECORD_VOLUME_ID
ebr_volume_label:               db          BOOT_RECORD_VOLUME_LABEL
ebr_system_id:                  db          BOOT_RECORD_SYSTEM_ID

; VARIABLES/CONSTANTS =========================================================
; ----- CONSTANTS -------------------------------------------------------------
MEMORY_OFFSET   equ     BOOTLOADER_OFFSET   ; Sets the STACK offset
; ----- VARIABLES -------------------------------------------------------------
msg_read_error:     db      "ER0",     ENDL,   0
msg_disk_error:     db      "ER1",     ENDL,   0
msg_state_loading:  db      "LDN",     ENDL,   0
msg_boot_error:     db      "ER2",     ENDL,   0
next_step_filename: db      NEXT_STEP_FILENAME
next_step_cluster:  dw      0
; CODE ========================================================================

main: 
    cli                     ; Disables Interrupts

    ; Clear DATA Registers
    xor     ax,     ax      ; Zero out AX
    mov     ds,     ax
    mov     es,     ax

    ; Clear STACK Register
    mov     ss,     ax
    mov     sp,     MEMORY_OFFSET

    ; Sets Code Segment to Zero
    push    es
    push    word    .code
    retf                                    ; Does a far return
.code:
    ; Main Code
    
    mov     si, msg_state_loading
    call    print


    ; Uses the BIOS to load things from the DISK
    
    mov     [ebr_drive_number], dl
    sti
    push    es
    mov     ah,     0x08                ; BIOS Get Drive Parameters Function
    int     0x13                        ; Calls the BIOS
    jc      disk_error
    pop     es
    and     cl,     0x3F                ; Remove the top 2 bytes
    xor     ch,     ch
    mov     [bdb_sectors_per_track], cx ; Sector Counts
    inc     dh
    mov     [bdb_heads],             dh ; Head Count
    cli

    ;   Reads the File Allocation Table Root Directory
    mov     ax,     [bdb_sectors_per_fat]   ; LBA of Root = (reserved + fats +\
    ;                                                       Sectors per FAT)
    mov     bl,     [bdb_fat_count]
    xor     bh,     bh                      ; Zeros the lower half of BX (LE) \
                                            ;          Little Endian Notation     
    mul     bx                              ; AX = FATs * Sectors Per FAT
    add     ax,     [bdb_reserved_sectors]  ; AX = LBA of FAT Root Directory
    ; Gets the SIZE in Sectors of the Root Directory \
    ;   Formula: (DIRS * DIR Size) / Bytes per Sectors
    ; Quick note: 32 = 2^5
    mov     ax,     [bdb_sectors_per_fat]
    shl     ax,     5                       ; Same as multiplying by 32
    xor     dx,     dx
    div     word    [bdb_bytes_per_sector]
    test    dx,     dx
    jz      .after_reading_fat_root_dir
    inc     ax                              ; if the remainder is != 0, add 1

.after_reading_fat_root_dir:
    ; Reads FAT Root DIR
    mov     cl,     al                      ; CL = Number of Sectors to read \ 
                                            ;       = Size of Root directory
    pop     ax                              ; AX = LBA of ROOT DIR
    mov     dl,     [ebr_drive_number]      ; DL = Drive number
    mov     bx,     buffer                  ; your buffer to WRITE into
    call    read_sector_from_disk

    ; Searches to the Next Boot Step
    xor     bx,     bx                      ; Number of Entries found
    mov     di,     buffer                  ; Current Entry

.search_for_next_boot_step:
    mov     si,     next_step_filename
    mov     cx,     11
    push    di                              ; Preserves DI
    repe    cmpsb                           ; Compare 2 strings (Bytes array) \
                                            ; Stored in the memory locations \
                                            ; of DS:SI & ES:DI, and SI & DI \
                                            ; are incremented.
                                            ; the REPE instructions is \
                                            ; Repeats while Equals, or until \
                                            ; CX reaches Zero. It basically \
                                            ; repeats a instruction while both\
                                            ; operands are equals.
    pop     di                              ; Restores DI
    je      .found_next_step

    ; Moves to the next Directory entry
    add     di,     FAT_DIR_SIZE
    inc     bx                              ; Increment the number of entries \
                                            ; checked.
    cmp     bx,     [bdb_dir_entries_count] ; Checks if reached the limit.
    jl      .search_for_next_boot_step

    ; Next Step was not found
    jmp     multi_booting_error

.found_next_step:
    mov     ax,                     [di + 26]   ; DI + FAT OFFSET
    mov     [next_step_cluster],    ax
    
    ; Loads FAT
    mov     ax,     [bdb_reserved_sectors]
    mov     bx,     buffer
    mov     cl,     [bdb_sectors_per_fat]
    mov     dl,     [ebr_drive_number]
    call    read_sector_from_disk

.checks_next_step_size:
    cmp     ax,     NEXT_STEP_MAX_SIZE
    jbe     .continues_loading
    inc     word    [bdb_total_sectors]

.continues_loading:
    ; Loads the next step and Process any FAT Chain
    ;   The most contiguous memory location is between 0x7E00 & 0x7FFF \
    ;   Since we are using some space to store the Fat data, we should not use\
    ;   0x7E00 directly (kinda of), so I'll pick some recommended values like \
    ;   0x2000
    mov     bx,     NEXT_STEP_LOAD_SEGMENT
    mov     es,     bx
    mov     bx,     NEXT_STEP_LOAD_OFFSET

.load_next_boot_step_loop: 
    ; Read & Calculates the First Cluster Location \
    ; Formula: First Cluster = (Next Step Cluster - 2) * Sectors Per Cluster +\
    ;   (Reserved Sectors + FATs + Root Directory Size)
    push    cx                                  ; Preserves any register

    mov     ax,     [next_step_cluster]
    sub     ax,     2
    mov     cx,     [bdb_sectors_per_cluster]
    mul     cx

    add     ax,     [bdb_reserved_sectors]
    mov     cl,     [bdb_fat_count]
    xor     ch,     ch
    mul     cx                                  ; AX = [bdb_fat_count] * \
                                                ;   [bdb_sectors_per_fat]
    add     ax,     [bdb_reserved_sectors]
    add     ax,     [bdb_dir_entries_count]
    pop     cx

    mov     cl,     1
    mov     dl,     [ebr_drive_number]
    call    read_sector_from_disk
    add     bx,     [bdb_bytes_per_sector]
    mov     cx,     3
    mul     cx
    mov     cx,     2
    div     cx

    mov     si,     buffer
    add     si,     ax
    mov     ax,     [ds:si]

    or      dx,     dx
    jz      .even

.odd:
    shr     ax,     4
    jmp     .after_reading_next_cluster

.even:
    and     ax,     0x0FFF

.after_reading_next_cluster:
    cmp     ax,     0x0FF8
    jae     .finished_reading_file

    mov     [next_step_cluster],    ax
    jmp     .load_next_boot_step_loop

.finished_reading_file:
    mov     dl,     [ebr_drive_number]
    mov     ax,     NEXT_STEP_LOAD_SEGMENT
    mov     ds,     ax
    mov     es,     ax

    jmp     NEXT_STEP_LOAD_SEGMENT:NEXT_STEP_LOAD_OFFSET

    ; Jump Failed
    jmp     key_press_wait_and_reboot

    ; This will be done in the final of the boot process
    ; Switching to graphical mode (320x200-256c)
    ;xor     ax,     ax      ; Ensures AL & AH are blank
    ;mov     ah,     0x00    ; Function 00h (Set Video Mode)
    ;mov     al,     0x13    ; Video Mode 13h (320x200 256 Colors)
    ;int     0x10            ; BIOS 10h (Video Interrupt)

    hlt                                     ; Halts the CPU


; ----- ERRORS ----------------------------------------------------------------
disk_error:
    mov     si,     msg_disk_error
    call    print
    jmp     key_press_wait_and_reboot

multi_booting_error:
    mov     si,     msg_boot_error
    call    print
    jmp     key_press_wait_and_reboot

key_press_wait_and_reboot:
    sti
    mov     ah,     0x0
    int     0x16            ; Waits for Key Press
    cli
    jmp     BIOS_BEGINNING

; ----- SAFE GUARDS -----------------------------------------------------------
.halt:                                      ; In case of failure use this \
    jmp .halt                               ; recusive halt.

; SUBROUTINES =================================================================
; -----------------------------------------------------------------------------
; Prints a string to the screen
;   Parameters:
;       * DS:SI - Points to the STRING
print:
    sti                     ; Enables Interrupts
    ; Preserve Register
    push    si
    push    ax

.charloop:
    lodsb                   ; Loads a byte from DS:SI into the A Register and \
                            ; increments SI by the number of bytes loaded.
                            ; Read more at:
                            ;   asmdude.github.io/x86doc/html/ \
                            ;       LODS_LODSB_LODSW_LODSD_LODSQ.html
    or      al,     al      ; if AL is Zero, sets the ZeroFlag (End of String)
    jz      .chardone       ; Finished Reading
    mov     ah,     0x0E    ; Set the INT mode of: Write Char in TTY Mode
    mov     bh,     0x00    ; Sets Page Number (0 == Current)
    int     0x10            ; Calls the BIOS
    
    jmp .charloop           ; Creates a loop

.chardone:
    cli                     ; Disable Interrupts
    ; Restores registers
    pop     ax
    pop     si
    ret                     ; Return from the subroutine


; -----------------------------------------------------------------------------
; LBA to CHS Converter Function
;   Parameters:
;       * AX -  LBA Address
;   Returns:
;       * CH -  Sector Number
;       * CL -  Cylinder
;       * DH -  Head
lba_to_chs_converter:
    ; Preserve Registers
    push    ax
    push    dx

    xor     dx,     dx                      ; Ensures that DX is Zero
    div     word    [bdb_sectors_per_track] ; AX = LBA / Sectors Per Track
                                            ; AX = Result
                                            ; DX = LBA % Sectors Per Track
                                            
    inc     dx                              ; DX = DX + 1
    mov     cx,     dx                      ; CX = Sectors

    xor     dx,     dx                      ; Zeros out DX
    div     word    [bdb_heads]             ; AX = (LBA / SPT) / Heads = Cylin.
                                            ; DX = (LBA / SPT) % Heads = Head
    mov     dh,     dl                      ; DH = Head
    mov     ch,     al                      ; Sector Number
    shl     ah,     6                       ; Left Shifts 6 bits from AH
    or      cl,     ah                      ; Set the 2 upper cylinder bits
    
    ; Restores Registers
    pop     ax
    mov     dl,     al
    pop     ax
    ret

; -----------------------------------------------------------------------------
; Read sector from the DISK
;   Parameters:
;       * AX    -   LBA Address
;       * CL    -   Number of sectors to read (Maximum value of 128)
;       * DL    -   Drive Number
;       * ES:BX -   Memory Address to store read data
read_sector_from_disk:
    sti                                 ; Enables Interrupts
    
    ; Preserve Registers
    push    ax
    push    bx
    push    cx
    push    dx
    push    di
    
    push    cx                          ; Preserves CL (Again)
    
    call    lba_to_chs_converter
    pop     ax                          ; AX = Number of sectors to Read
    mov     ah,     0x02                ; Read from disk function
    mov     di,     NUMBER_OF_READ_TRYS

.read_sector_from_disk_loop:
    pusha                               ; Preserves all registers
    stc                                 ; Sets the Carry Flag
    int     0x13                        ; Calls the BIOS
    jnc     .read_sector_from_disk_done
    popa
    call    reset_disk
    dec     di
    test    di,     di
    jnz     .read_sector_from_disk_loop

.read_sector_from_disk_terminal_failure:
    jmp     disk_error

.read_sector_from_disk_done:
    popa                                ; Restores all registers
    cli                                 ; Disables Interrupts

    pop     di
    pop     dx
    pop     cx
    pop     bx
    pop     ax

    ret

; -----------------------------------------------------------------------------
; Reset the DISK Controller
;   Parameters
;       * DL -  Drive Number
reset_disk:
    sti
    pusha
    mov     ah,     0
    stc
    int     0x13
    cli
    jc      disk_error
    popa
    ret

; END OF SECTOR ===============================================================
padsector:
    times   510-($-$$)  db  0x0             ; Fills the remaining space with 0s
    dw      BOOT_SIGNATURE

; BUFFER TO NEXT SECTOR =======================================================
buffer:


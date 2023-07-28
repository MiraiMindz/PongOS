; This is the second part of the Booting process, limited to only 512 bytes \
; I'll merely switch to graphical mode and load the next sector in the disk \
; before loading my kernel.

; MACROS ======================================================================
%define ENDL                0x0D,   0x0A

; DIRECTIVES ==================================================================
org     0x0
bits    16                                  ; Tells the assembler to emit \
                                            ; 16 bits machine code.

; VARIABLES/CONSTANTS =========================================================
msg_test:       db      "SECOND PHASE",     ENDL,   0

; CODE ========================================================================

main: 
    cli                     ; Disables Interrupts

    ; Main Code

    mov     si,     msg_test
    call    print


    ; This will be done in the final of the boot process
    ; Switching to graphical mode (320x200-256c)
    ;xor     ax,     ax      ; Ensures AL & AH are blank
    ;mov     ah,     0x00    ; Function 00h (Set Video Mode)
    ;mov     al,     0x13    ; Video Mode 13h (320x200 256 Colors)
    ;int     0x10            ; BIOS 10h (Video Interrupt)

    hlt                                     ; Halts the CPU

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


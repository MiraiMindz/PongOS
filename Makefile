ASM=nasm

SRC_DIR=./src
BUILD_DIR=./build

.PHONY:	all disk kernel bootloader clean always

# DISK IMAGE SECTION
disk: $(BUILD_DIR)/disk.img

$(BUILD_DIR)/disk.img: bootloader kernel
	dd if=/dev/zero of=$(BUILD_DIR)/disk.img bs=512 count=2880
	mkfs.fat -F 12 -n "PONGOS" $(BUILD_DIR)/disk.img
	dd if=$(BUILD_DIR)/bootloader.bin of=$(BUILD_DIR)/disk.img conv=notrunc
	mcopy -i $(BUILD_DIR)/disk.img $(BUILD_DIR)/kernel.bin "::kernel.bin"

# BOOTLOADER SECTION
bootloader: $(BUILD_DIR)/bootloader.bin

$(BUILD_DIR)/bootloader.bin: always
	$(ASM) $(SRC_DIR)/bootloader/step0.asm -f bin -o $(BUILD_DIR)/bootloader.bin

# KERNEL SECTION
kernel: $(BUILD_DIR)/kernel.bin

$(BUILD_DIR)/kernel.bin: always
	$(ASM) $(SRC_DIR)/kernel/step0.asm -f bin -o $(BUILD_DIR)/kernel.bin


# UTILITIES SECTION
always:
	mkdir -pv $(BUILD_DIR)

clean:
	rm -rfv $(BUILD_DIR)/*


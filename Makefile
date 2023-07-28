ASM=nasm
SRC_DIR=src
BUILD_DIR=build

.PHONY:	all	bootloader disk clean always

disk: $(BUILD_DIR)/disk.img
$(BUILD_DIR)/disk.img:	bootloader
	dd if=/dev/zero of=$(BUILD_DIR)/disk.img bs=512 count=2880
	mkfs.fat -F 12	-n "PONGOS" $(BUILD_DIR)/disk.img
	dd if=$(BUILD_DIR)/step0.bin of=$(BUILD_DIR)/disk.img conv=notrunc
	mcopy -i $(BUILD_DIR)/disk.img $(BUILD_DIR)/stepone.bin "::stepone.bin"

bootloader: $(BUILD_DIR)/bootloader.bin
$(BUILD_DIR)/bootloader.bin: always
	$(ASM) $(SRC_DIR)/bootloader/step0.asm -f bin -o $(BUILD_DIR)/step0.bin
	$(ASM) $(SRC_DIR)/bootloader/step1.asm -f bin -o $(BUILD_DIR)/stepone.bin

always:
	mkdir -pv $(BUILD_DIR)

clean:
	rm -rfv $(BUILD_DIR)/*


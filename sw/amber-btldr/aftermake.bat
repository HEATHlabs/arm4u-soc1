arm-elf-objdump -D boot-loader.elf > boot-loader.lss2
..\tools\amber-bin2mem boot-loader.bin >boot-loader.mem
..\tools\mem2mif boot-loader.mem > boot-loader.mif
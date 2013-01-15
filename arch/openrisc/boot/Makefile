ifneq '$(CONFIG_OPENRISC_BUILTIN_DTB)' '""'
BUILTIN_DTB := $(patsubst "%",%,$(CONFIG_OPENRISC_BUILTIN_DTB)).dtb.o
else
BUILTIN_DTB :=
endif
obj-y += $(BUILTIN_DTB)

clean-files := *.dtb.S

#DTC_FLAGS ?= -p 1024

$(obj)/%.dtb: $(src)/dts/%.dts
	$(call cmd,dtc)

compression_name_$(CONFIG_KERNEL_GZIP) = gzip
compression_name_$(CONFIG_KERNEL_LZO)  = lzo
compression_name_$(CONFIG_KERNEL_LZMA) = lzma
ifeq ($(compression_name_y),)
compression := none
suffix :=
else
compression := $(compression_name_y)
suffix := .$(compression_name_y)
endif

MKIMAGE         := $(srctree)/scripts/mkuboot.sh

quiet_cmd_uimage = UIMAGE  $@
      cmd_uimage = $(CONFIG_SHELL) $(MKIMAGE) -A or1k -O linux -T kernel \
                   -C $(compression) -a 0 -e 0x100 \
                   -n 'Linux-$(KERNELRELEASE)' -d $< $@

targets := uImage

$(obj)/Image: vmlinux FORCE
	$(call if_changed,objcopy)
	@echo '  Kernel: $@ is ready'

ifneq ($(suffix),)
$(obj)/Image$(suffix): $(obj)/Image FORCE
	$(call if_changed,$(compression))
endif

$(obj)/uImage: $(obj)/Image$(suffix)
	$(call if_changed,uimage)
	@echo '  Image $@ is ready'

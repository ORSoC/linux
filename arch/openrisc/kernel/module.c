/*  Kernel module help for OpenRISC.

    OpenRISC bits:
	Copyright 2010 Jonas Bonn <jonas@southpole.se>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

#include <linux/moduleloader.h>
#include <linux/elf.h>
#include <linux/vmalloc.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/kernel.h>

void *module_alloc(unsigned long size)
{
	pr_debug("module_alloc size: %lu\n", size);

	if (size == 0)
		return NULL;

	return vmalloc(size);
}


/* Free memory returned from module_alloc */
void module_free(struct module *mod, void *module_region)
{
	vfree(module_region);
	/* FIXME: If module_region == mod->init_region, trim exception
           table entries. */
}

/* We don't need anything special. */
int module_frob_arch_sections(Elf_Ehdr *hdr,
			      Elf_Shdr *sechdrs,
			      char *secstrings,
			      struct module *mod)
{
	return 0;
}

int apply_relocate_add(Elf32_Shdr *sechdrs,
		       const char *strtab,
		       unsigned int symindex,
		       unsigned int relsec,
		       struct module *me)
{
	unsigned int i;
	Elf32_Rela *rel = (void *)sechdrs[relsec].sh_addr;
	Elf32_Sym *sym;
	Elf32_Addr relocation;
	uint32_t *location;
	uint32_t value;

	pr_debug("Applying relocate section %u to %u\n", relsec,
		 sechdrs[relsec].sh_info);
	for (i = 0; i < sechdrs[relsec].sh_size / sizeof(*rel); i++) {
		/* This is where to make the change */
		location = (void *)sechdrs[sechdrs[relsec].sh_info].sh_addr
			+ rel[i].r_offset;

		/* This is the symbol it is referring to.  Note that all
		   undefined symbols have been resolved.  */
		sym = (Elf32_Sym *)sechdrs[symindex].sh_addr
			+ ELF32_R_SYM(rel[i].r_info);
		value = sym->st_value + rel[i].r_addend;

		switch(ELF32_R_TYPE(rel[i].r_info)) {
		case R_OR32_32:
			*location = value;
			break;
		case R_OR32_CONST:
			location = (uint16_t*)location + 1;
			*((uint16_t*)location) = (uint16_t) (value);
			break;
		case R_OR32_CONSTH:
			location = (uint16_t*)location + 1;
			*((uint16_t*)location) = (uint16_t) (value >> 16);
			break;
		case R_OR32_JUMPTARG:
			value -= (uint32_t)location;
			value >>= 2;
			value &= 0x03ffffff;
			value |= *location & 0xfc000000;
			*location = value;
			break;
		default:
			pr_err("module %s: Unknown relocation: %u\n",
			       me->name);
			break;
		}
	}

	return 0;
}

int apply_relocate(Elf32_Shdr *sechdrs,
		   const char *strtab,
		   unsigned int symindex,
		   unsigned int relsec,
		   struct module *me)
{
	pr_err("module %s: REL relocation unsupported\n", me->name);
	return -ENOEXEC;
}

int module_finalize(const Elf_Ehdr *hdr,
		    const Elf_Shdr *sechdrs,
		    struct module *me)
{
 	return 0;
}

void module_arch_cleanup(struct module *mod)
{
}

/*
 * Copyright 2010 South Pole AB.  All Rights Reserved.
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
 */

#include <asm/io.h>

#ifndef _LINUX_WISHBONE_H
#define _LINUX_WISHBONE_H

#ifdef CONFIG_WISHBONE_BIG_ENDIAN
#define wb_ioread8(p)  ioread8(p)
#define wb_ioread16(p) ioread16be(p)
#define wb_ioread32(p) ioread32be(p)

#define wb_iowrite8(v, p)  iowrite8(v, p)
#define wb_iowrite16(v, p) iowrite16be(v, p)
#define wb_iowrite32(v, p) iowrite32be(v, p)

#else

#define wb_ioread8(p)  ioread8(p)
#define wb_ioread16(p) ioread16(p)
#define wb_ioread32(p) ioread32(p)

#define wb_iowrite8(p, v)  iowrite8(p, v)
#define wb_iowrite16(p, v) iowrite16(p, v)
#define wb_iowrite32(p, v) iowrite32(p, v)

#endif /* CONFIG_WISHBONE_BIG_ENDIAN */

#endif /* _LINUX_WISHBONE_H */

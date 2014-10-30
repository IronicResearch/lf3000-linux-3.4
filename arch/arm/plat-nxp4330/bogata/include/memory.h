/*
 * (C) Copyright 2009
 * jung hyun kim, Nexell Co, <jhkim@nexell.co.kr>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#ifndef __ASM_ARCH_MEMORY_H
#define __ASM_ARCH_MEMORY_H

#include "cfg_mem.h"

/*
 * Physical DRAM offset, depend on "CONFIG_NEED_MACH_MEMORY_H"
 * and unselect "CONFIG_ARM_PATCH_PHYS_VIRT"
 */
#define PHYS_OFFSET		CFG_MEM_PHY_SYSTEM_BASE

/*
 * avoid overlap dma zone and vmalloc area.
 *
 * refer to <asm/memory.h>
 * CONSISTENT_END = (0xffe00000UL)
 */
#if defined(CFG_MEM_PHY_DMAZONE_SIZE)
	#if (0xff000000UL > (0xffe00000UL - CFG_MEM_PHY_DMAZONE_SIZE))
	/* refer to pgtable.h */
	#define VMALLOC_END	(0xffe00000UL - CFG_MEM_PHY_DMAZONE_SIZE)
	#endif
#endif

#endif	/*  __ASM_ARCH_MEMORY_H */


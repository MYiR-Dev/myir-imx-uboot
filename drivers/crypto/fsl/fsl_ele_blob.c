// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2025 NXP
 */

#include "errno.h"
#include <asm/cache.h>
#include <asm/mach-imx/ele_api.h>
#include <fsl_sec.h>
#include "desc.h"

/**
 * blob_decap() - Decapsulate the data from a blob
 * @key_mod:    - Key modifier address
 * @src:        - Source address (blob)
 * @dst:        - Destination address (data)
 * @len:        - Size of decapsulated data
 * @keycolor    - Determines if the source data is covered (black key) or
 *                plaintext.
 *
 * Note: Start and end of the key_mod, src and dst buffers have to be aligned to
 * the cache line size (ARCH_DMA_MINALIGN) for the ELE operation to succeed.
 *
 * Returns zero on success, negative on error.
 */
int blob_decap(u8 *key_mod, u8 *src, u8 *dst, u32 len, u8 keycolor)
{
	u32 input_size = 0, output_size = 0;
	u32 src_addr = (u32)(uintptr_t)src, dst_addr = (u32)(uintptr_t)dst;
	int wrap_blob = 0;

	/* check address input */
	if (!(src && dst)) {
		debug("src_addr or dst_addr invalid\n");
		return -1;
	}

	/* input will be encrypted data blob with 32 bytes key blob in
	 * beginning and 16 byte HMAC identifier at end
	 */
	input_size = len + KEY_BLOB_SIZE + MAC_SIZE;

	/* output will be decrypted data blob */
	output_size = len;

	/* check if the address can be accessed by EdgeLock Secure Enclave */
	if (src_addr > ELE_MAX_ADDR - input_size || dst_addr > ELE_MAX_ADDR - output_size) {
		printf("Error: %s: Address is not in valid range(< 0xE0000000)!\n", __func__);
		return -EINVAL;
	}

	/* check the address alignment */
	if (!IS_ALIGNED((uintptr_t)key_mod, ARCH_DMA_MINALIGN) ||
	    !IS_ALIGNED((uintptr_t)src, ARCH_DMA_MINALIGN) ||
	    !IS_ALIGNED((uintptr_t)dst, ARCH_DMA_MINALIGN)) {
		printf("Error: %s: Address arguments are not aligned!\n", __func__);
		return -EINVAL;
	}

	debug("\nDecapsulating data to form blob\n");

	/* Flush the cache */
	flush_dcache_range(src_addr, src_addr + input_size);

	/* Flush the cache */
	flush_dcache_range(dst_addr, (u32)(dst_addr +
			   roundup(output_size, ARCH_DMA_MINALIGN)));

	/* Call ELE */
	if (ele_blob(0x00, src_addr, input_size, dst_addr,
		     output_size, wrap_blob))
		return -1;

	/* Invalidate output buffer */
	invalidate_dcache_range(dst_addr, (u32)(dst_addr +
				roundup(output_size, ARCH_DMA_MINALIGN)));
	return 0;
}

/**
 * blob_encap() - Encapsulate the data as a blob
 * @key_mod:    - Key modifier address
 * @src:        - Source address (data)
 * @dst:        - Destination address (blob)
 * @len:        - Size of data to be encapsulated
 * @keycolor    - Determines if the source data is covered (black key) or
 *                plaintext.
 * Note: Start and end of the key_mod, src and dst buffers have to be aligned to
 * the cache line size (ARCH_DMA_MINALIGN) for the ELE operation to succeed.
 *
 * Returns zero on success, negative on error.
 */
int blob_encap(u8 *key_mod, u8 *src, u8 *dst, u32 len, u8 keycolor)
{
	u32 output_size = 0, input_size = 0;
	u32 src_addr = (u32)(uintptr_t)src, dst_addr = (u32)(uintptr_t)dst;
	int wrap_blob = 1;

	/* check address input */
	if (!(src && dst)) {
		debug("src_addr or dst_addr invalid\n");
		return -1;
	}

	/* input will be the data */
	input_size = len;

	/* output will be the encrypted data blob with 32 bytes key blob
	 * in beginning and 16 byte HMAC identifier at end
	 */
	output_size = len + KEY_BLOB_SIZE + MAC_SIZE;

	/* check if the address can be accessed by EdgeLock Secure Enclave */
	if (src_addr > ELE_MAX_ADDR - input_size || dst_addr > ELE_MAX_ADDR - output_size) {
		printf("Error: %s: Address is not in valid range(< 0xE0000000)!\n", __func__);
		return -EINVAL;
	}

	/* check the address alignment */
	if (!IS_ALIGNED((uintptr_t)key_mod, ARCH_DMA_MINALIGN) ||
	    !IS_ALIGNED((uintptr_t)src, ARCH_DMA_MINALIGN) ||
	    !IS_ALIGNED((uintptr_t)dst, ARCH_DMA_MINALIGN)) {
		printf("Error: %s: Address arguments are not aligned!\n", __func__);
		return -EINVAL;
	}

	debug("\nEncapsulating data to form blob\n");

	/* Flush the cache */
	flush_dcache_range(src_addr, src_addr + input_size);

	/* Flush the cache */
	flush_dcache_range(dst_addr, (u32)(dst_addr +
			   roundup(output_size, ARCH_DMA_MINALIGN)));

	/* Call ELE */
	if (ele_blob(0x00, src_addr, input_size, dst_addr,
		     output_size, wrap_blob))
		return -1;

	/* Invalidate output buffer */
	invalidate_dcache_range(dst_addr, (u32)(dst_addr +
				roundup(output_size, ARCH_DMA_MINALIGN)));
	return 0;
}

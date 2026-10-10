/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#ifndef __LINUX_SOC_QCOM_QPACE_H
#define __LINUX_SOC_QCOM_QPACE_H

#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/errno.h>
#include <linux/kconfig.h>
#include <linux/types.h>

/**
 * struct qpace_algorithm - Descriptor for a QPaCE compression algorithm.
 * @name:            Algorithm name string (e.g. "qpace-lz4").
 * @comp_opcode:     Hardware opcode for compression.
 * @decomp_opcode:   Hardware opcode for decompression.
 * @unused:          Reserved bits.
 * @urg_comp_cntxt:  Urgent-path context index for compression.
 * @urg_decomp_cntxt: Urgent-path context index for decompression.
 */
struct qpace_algorithm {
	char *name;
	u32 comp_opcode:4;
	u32 decomp_opcode:4;
	u32 unused:24;
	int urg_comp_cntxt;
	int urg_decomp_cntxt;
};

#if IS_ENABLED(CONFIG_QCOM_PAGE_COMPRESSION_ENGINE)

/**
 * qpace_is_valid_algorithm() - Check if a name identifies a supported QPaCE algorithm.
 * @algo_name: Algorithm name string to check.
 *
 * Returns true if @algo_name matches a QPaCE algorithm and the driver has
 * probed successfully, false otherwise.
 */
bool qpace_is_valid_algorithm(const char *algo_name);

/**
 * qpace_lz4_algorithm - Algorithm descriptor for QPaCE LZ4.
 *
 * Pass to qpace_urgent_compress() and qpace_urgent_decompress() to select
 * the LZ4 algorithm.
 */
extern const struct qpace_algorithm qpace_lz4_algorithm;

/**
 * qpace_is_dev_available() - Check whether the QPaCE hardware is ready.
 *
 * Returns true if the driver has probed successfully and the hardware is
 * available for use, false otherwise.
 */
bool qpace_is_dev_available(void);

/**
 * qpace_urgent_compress() - Compress a page synchronously via the urgent path.
 * @input_addr:  DMA address of the page to compress.
 * @output_addr: DMA address to write the compressed output to.
 * @algo:        Algorithm descriptor to use.
 *
 * Blocks the calling CPU until compression completes. Synchronously polls
 * completion status and uses spinlocks; safe to call from atomic context.
 *
 * Return: Compressed size in bytes on success, negative error code on failure.
 */
int qpace_urgent_compress(dma_addr_t input_addr,
			  dma_addr_t output_addr,
			  struct qpace_algorithm *algo);

/**
 * qpace_urgent_decompress() - Decompress a page synchronously via the urgent path.
 * @input_addr:  DMA address of the compressed input.
 * @output_addr: DMA address to write the decompressed output to.
 * @input_size:  Size of the compressed input in bytes.
 * @algo:        Algorithm descriptor to use.
 *
 * Blocks the calling CPU until decompression completes. Synchronously polls
 * completion status and uses spinlocks; safe to call from atomic context.
 *
 * Return: Decompressed size in bytes on success, negative error code on failure.
 */
int qpace_urgent_decompress(dma_addr_t input_addr,
			    dma_addr_t output_addr,
			    size_t input_size,
			    struct qpace_algorithm *algo);

/**
 * qpace_get() - Acquire a reference to QPaCE, preventing runtime power collapse.
 *
 * Must be paired with qpace_put() on success. Callers that hold a reference
 * will keep the hardware awake and the urgent command contexts programmed.
 *
 * Return: 0 on success, -EBUSY if the device is suspended.
 */
int qpace_get(void);

/**
 * qpace_put() - Release a reference acquired with qpace_get().
 *
 * When the last reference is dropped, QPaCE may enter a low-power state.
 */
void qpace_put(void);

/**
 * qpace_get_dma_dev() - Return the device to use for QPaCE DMA mappings.
 *
 * Return: Pointer to the QPaCE platform device, or NULL if unavailable.
 */
struct device *qpace_get_dma_dev(void);

#else /* !CONFIG_QCOM_PAGE_COMPRESSION_ENGINE */

static inline bool qpace_is_valid_algorithm(const char *algo_name)
{
	return false;
}

static const struct qpace_algorithm qpace_lz4_algorithm;

static inline bool qpace_is_dev_available(void)
{
	return false;
}

static inline int qpace_urgent_compress(dma_addr_t input_addr,
					dma_addr_t output_addr,
					struct qpace_algorithm *algo)
{
	return -EINVAL;
}

static inline int qpace_urgent_decompress(dma_addr_t input_addr,
					  dma_addr_t output_addr,
					  size_t input_size,
					  struct qpace_algorithm *algo)
{
	return -EINVAL;
}

static inline int qpace_get(void) { return -ENODEV; }
static inline void qpace_put(void) {}

static inline struct device *qpace_get_dma_dev(void)
{
	return NULL;
}

#endif /* CONFIG_QCOM_PAGE_COMPRESSION_ENGINE */
#endif /* __LINUX_SOC_QCOM_QPACE_H */

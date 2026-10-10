/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2014 Sergey Senozhatsky.
 */

#ifndef _ZCOMP_H_
#define _ZCOMP_H_
#include <linux/local_lock.h>

struct zcomp_params {
	void *drv_data;
};

struct zcomp_ctx {
	void *context;
};

struct zcomp_req {
	const void *src;
	size_t src_len;
	void *dst;
	size_t dst_len;
};

struct zcomp_ops {
	const char *name;
	int (*setup_params)(struct zcomp_params *params);
	void (*release_params)(struct zcomp_params *params);
	int (*create_ctx)(struct zcomp_params *params, struct zcomp_ctx *ctx);
	void (*destroy_ctx)(struct zcomp_ctx *ctx);
	int (*compress)(struct zcomp_params *params, struct zcomp_ctx *ctx,
			struct zcomp_req *req);
	int (*decompress)(struct zcomp_params *params, struct zcomp_ctx *ctx,
			  struct zcomp_req *req);
};

struct zcomp_strm {
	/* The members ->buffer and ->tfm are protected by ->lock. */
	local_lock_t lock;
	/* compression/decompression buffer */
	void *buffer;
	struct crypto_comp *tfm;
#if IS_ENABLED(CONFIG_ZRAM_BACKEND_QPACE)
	void *qctx;
#endif
};

/* dynamic per-device compression frontend */
struct zcomp {
	struct zcomp_strm __percpu *stream;
	const char *name;
	struct hlist_node node;
#if IS_ENABLED(CONFIG_ZRAM_BACKEND_QPACE)
	const struct zcomp_ops *ops;
#endif
};

int zcomp_cpu_up_prepare(unsigned int cpu, struct hlist_node *node);
int zcomp_cpu_dead(unsigned int cpu, struct hlist_node *node);
ssize_t zcomp_available_show(const char *comp, char *buf);
bool zcomp_available_algorithm(const char *comp);

struct zcomp *zcomp_create(const char *comp);
void zcomp_destroy(struct zcomp *comp);

struct zcomp_strm *zcomp_stream_get(struct zcomp *comp);
void zcomp_stream_put(struct zcomp *comp);

int zcomp_compress(struct zcomp_strm *zstrm,
		const void *src, unsigned int *dst_len);

int zcomp_decompress(struct zcomp_strm *zstrm,
		const void *src, unsigned int src_len, void *dst);

bool zcomp_set_max_streams(struct zcomp *comp, int num_strm);
#endif /* _ZCOMP_H_ */

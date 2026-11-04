// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020-2024 Oplus. All rights reserved.
 * Adapted for Linux 5.10 GKI by RapliVx <raffikun1203@gmail.com>
 *
 * OPlus Fast Zstandard-N (zstdn / zstdn_o) compression algorithms.
 *
 * Specifically engineered for mobile ZRAM page swapping:
 *  - zstdn: Fast Zstandard negative level (-1) with single-page
 *    window bounds (windowLog=12) to minimize L1/L2 cache pollution.
 *  - zstdn_o: OPlus-optimized variant with aggressive match-step
 *    acceleration (-3), ultra-compact hash tables (chainLog=12,
 *    hashLog=12) for minimal swap latency.
 */

#define pr_fmt(fmt) "[OPLUS_ZSTDN]: " fmt

#include <linux/crypto.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/vmalloc.h>
#include <linux/zstd.h>
#include <crypto/internal/scompress.h>

static int __read_mostly zstdn_compression_level = -1;
static int __read_mostly zstdn_o_compression_level = -3;

static int set_zstdn_compression_level(const char *buf, const struct kernel_param *kp)
{
	int ret;
	int temp;

	ret = kstrtoint(buf, 0, &temp);
	if (ret)
		return ret;

	if (temp < zstd_min_clevel() || temp > zstd_max_clevel())
		return -EINVAL;

	return param_set_int(buf, kp);
}

module_param_call(zstdn_compression_level, set_zstdn_compression_level, param_get_int,
		  &zstdn_compression_level, 0644);
MODULE_PARM_DESC(zstdn_compression_level, "Zstandard-N compression level (default: -1)");

module_param_call(zstdn_o_compression_level, set_zstdn_compression_level, param_get_int,
		  &zstdn_o_compression_level, 0644);
MODULE_PARM_DESC(zstdn_o_compression_level, "OPlus Zstandard-N-O compression level (default: -3)");

struct zstdn_ctx {
	zstd_cctx *cctx;
	zstd_dctx *dctx;
	void *cwksp;
	void *dwksp;
	bool is_oplus;
};

static zstd_parameters zstdn_params(void)
{
	zstd_parameters params = zstd_get_params(zstdn_compression_level, PAGE_SIZE);

	/* Clamped to single 4KB page window */
	if (params.cParams.windowLog > 12)
		params.cParams.windowLog = 12;

	if (params.cParams.chainLog > 14)
		params.cParams.chainLog = 14;
	if (params.cParams.hashLog > 14)
		params.cParams.hashLog = 14;

	return params;
}

static zstd_parameters zstdn_o_params(void)
{
	zstd_parameters params = zstd_get_params(zstdn_o_compression_level, PAGE_SIZE);

	/* Clamped to single 4KB page window */
	if (params.cParams.windowLog > 12)
		params.cParams.windowLog = 12;

	/* OPlus ultra-compact cache footprint: fits in 32KB L1/L2 cache */
	if (params.cParams.chainLog > 12)
		params.cParams.chainLog = 12;
	if (params.cParams.hashLog > 12)
		params.cParams.hashLog = 12;

	if (params.cParams.minMatch < 4)
		params.cParams.minMatch = 4;

	return params;
}

static int zstdn_comp_init(struct zstdn_ctx *ctx)
{
	const zstd_parameters params = ctx->is_oplus ? zstdn_o_params() : zstdn_params();
	const size_t wksp_size = zstd_cctx_workspace_bound(&params.cParams);

	ctx->cwksp = vzalloc(wksp_size);
	if (!ctx->cwksp)
		return -ENOMEM;

	ctx->cctx = zstd_init_cctx(ctx->cwksp, wksp_size);
	if (!ctx->cctx) {
		vfree(ctx->cwksp);
		ctx->cwksp = NULL;
		return -EINVAL;
	}

	return 0;
}

static int zstdn_decomp_init(struct zstdn_ctx *ctx)
{
	const size_t wksp_size = zstd_dctx_workspace_bound();

	ctx->dwksp = vzalloc(wksp_size);
	if (!ctx->dwksp)
		return -ENOMEM;

	ctx->dctx = zstd_init_dctx(ctx->dwksp, wksp_size);
	if (!ctx->dctx) {
		vfree(ctx->dwksp);
		ctx->dwksp = NULL;
		return -EINVAL;
	}

	return 0;
}

static void zstdn_comp_exit(struct zstdn_ctx *ctx)
{
	vfree(ctx->cwksp);
	ctx->cwksp = NULL;
	ctx->cctx = NULL;
}

static void zstdn_decomp_exit(struct zstdn_ctx *ctx)
{
	vfree(ctx->dwksp);
	ctx->dwksp = NULL;
	ctx->dctx = NULL;
}

static int __zstdn_init(struct zstdn_ctx *ctx)
{
	int ret;

	ret = zstdn_comp_init(ctx);
	if (ret)
		return ret;
	ret = zstdn_decomp_init(ctx);
	if (ret)
		zstdn_comp_exit(ctx);
	return ret;
}

static void *zstdn_alloc_ctx_common(bool is_oplus)
{
	int ret;
	struct zstdn_ctx *ctx;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return ERR_PTR(-ENOMEM);

	ctx->is_oplus = is_oplus;
	ret = __zstdn_init(ctx);
	if (ret) {
		kfree(ctx);
		return ERR_PTR(ret);
	}

	return ctx;
}

static void *zstdn_alloc_ctx(struct crypto_scomp *tfm)
{
	return zstdn_alloc_ctx_common(false);
}

static void *zstdn_o_alloc_ctx(struct crypto_scomp *tfm)
{
	return zstdn_alloc_ctx_common(true);
}

static int zstdn_init_tfm(struct crypto_tfm *tfm)
{
	struct zstdn_ctx *ctx = crypto_tfm_ctx(tfm);

	ctx->is_oplus = false;
	return __zstdn_init(ctx);
}

static int zstdn_o_init_tfm(struct crypto_tfm *tfm)
{
	struct zstdn_ctx *ctx = crypto_tfm_ctx(tfm);

	ctx->is_oplus = true;
	return __zstdn_init(ctx);
}

static void __zstdn_exit(void *ctx)
{
	zstdn_comp_exit(ctx);
	zstdn_decomp_exit(ctx);
}

static void zstdn_free_ctx(struct crypto_scomp *tfm, void *ctx)
{
	__zstdn_exit(ctx);
	kfree_sensitive(ctx);
}

static void zstdn_exit_tfm(struct crypto_tfm *tfm)
{
	struct zstdn_ctx *ctx = crypto_tfm_ctx(tfm);

	__zstdn_exit(ctx);
}

static int __zstdn_compress(const u8 *src, unsigned int slen,
			    u8 *dst, unsigned int *dlen, void *ctx)
{
	size_t out_len;
	struct zstdn_ctx *zctx = ctx;
	const zstd_parameters params = zctx->is_oplus ? zstdn_o_params() : zstdn_params();

	out_len = zstd_compress_cctx(zctx->cctx, dst, *dlen, src, slen, &params);
	if (zstd_is_error(out_len))
		return -EINVAL;
	*dlen = out_len;
	return 0;
}

static int zstdn_compress_tfm(struct crypto_tfm *tfm, const u8 *src,
			      unsigned int slen, u8 *dst, unsigned int *dlen)
{
	struct zstdn_ctx *ctx = crypto_tfm_ctx(tfm);

	return __zstdn_compress(src, slen, dst, dlen, ctx);
}

static int zstdn_scompress(struct crypto_scomp *tfm, const u8 *src,
			   unsigned int slen, u8 *dst, unsigned int *dlen,
			   void *ctx)
{
	return __zstdn_compress(src, slen, dst, dlen, ctx);
}

static int __zstdn_decompress(const u8 *src, unsigned int slen,
			      u8 *dst, unsigned int *dlen, void *ctx)
{
	size_t out_len;
	struct zstdn_ctx *zctx = ctx;

	out_len = zstd_decompress_dctx(zctx->dctx, dst, *dlen, src, slen);
	if (zstd_is_error(out_len))
		return -EINVAL;
	*dlen = out_len;
	return 0;
}

static int zstdn_decompress_tfm(struct crypto_tfm *tfm, const u8 *src,
				unsigned int slen, u8 *dst, unsigned int *dlen)
{
	struct zstdn_ctx *ctx = crypto_tfm_ctx(tfm);

	return __zstdn_decompress(src, slen, dst, dlen, ctx);
}

static int zstdn_sdecompress(struct crypto_scomp *tfm, const u8 *src,
			     unsigned int slen, u8 *dst, unsigned int *dlen,
			     void *ctx)
{
	return __zstdn_decompress(src, slen, dst, dlen, ctx);
}

static struct crypto_alg zstdn_algs[] = {
	{
		.cra_name		= "zstdn",
		.cra_driver_name	= "zstdn-generic",
		.cra_flags		= CRYPTO_ALG_TYPE_COMPRESS,
		.cra_ctxsize		= sizeof(struct zstdn_ctx),
		.cra_priority		= 350,
		.cra_module		= THIS_MODULE,
		.cra_init		= zstdn_init_tfm,
		.cra_exit		= zstdn_exit_tfm,
		.cra_u			= { .compress = {
			.coa_compress	= zstdn_compress_tfm,
			.coa_decompress	= zstdn_decompress_tfm } }
	},
	{
		.cra_name		= "zstdn_o",
		.cra_driver_name	= "zstdn_o-generic",
		.cra_flags		= CRYPTO_ALG_TYPE_COMPRESS,
		.cra_ctxsize		= sizeof(struct zstdn_ctx),
		.cra_priority		= 360,
		.cra_module		= THIS_MODULE,
		.cra_init		= zstdn_o_init_tfm,
		.cra_exit		= zstdn_exit_tfm,
		.cra_u			= { .compress = {
			.coa_compress	= zstdn_compress_tfm,
			.coa_decompress	= zstdn_decompress_tfm } }
	}
};

static struct scomp_alg zstdn_scomps[] = {
	{
		.alloc_ctx		= zstdn_alloc_ctx,
		.free_ctx		= zstdn_free_ctx,
		.compress		= zstdn_scompress,
		.decompress		= zstdn_sdecompress,
		.base			= {
			.cra_name	= "zstdn",
			.cra_driver_name = "zstdn-scomp",
			.cra_priority	 = 350,
			.cra_module	 = THIS_MODULE,
		}
	},
	{
		.alloc_ctx		= zstdn_o_alloc_ctx,
		.free_ctx		= zstdn_free_ctx,
		.compress		= zstdn_scompress,
		.decompress		= zstdn_sdecompress,
		.base			= {
			.cra_name	= "zstdn_o",
			.cra_driver_name = "zstdn_o-scomp",
			.cra_priority	 = 360,
			.cra_module	 = THIS_MODULE,
		}
	}
};

static int __init zstdn_mod_init(void)
{
	int ret;

	ret = crypto_register_algs(zstdn_algs, ARRAY_SIZE(zstdn_algs));
	if (ret) {
		pr_err("failed to register crypto algs: %d\n", ret);
		return ret;
	}

	ret = crypto_register_scomps(zstdn_scomps, ARRAY_SIZE(zstdn_scomps));
	if (ret) {
		pr_err("failed to register scomps: %d\n", ret);
		crypto_unregister_algs(zstdn_algs, ARRAY_SIZE(zstdn_algs));
		return ret;
	}

	pr_info("registered native zstdn (clevel %d, prio 350) and zstdn_o (clevel %d, prio 360)\n",
		zstdn_compression_level, zstdn_o_compression_level);
	return 0;
}

static void __exit zstdn_mod_fini(void)
{
	crypto_unregister_algs(zstdn_algs, ARRAY_SIZE(zstdn_algs));
	crypto_unregister_scomps(zstdn_scomps, ARRAY_SIZE(zstdn_scomps));
	pr_info("unregistered zstdn and zstdn_o algorithms\n");
}

subsys_initcall(zstdn_mod_init);
module_exit(zstdn_mod_fini);

MODULE_AUTHOR("RapliVx <raffikun1203@gmail.com>");
MODULE_AUTHOR("Oplus Kernel Team");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("OPlus Zstandard Fast Compression Algorithm (zstdn / zstdn_o)");
MODULE_ALIAS_CRYPTO("zstdn");
MODULE_ALIAS_CRYPTO("zstdn_o");


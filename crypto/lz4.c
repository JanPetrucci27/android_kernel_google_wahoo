/*
 * Cryptographic API.
 *
 * Copyright (c) 2013 Chanho Min <chanho.min@lge.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc., 51
 * Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/crypto.h>
#include <linux/vmalloc.h>
#include <linux/lz4.h>

struct lz4_ctx {
	void *lz4_comp_mem;
};

static int lz4_init(struct crypto_tfm *tfm)
{
	struct lz4_ctx *ctx = crypto_tfm_ctx(tfm);

	ctx->lz4_comp_mem = vmalloc(LZ4_MEM_COMPRESS);
	if (!ctx->lz4_comp_mem)
		return -ENOMEM;

	return 0;
}

static void lz4_exit(struct crypto_tfm *tfm)
{
	struct lz4_ctx *ctx = crypto_tfm_ctx(tfm);
	vfree(ctx->lz4_comp_mem);
}

static int lz4_compress_crypto(struct crypto_tfm *tfm, const u8 *src,
			    unsigned int slen, u8 *dst, unsigned int *dlen)
{
	struct lz4_ctx *ctx = crypto_tfm_ctx(tfm);
	int out_len = LZ4_compress_default(src, dst,
		slen, *dlen, ctx->lz4_comp_mem);

	if (!out_len)
		return -EINVAL;

	*dlen = out_len;
	return 0;
}

static int lz4_decompress_crypto(struct crypto_tfm *tfm, const u8 *src,
			      unsigned int slen, u8 *dst, unsigned int *dlen)
{
#if defined(CONFIG_ARM64) && defined(CONFIG_KERNEL_MODE_NEON)
	int out_len = LZ4_arm64_decompress_safe(src, dst, slen, *dlen, false);
#else
	int out_len = LZ4_decompress_safe(src, dst, slen, *dlen);
#endif

	if (out_len < 0)
		return out_len;

	*dlen = out_len;
	return 0;
}

static struct crypto_alg alg_lz4 = {
	.cra_name		= "lz4",
	.cra_flags		= CRYPTO_ALG_TYPE_COMPRESS,
	.cra_ctxsize		= sizeof(struct lz4_ctx),
	.cra_module		= THIS_MODULE,
	.cra_list		= LIST_HEAD_INIT(alg_lz4.cra_list),
	.cra_init		= lz4_init,
	.cra_exit		= lz4_exit,
	.cra_u			= { .compress = {
	.coa_compress		= lz4_compress_crypto,
	.coa_decompress		= lz4_decompress_crypto } }
};

static int __init lz4_mod_init(void)
{
	return crypto_register_alg(&alg_lz4);
}

static void __exit lz4_mod_fini(void)
{
	crypto_unregister_alg(&alg_lz4);
}

module_init(lz4_mod_init);
module_exit(lz4_mod_fini);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("LZ4 Compression Algorithm");
MODULE_ALIAS_CRYPTO("lz4");

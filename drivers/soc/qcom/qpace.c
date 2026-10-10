// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#include <dt-bindings/interconnect/qcom,icc.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/export.h>
#include <linux/interconnect.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_qos.h>
#include <linux/pm_wakeup.h>
#include <linux/soc/qcom/llcc-qcom.h>
#include <linux/soc/qcom/qpace.h>
#include <linux/workqueue.h>

#include "qpace_internal.h"

#define CREATE_TRACE_POINTS
#include <trace/events/qpace.h>

#define QPACE_REG_PAGE_SIZE 4096
#define QPACE_CORE_CACHEINDEX_SIZE 4

struct qpace_priv {
	void __iomem *gen_regs;
	void __iomem *gen_core_regs;
	void __iomem *comp_core_regs;
	void __iomem *decomp_core_regs;
	void __iomem *urg_regs;

	struct device *dev;
	u32 hw_version;
	bool suspended;

	struct icc_path *interconnect;
	struct pm_qos_request qos_req;
	struct llcc_slice_desc *llc_comp;
	struct llcc_slice_desc *llc_decomp;

	int active_rings;
	struct completion no_active_refs;

	bool broken;
	struct work_struct disable_work;
};

static inline u32 qpace_read_gen(struct qpace_priv *p, u32 offset)
{
	return readl(p->gen_regs + offset);
}

static inline void qpace_write_gen(struct qpace_priv *p, u32 offset, u32 val)
{
	writel(val, p->gen_regs + offset);
}

static inline u32 qpace_read_gen_core(struct qpace_priv *p, u32 offset)
{
	return readl(p->gen_core_regs + offset);
}

static inline void qpace_write_gen_core(struct qpace_priv *p, u32 offset, u32 val)
{
	writel(val, p->gen_core_regs + offset);
}

static inline u32 qpace_read_comp_core(struct qpace_priv *p, u32 offset)
{
	return readl(p->comp_core_regs + offset);
}

static inline void qpace_write_comp_core(struct qpace_priv *p, u32 offset, u32 val)
{
	writel(val, p->comp_core_regs + offset);
}

static inline u32 qpace_read_urg_cacheindex(struct qpace_priv *p, int idx, u32 offset)
{
	return readl(p->gen_regs + idx * QPACE_CORE_CACHEINDEX_SIZE + offset);
}

static inline void qpace_write_urg_cacheindex(struct qpace_priv *p, int idx, u32 offset, u32 val)
{
	writel(val, p->gen_regs + idx * QPACE_CORE_CACHEINDEX_SIZE + offset);
}

static inline u32 qpace_read_decomp_core(struct qpace_priv *p, u32 offset)
{
	return readl(p->decomp_core_regs + offset);
}

static inline void qpace_write_decomp_core(struct qpace_priv *p, u32 offset, u32 val)
{
	writel(val, p->decomp_core_regs + offset);
}

static inline u32 qpace_read_urg_cmd(struct qpace_priv *p, int idx, u32 offset)
{
	return readl(p->urg_regs + idx * QPACE_REG_PAGE_SIZE + offset);
}

static inline void qpace_write_urg_cmd(struct qpace_priv *p, int idx, u32 offset, u32 val)
{
	writel(val, p->urg_regs + idx * QPACE_REG_PAGE_SIZE + offset);
}

static inline void qpace_write_urg_cmd_ctx(struct qpace_priv *p, u32 offset,
					   int urg_reg_num, int ctx_num, u32 val)
{
	writel(val, p->urg_regs + urg_reg_num * QPACE_REG_PAGE_SIZE +
	       offset + ctx_num * URG_CMD_CONTEXT_SPACING);
}

static struct qpace_priv *qpace_priv;

static DEFINE_STATIC_KEY_FALSE(qpace_drv_probed);

static void qpace_disable_work_fn(struct work_struct *work)
{
	static_branch_disable(&qpace_drv_probed);
}

enum urg_reg_cxts {
	LZ4_URG_COMP_CNTXT,
	LZ4_URG_DECOMP_CNTXT,
};

const struct qpace_algorithm qpace_lz4_algorithm = {
	.name = "qpace-lz4",
	.comp_opcode = LZ4_COMP,
	.decomp_opcode = LZ4_DECOMP,
	.urg_comp_cntxt = LZ4_URG_COMP_CNTXT,
	.urg_decomp_cntxt = LZ4_URG_DECOMP_CNTXT,
};
EXPORT_SYMBOL_GPL(qpace_lz4_algorithm);

bool qpace_is_valid_algorithm(const char *algo_name)
{
	return algo_name && !strcmp(algo_name, qpace_lz4_algorithm.name);
}
EXPORT_SYMBOL_GPL(qpace_is_valid_algorithm);

static inline void program_urg_comp_context(int urg_reg_num, const struct qpace_algorithm *algo)
{
	u32 urg_cmd_settings = 0;
	int page_cnt = (1 << (PAGE_SHIFT - 12)) - 1;

	/* Set the limit of the compression output size */
	urg_cmd_settings = FIELD_PREP(URG_CMD_0_CFG_CNTXT_SIZE_SIZE, PAGE_SIZE - 1);
	qpace_write_urg_cmd_ctx(qpace_priv, QPACE_URG_CMD_0_CFG_CNTXT_SIZE_n_OFFSET,
				urg_reg_num, algo->urg_comp_cntxt,
				urg_cmd_settings);

	urg_cmd_settings = FIELD_PREP(URG_CMD_0_CFG_CNTXT_MISC_OPER, algo->comp_opcode);

	/* Set page count for urgent compression input */
	urg_cmd_settings |= FIELD_PREP(URG_CMD_0_CFG_CNTXT_MISC_PAGE_CNT, page_cnt);

	/* Program part of the SMMU input and output SIDs */
	urg_cmd_settings |= FIELD_PREP(URG_CMD_0_CFG_CNTXT_MISC_SCTX,
				       DEFAULT_SMMU_CONTEXT);
	urg_cmd_settings |= FIELD_PREP(URG_CMD_0_CFG_CNTXT_MISC_DCTX,
				       DEFAULT_SMMU_CONTEXT);

	/* Cache accesses in the system cache during compression */
	urg_cmd_settings |= FIELD_PREP(URG_CMD_0_CFG_CNTXT_MISC_WA, 1);

	qpace_write_urg_cmd_ctx(qpace_priv, QPACE_URG_CMD_0_CFG_CNTXT_MISC_n_OFFSET,
				urg_reg_num, algo->urg_comp_cntxt,
				urg_cmd_settings);
}

static inline void program_urg_decomp_context(int urg_reg_num, const struct qpace_algorithm *algo)
{
	u32 urg_cmd_settings = 0;
	int page_cnt = (1 << (PAGE_SHIFT - 12)) - 1;

	/* The input size will be programmed by a requester */
	urg_cmd_settings = FIELD_PREP(URG_CMD_0_CFG_CNTXT_SIZE_SIZE, 0);
	qpace_write_urg_cmd_ctx(qpace_priv, QPACE_URG_CMD_0_CFG_CNTXT_SIZE_n_OFFSET,
				urg_reg_num, algo->urg_decomp_cntxt,
				urg_cmd_settings);

	urg_cmd_settings = FIELD_PREP(URG_CMD_0_CFG_CNTXT_MISC_OPER, algo->decomp_opcode);

	/* Configure page count which will limit decompression output size */
	urg_cmd_settings |= FIELD_PREP(URG_CMD_0_CFG_CNTXT_MISC_PAGE_CNT, page_cnt);

	/* Program part of the SMMU input and output SIDs */
	urg_cmd_settings |= FIELD_PREP(URG_CMD_0_CFG_CNTXT_MISC_SCTX,
				       DEFAULT_SMMU_CONTEXT);
	urg_cmd_settings |= FIELD_PREP(URG_CMD_0_CFG_CNTXT_MISC_DCTX,
				       DEFAULT_SMMU_CONTEXT);

	/* Cache accesses in the system cache during decompression */
	urg_cmd_settings |= FIELD_PREP(URG_CMD_0_CFG_CNTXT_MISC_WA, 1);

	qpace_write_urg_cmd_ctx(qpace_priv, QPACE_URG_CMD_0_CFG_CNTXT_MISC_n_OFFSET,
				urg_reg_num, algo->urg_decomp_cntxt,
				urg_cmd_settings);
}

static void program_urg_cmd_cacheindex(int urg_reg_num)
{
	u32 reg_val;

	reg_val = qpace_read_urg_cacheindex(qpace_priv, urg_reg_num,
					    QPACE_CORE_URG_CMD_n_CACHEINDEX_CFG_OFFSET);
	reg_val = u32_replace_bits(reg_val, 0x24, CORE_URG_CMD_n_CACHEINDEX__ENGINE_CACHEINDEX);
	reg_val = u32_replace_bits(reg_val, 0x1,
				   CORE_URG_CMD_n_CACHEINDEX__ENGINE_CACHEINDEX_OVERRIDE);
	qpace_write_urg_cacheindex(qpace_priv, urg_reg_num,
				   QPACE_CORE_URG_CMD_n_CACHEINDEX_CFG_OFFSET, reg_val);
}

static void program_decomp_core_cfg(void)
{
	u32 reg_val;

	reg_val = qpace_read_decomp_core(qpace_priv, QPACE_DECOMP_CORE_CFG_OFFSET);
	reg_val = u32_replace_bits(reg_val, 0x8, DECOMP_CORE_CFG_DMA_RD_MAX_OT);
	reg_val = u32_replace_bits(reg_val, 0x16, DECOMP_CORE_CFG_DMA_WR_MAX_OT);
	/*
	 * MEM_MNG_PAGE_SIZE controls the prefetch boundary:
	 * 0: 4 KB, 1: 8 KB, 2: 16 KB, 3: 32 KB, 4: 64 KB
	 */
	reg_val = u32_replace_bits(reg_val, 0x0, DECOMP_CORE_CFG_DECOMP_DMA_MEM_MNG_PAGE_SIZE);
	qpace_write_decomp_core(qpace_priv, QPACE_DECOMP_CORE_CFG_OFFSET, reg_val);
}

static void program_urg_command_contexts_v2(void)
{
	int urg_reg_num;

	/* Configure the needed contexts for each of the urgent command registers */
	for (urg_reg_num = 0; urg_reg_num < NUM_TRS_ERS_URG_CMD_REGS; urg_reg_num++) {
		program_urg_cmd_cacheindex(urg_reg_num);
		program_urg_comp_context(urg_reg_num, &qpace_lz4_algorithm);
		program_urg_decomp_context(urg_reg_num, &qpace_lz4_algorithm);
	}
}

static inline int qpace_urgent_command_trigger(dma_addr_t input_addr,
					       dma_addr_t output_addr,
					       int urg_reg_num,
					       enum urg_reg_cxts command)
{
	void __iomem *td_dst_src_reg;
	u64 urg_addr_field_lower, urg_addr_field_upper;
	u32 stat_reg;
	unsigned long ret;

	if (!qpace_priv || !qpace_priv->urg_regs || READ_ONCE(qpace_priv->broken))
		return -ENODEV;

	td_dst_src_reg = qpace_priv->urg_regs +
			 (urg_reg_num * QPACE_REG_PAGE_SIZE) +
			 QPACE_URG_CMD_0_TD_DST_ADDR_L_CFG_CNTXT_OFFSET;

	urg_addr_field_lower = FIELD_PREP(URG_CMD_0_TD_DST_ADDR_L__CMD_CFG_CNTXT,
					  command);
	urg_addr_field_lower |= GENMASK(63, 8) & output_addr;

	urg_addr_field_upper = input_addr;

	/* Clear a stale request-on-active error before ringing the doorbell. */
	qpace_write_urg_cmd(qpace_priv, urg_reg_num, QPACE_URG_CMD_0_STAT_CLR_OFFSET,
			    URG_CMD_0_STAT_CLR_REQ_ON_ACTIVE_ERR);

	/* Ensure that preceding stores that QPaCE will depend on are done executing */
	dma_wmb();

	/*
	 * Ring the doorbell with a single 128-bit atomic store. QPaCE
	 * triggers processing when it observes the (dst_addr, src_addr)
	 * pair land together.
	 */
	asm volatile("stp %0, %1, [%2]\n" :
		     : "r" (urg_addr_field_lower), "r" (urg_addr_field_upper), "r" (td_dst_src_reg)
		     : "memory");

	ret = readl_poll_timeout_atomic(qpace_priv->urg_regs +
					urg_reg_num * QPACE_REG_PAGE_SIZE +
					QPACE_URG_CMD_0_ED_STAT_OFFSET,
					stat_reg,
					FIELD_GET(URG_CMD_0_ED_STAT_COMP_CODE,
						  stat_reg) != OP_URG_ONGOING,
					1, 10 * USEC_PER_MSEC);
	if (ret) {
		dev_err_ratelimited(qpace_priv->dev, "QPaCE urgent command timed out\n");
		return -ETIMEDOUT;
	}

	return stat_reg;
}

int qpace_urgent_compress(dma_addr_t input_addr,
			  dma_addr_t output_addr,
			  struct qpace_algorithm *algo)
{
	int urg_reg_num;
	int stat_reg;
	u32 stat_reg_val;
	int ret;

	if (!qpace_is_dev_available() || !algo)
		return -ENODEV;

	ret = qpace_get();
	if (ret)
		return ret;

	trace_start_qpace_urgent_compress((u64)input_addr, (u64)output_addr);

	urg_reg_num = get_cpu() % NUM_TRS_ERS_URG_CMD_REGS;
	stat_reg = qpace_urgent_command_trigger(input_addr, output_addr, urg_reg_num,
						algo->urg_comp_cntxt);
	put_cpu();

	qpace_put();

	if (stat_reg < 0) {
		ret = stat_reg;
		goto out;
	}

	stat_reg_val = FIELD_GET(URG_CMD_0_ED_STAT_COMP_CODE, stat_reg);

	if (stat_reg_val == OP_COMP_TOO_BIG) {
		ret = -E2BIG;
		goto out;
	}

	if (stat_reg_val != OP_OK) {
		pr_err_ratelimited("%s: register %d failed with %u\n",
				   __func__, urg_reg_num, stat_reg_val);
		ret = -EINVAL;
		goto out;
	}

	ret = FIELD_GET(URG_CMD_0_ED_STAT_SIZE, stat_reg);
out:
	trace_end_qpace_urgent_compress((u64)input_addr, (u64)output_addr, ret);
	return ret;
}
EXPORT_SYMBOL_GPL(qpace_urgent_compress);

int qpace_urgent_decompress(dma_addr_t input_addr,
			    dma_addr_t output_addr,
			    size_t input_size,
			    struct qpace_algorithm *algo)
{
	int urg_reg_num;
	int stat_reg;
	u32 stat_reg_val;
	int ret;

	if (!qpace_is_dev_available() || !algo)
		return -ENODEV;

	ret = qpace_get();
	if (ret)
		return ret;

	trace_start_qpace_urgent_decompress((u64)input_addr,
					    (u64)output_addr, input_size);

	urg_reg_num = get_cpu() % NUM_TRS_ERS_URG_CMD_REGS;
	qpace_write_urg_cmd_ctx(qpace_priv, QPACE_URG_CMD_0_CFG_CNTXT_SIZE_n_OFFSET,
				urg_reg_num, algo->urg_decomp_cntxt,
				FIELD_PREP(URG_CMD_0_CFG_CNTXT_SIZE_SIZE, input_size));
	stat_reg = qpace_urgent_command_trigger(input_addr, output_addr, urg_reg_num,
						algo->urg_decomp_cntxt);
	put_cpu();

	qpace_put();

	if (stat_reg < 0) {
		ret = stat_reg;
		goto out;
	}

	stat_reg_val = FIELD_GET(URG_CMD_0_ED_STAT_COMP_CODE, stat_reg);
	if (stat_reg_val != OP_OK) {
		pr_err_ratelimited("%s: register %d failed with %u\n",
				   __func__, urg_reg_num, stat_reg_val);
		ret = -EINVAL;
		goto out;
	}

	ret = FIELD_GET(URG_CMD_0_ED_STAT_SIZE, stat_reg);
out:
	trace_end_qpace_urgent_decompress((u64)input_addr,
					  (u64)output_addr,
					  input_size, ret);
	return ret;
}
EXPORT_SYMBOL_GPL(qpace_urgent_decompress);

static DEFINE_SPINLOCK(qpace_ref_lock);

static void _get_qpace(void)
{
	lockdep_assert_held(&qpace_ref_lock);
	if (!qpace_priv)
		return;
	if (!qpace_priv->active_rings) {
		reinit_completion(&qpace_priv->no_active_refs);
		pm_stay_awake(qpace_priv->dev);
		cpu_latency_qos_update_request(&qpace_priv->qos_req, 300);
		program_urg_command_contexts_v2();
		program_decomp_core_cfg();
	}
	qpace_priv->active_rings++;
}

static void _put_qpace(void)
{
	lockdep_assert_held(&qpace_ref_lock);
	if (!qpace_priv)
		return;
	if (!--qpace_priv->active_rings) {
		cpu_latency_qos_update_request(&qpace_priv->qos_req, PM_QOS_DEFAULT_VALUE);
		pm_relax(qpace_priv->dev);
		complete(&qpace_priv->no_active_refs);
	}
}

int qpace_get(void)
{
	unsigned long flags;
	int ret = 0;

	if (!qpace_is_dev_available())
		return -ENODEV;

	spin_lock_irqsave(&qpace_ref_lock, flags);
	if (!qpace_priv || qpace_priv->suspended || READ_ONCE(qpace_priv->broken))
		ret = -EBUSY;
	else
		_get_qpace();
	spin_unlock_irqrestore(&qpace_ref_lock, flags);
	return ret;
}
EXPORT_SYMBOL_GPL(qpace_get);

void qpace_put(void)
{
	unsigned long flags;

	spin_lock_irqsave(&qpace_ref_lock, flags);
	if (qpace_priv && qpace_priv->active_rings > 0)
		_put_qpace();
	spin_unlock_irqrestore(&qpace_ref_lock, flags);
}
EXPORT_SYMBOL_GPL(qpace_put);

static irqreturn_t urgent_interrupt_handler(int irq, void *unused)
{
	pr_debug("Urgent interrupt handled\n");
	return IRQ_HANDLED;
}

static int qpace_hw_init(void)
{
	u32 reg_val;

	/* Select CPU SCID for our system cache slice. */
	reg_val = qpace_read_gen(qpace_priv, QPACE_CORE_QNS4_CFG_OFFSET);
	reg_val = u32_replace_bits(reg_val, 0x1, CORE_QNS4_CFG_CACHEINDEX);
	qpace_write_gen(qpace_priv, QPACE_CORE_QNS4_CFG_OFFSET, reg_val);

	/* QMB2 register configurations. */
	reg_val = qpace_read_gen(qpace_priv, QPACE_CORE_GEN_CFG_OFFSET);
	reg_val = u32_replace_bits(reg_val, 0x48, CORE_GEN_CFG_QMB2_MAX_RD_OUTST_LIMIT);
	reg_val = u32_replace_bits(reg_val, 0x48, CORE_GEN_CFG_QMB2_MAX_WR_OUTST_LIMIT);
	qpace_write_gen(qpace_priv, QPACE_CORE_GEN_CFG_OFFSET, reg_val);

	/* DECOMP_CORE_CFG init steps. */
	program_decomp_core_cfg();

	/* Below settings help save power since all decomp cores are set to sync. */
	reg_val = qpace_read_gen_core(qpace_priv, QPACE_CORE_OPER_CFG_OFFSET);
	reg_val |= CORE_OPER_CFG_COMP_MEM_PWR_DWN_1;
	qpace_write_gen_core(qpace_priv, QPACE_CORE_OPER_CFG_OFFSET, reg_val);

	reg_val = qpace_read_comp_core(qpace_priv, QPACE_COMP_CORE_CFG_OFFSET);
	reg_val = u32_replace_bits(reg_val, 0x8, COMP_CORE_CFG_DMA_RD_MAX_OT);
	reg_val = u32_replace_bits(reg_val, 0x8, COMP_CORE_CFG_DMA_WR_MAX_OT);
	qpace_write_comp_core(qpace_priv, QPACE_COMP_CORE_CFG_OFFSET, reg_val);

	/* Set all COMP engines to bulk mode. */
	reg_val = qpace_read_comp_core(qpace_priv, QPACE_COMP_CORE_BULK_MODE_OFFSET);
	reg_val |= COMP_CORE_BULK_MODE_ALL_CORES;
	qpace_write_comp_core(qpace_priv, QPACE_COMP_CORE_BULK_MODE_OFFSET, reg_val);

	/* URG CMD register configurations. */
	program_urg_command_contexts_v2();

	return 0;
}

enum qpace_interrupts {
	QPACE_IRQ_URGENT
};

static int qpace_register_interrupts(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int irq, ret;

	/*
	 * The urgent command path is purely poll based; the interrupt is
	 * optional and only used to note completion.
	 */
	irq = platform_get_irq(pdev, QPACE_IRQ_URGENT);
	if (irq < 0) {
		dev_info(dev, "no urgent interrupt, relying on polling\n");
		return 0;
	}

	ret = devm_request_irq(dev, irq, urgent_interrupt_handler,
			       0, "qpace-urgent-irq", NULL);
	if (ret)
		dev_err(dev, "failed to request urgent interrupt\n");

	return ret;
}

static inline bool _qpace_power_on(void)
{
	u32 ready_status;

	qpace_write_gen_core(qpace_priv, QPACE_CORE_OPER_CORE_RUN_STOP_OFFSET, QPACE_RUN);

	if (readl_poll_timeout(qpace_priv->gen_core_regs +
			       QPACE_CORE_OPER_CORE_READY_OFFSET,
			       ready_status, ready_status,
			       1000, 5 * QPACE_STATE_CHANGE_TIMEOUT_US)) {
		pr_err("Timeout in waiting for QPaCE to turn on\n");
		return false;
	}

	return true;
}

static int qpace_power_on(struct device *dev)
{
	int ret, ret2;

	if (!qpace_priv || !qpace_priv->interconnect)
		return -ENODEV;

	ret = device_init_wakeup(dev, true);
	if (ret) {
		pr_err("%s: device_init_wakeup() failed with %d\n", __func__, ret);
		return ret;
	}

	cpu_latency_qos_add_request(&qpace_priv->qos_req, PM_QOS_DEFAULT_VALUE);

	icc_set_tag(qpace_priv->interconnect, QCOM_ICC_TAG_ACTIVE_ONLY);

	ret = icc_set_bw(qpace_priv->interconnect, 0, 1);
	if (ret) {
		pr_err("Failed to turn on QPaCE VCD: %d\n", ret);
		goto rm_qos;
	}

	if (!_qpace_power_on()) {
		pr_err("Failed to start QPaCE\n");
		ret = -EINVAL;
		goto rm_bw;
	}

	return 0;

rm_bw:
	ret2 = icc_set_bw(qpace_priv->interconnect, 0, 0);
	if (ret2)
		pr_err("Failed to remove QPaCE VCD vote: %d\n", ret2);
rm_qos:
	cpu_latency_qos_remove_request(&qpace_priv->qos_req);
	device_init_wakeup(dev, false);

	return ret;
}

static inline bool _qpace_power_off(void)
{
	u32 ready_status;

	if (!qpace_priv)
		return false;

	qpace_write_gen_core(qpace_priv, QPACE_CORE_OPER_CORE_RUN_STOP_OFFSET, QPACE_STOP);

	if (readl_poll_timeout(qpace_priv->gen_core_regs +
			       QPACE_CORE_OPER_CORE_READY_OFFSET,
			       ready_status, !ready_status,
			       1000, 5 * QPACE_STATE_CHANGE_TIMEOUT_US)) {
		pr_err("Timeout in waiting for QPaCE to turn off\n");
		return false;
	}

	return true;
}

static void qpace_power_off(struct device *dev)
{
	int ret;

	/* If this fails we can still remove our vote for the VCD to turn QPaCE off */
	if (!_qpace_power_off())
		pr_err("Failed to stop QPaCE\n");

	if (qpace_priv && qpace_priv->interconnect) {
		ret = icc_set_bw(qpace_priv->interconnect, 0, 0);
		if (ret)
			pr_err("Failed to turn off QPaCE VCD: %d\n", ret);
	}

	if (qpace_priv)
		cpu_latency_qos_remove_request(&qpace_priv->qos_req);

	device_init_wakeup(dev, false);
}

static inline int qpace_register_ioremap(struct platform_device *pdev)
{
	struct qpace_priv *priv = platform_get_drvdata(pdev);

	priv->gen_regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(priv->gen_regs))
		return PTR_ERR(priv->gen_regs);

	priv->gen_core_regs = priv->gen_regs + QPACE_GEN_CORE_REGS_OFFSET;
	priv->comp_core_regs = priv->gen_regs + QPACE_COMP_CORE_REGS_OFFSET;
	priv->decomp_core_regs = priv->gen_regs + QPACE_DECOMP_CORE_REGS_OFFSET;
	priv->urg_regs = priv->gen_regs + QPACE_URG_REGS_OFFSET;

	return 0;
}

bool qpace_is_dev_available(void)
{
	return static_branch_likely(&qpace_drv_probed) &&
	       qpace_priv && !READ_ONCE(qpace_priv->broken);
}
EXPORT_SYMBOL_GPL(qpace_is_dev_available);

struct device *qpace_get_dma_dev(void)
{
	return qpace_is_dev_available() ? qpace_priv->dev : NULL;
}
EXPORT_SYMBOL_GPL(qpace_get_dma_dev);

static int qpace_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct qpace_priv *priv;
	int ret;

	if (!dev->of_node || !of_device_is_available(dev->of_node))
		return -ENODEV;

	/*
	 * Validate required DT properties per qcom,hawi-qpace binding.
	 * QPaCE requires IOMMU mapping for DMA operations. If missing,
	 * do not attempt to probe hardware or touch MMIO registers.
	 */
	if (!of_find_property(dev->of_node, "iommus", NULL)) {
		dev_info(dev, "missing required iommus property, hardware not present\n");
		return -ENODEV;
	}

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = dev;
	/* Starts already complete since active_rings == 0 at init. */
	init_completion(&priv->no_active_refs);
	complete(&priv->no_active_refs);
	INIT_WORK(&priv->disable_work, qpace_disable_work_fn);
	platform_set_drvdata(pdev, priv);

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(64));
	if (ret)
		return dev_err_probe(dev, ret, "failed to set DMA mask\n");

	ret = qpace_register_ioremap(pdev);
	if (ret)
		return dev_err_probe(dev, ret, "failed to map QPaCE registers\n");

	/*
	 * Interconnect path is mandatory to clock and power the NoC bus to
	 * QPaCE. Accessing registers without an active interconnect vote
	 * causes bus faults (SError) on Qualcomm platforms.
	 */
	priv->interconnect = devm_of_icc_get(dev, "qpace-mem");
	if (IS_ERR(priv->interconnect)) {
		ret = PTR_ERR(priv->interconnect);
		if (ret != -EPROBE_DEFER)
			dev_info(dev, "no interconnect path (%d), hardware not present\n", ret);
		return ret;
	}

	/*
	 * LLCC cache slices are required by QPaCE hardware for system cache.
	 * On SoCs where LLCC slices are not configured, abort probe safely.
	 */
	priv->llc_comp = llcc_slice_getd(LLCC_QPACE_COMPRESSION);
	if (IS_ERR_OR_NULL(priv->llc_comp)) {
		ret = priv->llc_comp ? PTR_ERR(priv->llc_comp) : -ENODEV;
		if (ret != -EPROBE_DEFER)
			dev_info(dev, "no compression LLCC slice (%d), hardware not present\n", ret);
		return ret;
	}

	priv->llc_decomp = llcc_slice_getd(LLCC_QPACE_DECOMPRESSION);
	if (IS_ERR_OR_NULL(priv->llc_decomp)) {
		ret = priv->llc_decomp ? PTR_ERR(priv->llc_decomp) : -ENODEV;
		llcc_slice_putd(priv->llc_comp);
		priv->llc_comp = NULL;
		if (ret != -EPROBE_DEFER)
			dev_info(dev, "no decompression LLCC slice (%d), hardware not present\n", ret);
		return ret;
	}

	ret = llcc_slice_activate(priv->llc_comp);
	if (ret) {
		dev_err_probe(dev, ret, "failed to activate compression LLCC slice\n");
		goto llc_put;
	}

	ret = llcc_slice_activate(priv->llc_decomp);
	if (ret) {
		dev_err_probe(dev, ret, "failed to activate decompression LLCC slice\n");
		goto llc_deactivate_comp;
	}

	qpace_priv = priv;
	ret = qpace_power_on(dev);
	if (ret) {
		qpace_priv = NULL;
		goto llc_deactivate_decomp;
	}

	/* Get QPaCE HW version. */
	priv->hw_version = qpace_read_gen(priv, QPACE_CORE_HW_VERSION_OFFSET);
	if (priv->hw_version != QPACE_HW_VERSION_V2) {
		dev_err(dev, "Unsupported QPaCE HW version returned: 0x%x\n",
			priv->hw_version);
		ret = -EINVAL;
		goto power_off;
	}

	ret = qpace_hw_init();
	if (ret) {
		dev_err(dev, "init failed: (%d)\n", ret);
		goto power_off;
	}

	ret = qpace_register_interrupts(pdev);
	if (ret) {
		dev_err(dev, "failed to register interrupts\n");
		goto power_off;
	}

	static_branch_enable(&qpace_drv_probed);
	dev_info(dev, "Qualcomm Page Compression Engine (QPaCE) initialized\n");
	return 0;

power_off:
	qpace_power_off(dev);
llc_deactivate_decomp:
	llcc_slice_deactivate(priv->llc_decomp);
llc_deactivate_comp:
	llcc_slice_deactivate(priv->llc_comp);
llc_put:
	llcc_slice_putd(priv->llc_decomp);
	llcc_slice_putd(priv->llc_comp);
	priv->llc_decomp = NULL;
	priv->llc_comp = NULL;
	qpace_priv = NULL;
	return ret;
}

static int qpace_remove(struct platform_device *pdev)
{
	unsigned long flags;

	spin_lock_irqsave(&qpace_ref_lock, flags);
	if (qpace_priv)
		qpace_priv->suspended = true;
	spin_unlock_irqrestore(&qpace_ref_lock, flags);

	static_branch_disable(&qpace_drv_probed);

	if (qpace_priv) {
		wait_for_completion(&qpace_priv->no_active_refs);

		/* No callers remain; tear down the hardware. */
		cancel_work_sync(&qpace_priv->disable_work);
		qpace_power_off(&pdev->dev);
		if (qpace_priv->llc_decomp) {
			llcc_slice_deactivate(qpace_priv->llc_decomp);
			llcc_slice_putd(qpace_priv->llc_decomp);
		}
		if (qpace_priv->llc_comp) {
			llcc_slice_deactivate(qpace_priv->llc_comp);
			llcc_slice_putd(qpace_priv->llc_comp);
		}
	}

	spin_lock_irqsave(&qpace_ref_lock, flags);
	qpace_priv = NULL;
	spin_unlock_irqrestore(&qpace_ref_lock, flags);

	return 0;
}

static const struct of_device_id qpace_match_table[] = {
	{ .compatible = "qcom,hawi-qpace" },
	{ }
};
MODULE_DEVICE_TABLE(of, qpace_match_table);

static int qpace_suspend(struct device *dev)
{
	unsigned long flags;

	if (!qpace_priv)
		return 0;

	spin_lock_irqsave(&qpace_ref_lock, flags);
	qpace_priv->suspended = true;

	if (qpace_priv->active_rings) {
		dev_err(dev, "active_rings not 0, abort suspend\n");
		qpace_priv->suspended = false;
		spin_unlock_irqrestore(&qpace_ref_lock, flags);
		return -EBUSY;
	}
	spin_unlock_irqrestore(&qpace_ref_lock, flags);

	if (qpace_priv->llc_comp)
		llcc_slice_deactivate(qpace_priv->llc_comp);
	if (qpace_priv->llc_decomp)
		llcc_slice_deactivate(qpace_priv->llc_decomp);

	return 0;
}

static int qpace_resume(struct device *dev)
{
	unsigned long flags;
	int ret;

	if (!qpace_priv)
		return 0;

	if (qpace_priv->active_rings)
		dev_err(dev, "active_rings not 0, unexpected case\n");

	if (qpace_priv->interconnect) {
		ret = icc_set_bw(qpace_priv->interconnect, 0, 1);
		if (ret)
			goto out_err;
	}

	if (qpace_priv->llc_comp) {
		ret = llcc_slice_activate(qpace_priv->llc_comp);
		if (ret)
			goto out_err;

		ret = llcc_slice_activate(qpace_priv->llc_decomp);
		if (ret) {
			llcc_slice_deactivate(qpace_priv->llc_comp);
			goto out_err;
		}
	}

	spin_lock_irqsave(&qpace_ref_lock, flags);
	program_urg_command_contexts_v2();
	program_decomp_core_cfg();
	qpace_priv->suspended = false;
	spin_unlock_irqrestore(&qpace_ref_lock, flags);
	return 0;

out_err:
	dev_err(dev, "failed to resume QPaCE: %d\n", ret);
	return ret;
}

DEFINE_SIMPLE_DEV_PM_OPS(qpace_pm_ops, qpace_suspend, qpace_resume);

static struct platform_driver qpace_driver = {
	.probe = qpace_probe,
	.remove = qpace_remove,
	.driver = {
		.name = "qcom-qpace",
		.of_match_table = qpace_match_table,
		.pm = pm_sleep_ptr(&qpace_pm_ops),
	},
};

module_platform_driver(qpace_driver);

MODULE_DESCRIPTION("Qualcomm Page Compression Engine driver");
MODULE_LICENSE("GPL");

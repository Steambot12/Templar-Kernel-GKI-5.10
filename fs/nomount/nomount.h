#ifndef _LINUX_NOMOUNT_H
#define _LINUX_NOMOUNT_H

#include <linux/types.h>
#include <linux/idr.h>
#include <linux/list.h>
#include <linux/hashtable.h>
#include <linux/rcupdate.h>
#include <linux/rwsem.h>
#include <linux/atomic.h>
#include <linux/file.h>
#include <linux/key-type.h>
#include <linux/highmem.h>
#include <linux/version.h>
#include <linux/compat.h>

#define NOMOUNT_VERSION "20"
#define NOMOUNT_MAGIC_SIG 0x4E4F4D4F554E54ULL /* "NOMOUNT" in hex */
#define NM_FLAG_IS_DIR      (1 << 0)
#define NM_FLAG_VIRTUAL_DIR (1 << 1)
#define NM_FLAG_WHITEOUT    (1 << 2)

/* flags for cleanup */
#define NM_CLEAR_UIDS  (1 << 0)
#define NM_CLEAR_RULES (1 << 1)
#define NM_CLEAR_EXIT  (1 << 2)

/* logs */
#define nm_debug(fmt, ...) printk(KERN_DEBUG "NoMount: [DEBUG] " fmt, ##__VA_ARGS__)
#define nm_info(fmt, ...) printk(KERN_INFO "NoMount: " fmt, ##__VA_ARGS__)
#define nm_warn(fmt, ...) printk(KERN_WARNING "NoMount: [WARN] " fmt, ##__VA_ARGS__)
#define nm_err(fmt, ...)  printk(KERN_ERR "NoMount: [ERROR] " fmt, ##__VA_ARGS__)

static struct nm_uid_array __rcu *nomount_uids = NULL;
static DEFINE_HASHTABLE(nomount_rules_ht, 6);
static DEFINE_MUTEX(nomount_mutex);
static LIST_HEAD(nomount_sb_list);

/* * Helpers to dynamically calculate the memory address of the strings / structs */
#define nm_get_vpath(rule) ((rule)->paths)
#define nm_get_rpath(rule) ((rule)->paths + (rule)->v_len + 1)
#define nm_get_child_name(rule) (nm_get_vpath(rule) + (rule)->v_len - (rule)->child_len)
#define nm_get_child_rules(array) ((struct nomount_rule **)((array)->hashes + (array)->capacity))
#define nm_children_is_single(children) ((unsigned long)(children) & 1UL)
#define nm_children_single_rule(children) ((struct nomount_rule *)((unsigned long)(children) & ~1UL))
#define nm_children_from_single(rule) ((void *)((unsigned long)(rule) | 1UL))
#define nm_dir_tag(dir_node) READ_ONCE((dir_node)->_tag_ptr)
#define nm_dir_is_virtual(dir_node) (nm_dir_tag((dir_node)) & 1UL)
#define nm_dir_set_owner(dir_node, owner) WRITE_ONCE((dir_node)->_tag_ptr, (unsigned long)(owner) | 1UL)
#define nm_dir_owner(dir_node) ({ \
    unsigned long _tag = nm_dir_tag(dir_node); \
    (_tag & 1UL) ? (struct nomount_rule *)(_tag & ~1UL) : NULL; \
})

struct nm_iop {
    struct inode_operations fake_iop; /* MUST be exactly at offset 0 */
    const struct inode_operations *orig_iop;
    struct nomount_dir_node *dir_node;
    struct rcu_head rcu;

    /* Dentry Operations Hijacking */
    struct dentry_operations fake_dops;
    const struct dentry_operations *orig_dops;
};

struct nm_fop {
    struct file_operations fake_fop;  /* MUST be exactly at offset 0 */
    const struct file_operations *orig_fop;
    struct nomount_dir_node *dir_node;
    struct rcu_head rcu;
};

struct nm_sop {
    struct super_operations fake_sop; /* MUST be exactly at offset 0 */
    const struct super_operations *orig_sop;
    const struct xattr_handler **orig_xattr;
    const struct xattr_handler **fake_xattr;
    struct super_block *sb;
    struct rcu_head rcu;
    struct list_head list;
};

struct nm_inode_info {
    struct path r_path;
    struct nomount_dir_node *dir_node;
    u8 flags;
};

struct nomount_child_array {
    struct rcu_head rcu;
    int count;
    int capacity;
    u32 hashes[];
};

struct nomount_dir_node {
    struct rcu_head rcu;
    void __rcu *children;
    u64 bloom_mask;
    struct inode *v_inode;
    union {
        unsigned long _tag_ptr;
        struct {
            struct nm_iop __rcu *iop;
            struct nm_fop __rcu *fop;
        };
    };
};

struct nomount_rule {
    struct path r_path;
    struct nomount_dir_node *this_dir;
    unsigned long v_ino;
    u32 v_hash;
    unsigned int target_uid;
    u16 v_len;
    u16 r_len;
    u16 child_len;
    u16 flags;

    struct nomount_dir_node *parent_dir;
    struct hlist_node ht_node;
    char paths[];
};

struct nm_rule_info {
    u16 flags;
    unsigned long v_ino;
    struct path r_path;
    struct nomount_dir_node *this_dir;
};

struct nm_uid_array {
    struct rcu_head rcu;
    int count;
    uid_t uids[];
};

/*** Operaction Vectors ***/
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 16, 0)
static const struct file_operations nm_file_fops_mmap_prepare;
#endif
static const struct file_operations nm_file_fops;
static const struct inode_operations nm_file_iops;
static const struct file_operations nm_dir_fops;
static const struct inode_operations nm_dir_iops;
static const struct dentry_operations nm_dops;
static const struct dentry_operations nm_owned_dops;

/*** forward declarations ***/
static struct dentry *nomount_hijacked_lookup(struct inode *dir, struct dentry *dentry, unsigned int flags);
static int nomount_hijacked_iterate_dir(struct file *file, struct dir_context *ctx);
static void nomount_hijacked_evict_inode(struct inode *inode);
static void nomount_hijack_dentry_ops(struct inode *dir, struct dentry *dentry, bool injected);
static void nm_free_rule(struct nomount_rule *rule);

/* =====================================================================
 * NoMount VFS Offset Protocol
 * =====================================================================
 * 64-bit layout: [ 16-bit 'nm' ][ 16-bit 0 ][ 32-bit ID ] 
 * 32-bit layout: [ 16-bit 'nm' ][ 16-bit ID ]
 */
#define NM_SIG_16 0x6E6DULL /* "nm" in hex */
static inline bool nm_is_virtual_pos(loff_t pos) {
#ifdef CONFIG_COMPAT
    if (in_compat_syscall()) return (pos & 0xFFFF0000ULL) == (NM_SIG_16 << 16);
#endif
    return (pos & 0xFFFFFFFF00000000ULL) == (NM_SIG_16 << 48);
}

static inline loff_t nm_pack_pos(int id) {
#ifdef CONFIG_COMPAT
    if (in_compat_syscall()) return (NM_SIG_16 << 16) | (id & 0xFFFF);
#endif
    return (NM_SIG_16 << 48) | (id & 0xFFFFFFFF);
}

static inline int nm_unpack_pos(loff_t pos) {
#ifdef CONFIG_COMPAT
    if (in_compat_syscall()) return (int)(pos & 0xFFFF);
#endif
    return (int)(pos & 0xFFFFFFFF);
}

/* --- UIDs Array RCU Management --- */
static inline int nm_uid_add(uid_t target)
{
    struct nm_uid_array *old, *new_arr;
    int count = 0;
    if ((old = rcu_dereference_protected(nomount_uids, lockdep_is_held(&nomount_mutex)))) {
        for (int i = 0; i < (count = old->count); i++) if (old->uids[i] == target) return -EEXIST;
    }

    if (!(new_arr = kmalloc(sizeof(*new_arr) + (count + 1) * sizeof(uid_t), GFP_KERNEL))) return -ENOMEM;
    new_arr->count = count + 1;
    if (old) memcpy(new_arr->uids, old->uids, count * sizeof(uid_t));
    new_arr->uids[count] = target;
    rcu_assign_pointer(nomount_uids, new_arr);
    if (old) kfree_rcu(old, rcu);
    return 0;
}

static inline int nm_uid_del(uid_t target)
{
    struct nm_uid_array *old, *new_arr = NULL;
    int count, target_idx = -1;

    if (!(old = rcu_dereference_protected(nomount_uids, lockdep_is_held(&nomount_mutex)))) return -ENOENT;
    for (int i = 0; i < (count = old->count); i++) if (old->uids[i] == target) { target_idx = i; break; }
    if (target_idx < 0) return -ENOENT;

    if (count > 1) {
        if (!(new_arr = kmalloc(sizeof(*new_arr) + (count - 1) * sizeof(uid_t), GFP_KERNEL))) return -ENOMEM;
        new_arr->count = count - 1;
        if (target_idx > 0) memcpy(new_arr->uids, old->uids, target_idx * sizeof(uid_t));
        if (target_idx < count - 1) memcpy(new_arr->uids + target_idx, old->uids + target_idx + 1, (count - target_idx - 1) * sizeof(uid_t));
    }
    rcu_assign_pointer(nomount_uids, new_arr);
    kfree_rcu(old, rcu);
    return 0;
}

/* ============================ */
/* NOMOUNT PAYLOAD PROTOCOL     */
/* ============================ */

enum {
    NM_CMD_UNSPEC = 0,
    NM_CMD_GET_VERSION,
    NM_CMD_ADD_RULE,
    NM_CMD_DEL_RULE,
    NM_CMD_ADD_UID,
    NM_CMD_DEL_UID,
    NM_CMD_CLEAR_ALL,
    NM_CMD_CLEAR_RULES,
    NM_CMD_CLEAR_UIDS,
    NM_CMD_GET_LIST,
    NM_CMD_GET_UIDS,
    NM_CMD_BLOCK_ISOLATED_UIDS,
    NM_CMD_GET_ISOLATED_STATE,
};

struct nm_payload {
    u64 magic;
    u32 cmd;
    u32 target_uid;
    int status;
    u32 arg1;
    u32 data_size;
    char buffer[4068];
} __attribute__((packed));

struct nm_rule_hdr {
	u32 flags;
	u32 uid;
	u16 v_len;
	u16 r_len;
} __attribute__((packed));

struct nm_del_hdr {
	u32 uid;
	u16 v_len;
} __attribute__((packed));

/* * Compat macros * */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
    #define IDMAP_PATH(path) mnt_idmap((path).mnt),
    #define IDMAP_ARG struct mnt_idmap *idmap,
    #define IDMAP_CALL idmap,
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(5, 12, 0)
    #define IDMAP_PATH(path) mnt_user_ns((path).mnt),
    #define IDMAP_ARG struct user_namespace *mnt_userns,
    #define IDMAP_CALL mnt_userns,
#else
    #define IDMAP_PATH(path)/* Nothing */
    #define IDMAP_ARG /* Nothing */
    #define IDMAP_CALL /* Nothing */
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0)
    #define NM_ACTOR_RET bool
    #define NM_ACTOR_CONTINUE true
#else
    #define NM_ACTOR_RET int
    #define NM_ACTOR_CONTINUE 0
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 12, 0) && LINUX_VERSION_CODE >= KERNEL_VERSION(5, 2, 0)
    #define FLAGS_ARG , int flags
    #define FLAGS_VAL , flags
#else
    #define FLAGS_ARG /* Nothing */
    #define FLAGS_VAL /* Nothing */
#endif

#ifndef DCACHE_DONTCACHE
# define DCACHE_DONTCACHE 0
#endif

static inline void nm_sync_inode_times(struct inode *v_inode, struct inode *r_inode)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 12, 0)
    v_inode->i_atime_sec = r_inode->i_atime_sec;
    v_inode->i_atime_nsec = r_inode->i_atime_nsec;
    v_inode->i_mtime_sec = r_inode->i_mtime_sec;
    v_inode->i_mtime_nsec = r_inode->i_mtime_nsec;
    v_inode->i_ctime_sec = r_inode->i_ctime_sec;
    v_inode->i_ctime_nsec = r_inode->i_ctime_nsec;
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(6, 6, 0)
    v_inode->i_atime = r_inode->i_atime;
    v_inode->i_mtime = r_inode->i_mtime;
    inode_set_ctime_to_ts(v_inode, inode_get_ctime(r_inode));
#else
    v_inode->i_atime = r_inode->i_atime;
    v_inode->i_mtime = r_inode->i_mtime;
    v_inode->i_ctime = r_inode->i_ctime;
#endif
}

static inline int nm_call_iterate(struct file *file, struct dir_context *ctx, const struct file_operations *fop)
{
    if (fop->iterate_shared)
        return fop->iterate_shared(file, ctx);
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 6, 0)
    else if (fop->iterate)
        return fop->iterate(file, ctx);
#endif
    return -ENOTDIR;
}

static inline struct dentry *nm_hash_and_lookup(struct dentry *dir, struct qstr *n) {
    n->hash = full_name_hash(dir, n->name, n->len);
    return (unlikely(dir->d_flags & DCACHE_OP_HASH) && dir->d_op->d_hash(dir, n) < 0) ? NULL : d_lookup(dir, n);
}

static inline struct nm_iop *nm_get_nm_iop(const struct inode_operations *iop) {
    if (likely(iop) && iop->lookup == nomount_hijacked_lookup)
        return container_of(iop, struct nm_iop, fake_iop);
    return NULL;
}

static inline struct nm_fop *nm_get_nm_fop(const struct file_operations *fop) {
    if (unlikely(!fop)) return NULL;
    if (fop->iterate_shared == nomount_hijacked_iterate_dir)
        return container_of(fop, struct nm_fop, fake_fop);
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 6, 0)
    if (fop->iterate == nomount_hijacked_iterate_dir)
        return container_of(fop, struct nm_fop, fake_fop);
#endif
    return NULL;
}

static inline struct nm_sop *nm_get_nm_sop(const struct super_operations *sop) {
    if (likely(sop) && sop->evict_inode == nomount_hijacked_evict_inode)
        return container_of(sop, struct nm_sop, fake_sop);
    return NULL;
}

#define NM_DOP_INITIALIZING ((const struct dentry_operations *)1L)
static inline const struct dentry_operations *nm_get_orig_dops(struct nm_iop *iop)
{
    const struct dentry_operations *dops;
    if (!iop) return NULL;
    dops = smp_load_acquire(&iop->orig_dops);
    return (dops == NM_DOP_INITIALIZING) ? NULL : dops;
}

#endif /* _LINUX_NOMOUNT_H */

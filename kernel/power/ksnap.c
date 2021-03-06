/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2009 Wind River Systems,
 *   written by Ralf Baechle <ralf.baechle@windriver.com>
 */
#include <linux/cred.h>
#include <linux/elf.h>
#include <linux/elfcore.h>
#include <linux/err.h>
#include <linux/fcntl.h>
#include <linux/freezer.h>
#include <linux/fs.h>
#include <linux/gfp.h>
#include <linux/kobject.h>
#include <linux/list.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/stat.h>
#include <linux/suspend.h>
#include <linux/uaccess.h>

#include "power.h"

#define CORE_STR "CORE"

#ifndef ELF_CORE_EFLAGS
#define ELF_CORE_EFLAGS 0
#endif

/*
 * Access to all global variables related to snapshot generation is
 * serialized by the pm_mutex.
 */
static LIST_HEAD(ext_list);

struct extend {
	struct list_head	ext_list;
	unsigned long		ext_first;
	unsigned long		ext_last;
	loff_t			ext_offset;
	const void		*ext_addr;
	struct elf_phdr		ext_phdr;
};

static unsigned int ext_number;
static loff_t ext_offset;

/*
 * Return a filled-in ELF core file header
 */
static struct elfhdr *get_elfhdr(void)
{
	struct elfhdr *hdr;

	hdr = kzalloc(sizeof(*hdr), GFP_KERNEL);
	if (!hdr)
		return NULL;

	memcpy(hdr->e_ident, ELFMAG, SELFMAG);
	hdr->e_ident[EI_CLASS]		= ELF_CLASS;
	hdr->e_ident[EI_DATA]		= ELF_DATA;
	hdr->e_ident[EI_VERSION]	= EV_CURRENT;
	hdr->e_ident[EI_OSABI]		= ELF_OSABI;

	hdr->e_type			= ET_CORE;
	hdr->e_machine			= ELF_ARCH;
	hdr->e_version			= EV_CURRENT;
	hdr->e_phoff			= sizeof(struct elfhdr);
	hdr->e_flags			= ELF_CORE_EFLAGS;
	hdr->e_ehsize			= sizeof(struct elfhdr);
	hdr->e_phentsize		= sizeof(struct elf_phdr);

	return hdr;
}

static int ext_add(unsigned long first, unsigned long last)
{
	struct extend *ext;
	struct elf_phdr *ph;

	/*
	 * We had at least one valid page, create
	 * extend.
	 */
	ext = kmalloc(sizeof(struct extend), GFP_KERNEL);
	if (!ext)
		return -ENOMEM;

	ext->ext_first = first;
	ext->ext_last  = last;
	ext->ext_addr  = page_address(pfn_to_page(first));
	list_add_tail(&ext->ext_list, &ext_list);
	ext_number++;

	ext->ext_offset += ext_offset;
	ph = &ext->ext_phdr;
	ph->p_type	= PT_LOAD;
	ph->p_offset	= 0;
	ph->p_vaddr	= (unsigned long) ext->ext_addr;
	ph->p_paddr	= 0;
	ph->p_filesz	= (last - first + 1) << PAGE_SHIFT;
	ph->p_memsz	= (last - first + 1) << PAGE_SHIFT;
	ph->p_flags	= PF_R | PF_W;
	ph->p_align	= PAGE_SIZE;

	ext_offset += sizeof(struct elf_phdr);

	return 0;
}

/*
 * An ELF note in memory
 */
struct memelfnote {
	const char *name;
	int type;
	unsigned int datasz;
	void *data;
};

static unsigned long notes_page;

static int notesize(struct memelfnote *en)
{
	int sz;

	sz = sizeof(struct elf_note);
	sz += roundup((strlen(en->name) + 1), 4);
	sz += roundup(en->datasz, 4);

	return sz;
}

static char *storenote(struct memelfnote *men, char *buf)
{
	struct elf_note en;

	en.n_namesz = strlen(men->name) + 1;
	en.n_descsz = men->datasz;
	en.n_type = men->type;

	memcpy(buf, &en, sizeof(en));
	buf += sizeof(en);
	memcpy(buf, men->name, en.n_namesz);
	buf += en.n_namesz;

	buf = (char*) roundup((unsigned long)buf, 4);
	memcpy(buf, men->data, men->datasz);
	buf += men->datasz;
	buf = (char*) roundup((unsigned long)buf, 4);

	return buf;
}

static int ext_add_notes_phdr(void)
{
	struct elf_prstatus prstatus;	/* NT_PRSTATUS */
	struct elf_prpsinfo prpsinfo;	/* NT_PRPSINFO */
	struct memelfnote notes[3];
	struct elf_phdr *ph;
	struct extend *ext;
	char *buf;
	int ret;

	/*
	 * We had at least one valid page, create
	 * extend.
	 */
	ext = kmalloc(sizeof(struct extend), GFP_KERNEL);
	if (!ext)
		return -ENOMEM;

	notes_page =  __get_free_pages(GFP_KERNEL, 0);
	if (!notes_page) {
		ret = -ENOMEM;
		goto out_free_extend;
	}
	buf = (char *) notes_page;

	ext->ext_first = 0;
	ext->ext_last  = 0;
	list_add_tail(&ext->ext_list, &ext_list);
	ext_number++;

	ext->ext_offset = ext_offset;
	ph = &ext->ext_phdr;
	ph->p_type	= PT_NOTE;
	ph->p_offset	= 0;
	ph->p_vaddr	= 0;
	ph->p_paddr	= 0;
	ph->p_filesz	= 0;
	ph->p_memsz	= 0;
	ph->p_flags	= 0;
	ph->p_align	= 0;

	ext->ext_addr  = buf;

	/*
	 * Set up the process status
	 */
	notes[0].name = CORE_STR;
	notes[0].type = NT_PRSTATUS;
	notes[0].datasz = sizeof(struct elf_prstatus);
	notes[0].data = &prstatus;

	memset(&prstatus, 0, sizeof(struct elf_prstatus));

	ph->p_filesz  = notesize(&notes[0]);
	buf = storenote(notes + 0, buf);

	/*
	 * Set up the process info
	 */
	notes[1].name   = CORE_STR;
	notes[1].type   = NT_PRPSINFO;
	notes[1].datasz = sizeof(struct elf_prpsinfo);
	notes[1].data   = &prpsinfo;

	memset(&prpsinfo, 0, sizeof(struct elf_prpsinfo));
	prpsinfo.pr_state       = 0;
	prpsinfo.pr_sname       = 'R';
	prpsinfo.pr_zomb        = 0;

	strcpy(prpsinfo.pr_fname, "vmlinux");
	strncpy(prpsinfo.pr_psargs, saved_command_line, ELF_PRARGSZ);

	ph->p_filesz  += notesize(&notes[1]);
	buf = storenote(notes + 1, buf);

	/*
	 * Set up the task structure
	 */
	notes[2].name   = CORE_STR;
	notes[2].type   = NT_TASKSTRUCT;
	notes[2].datasz = sizeof(struct task_struct);
	notes[2].data   = current;

	ph->p_filesz  += notesize(&notes[2]);
	buf = storenote(notes + 2, buf);

	ext_offset += sizeof(struct elf_phdr);

	return 0;

out_free_extend:
	kfree(ext);
	return ret;
}

static void ext_layout(void)
{
	struct extend *ext;

	list_for_each_entry(ext, &ext_list, ext_list) {
		struct elf_phdr *ph = &ext->ext_phdr;
		unsigned long align = ph->p_align ? ph->p_align : 1;

		ext_offset = (ext_offset + align - 1) & ~(align - 1);
		ph->p_offset = ext_offset;
		ext_offset += ph->p_filesz;
	}
}

static void ext_free(void)
{
	struct extend *ext, *tmp;

	list_for_each_entry_safe(ext, tmp, &ext_list, ext_list) {
		list_del(&ext->ext_list);
		ext_number--;
		kfree(ext);
	}
}

static int ext_create(void)
{
	struct zone *zone;
	unsigned long pfn;
	int res, i = 0;

	for_each_populated_zone(zone) {
		unsigned long start_pfn, end_pfn;
		unsigned long ext_start, invalid;

		start_pfn = zone->zone_start_pfn;
		end_pfn = start_pfn + zone->spanned_pages;

		ext_start = -1;
		invalid = 0;
		for (pfn = start_pfn; pfn < end_pfn; pfn++) {
			if (!pfn_valid(pfn))
				invalid++;
			if (pfn_valid(pfn)) {
				if (ext_start == -1)
					ext_start = pfn;
			} else {
				if (ext_start == -1)
					continue;

				res = ext_add(ext_start, pfn - 1);
				if (res)
					goto out_free;

				ext_start = -1;
			}
		}
		i++;
	}

	return 0;

out_free:
	ext_free();

	return res;
}

static struct file *dump_open(const char *sname)
{
	struct inode *inode;
	struct file *file;

	file = filp_open(sname,
			 O_CREAT | 2 | O_NOFOLLOW | O_LARGEFILE, 0600);

	if (IS_ERR(file))
		goto fail_unlock;
	inode = file->f_path.dentry->d_inode;
	if (inode->i_nlink > 1)
		goto close_fail;	/* multiple links - don't dump */
	if (d_unhashed(file->f_path.dentry))
		goto close_fail;

	if (!S_ISREG(inode->i_mode))
		goto close_fail;
	/*
	 * Dont allow local users get cute and trick others to coredump
	 * into their pre-created files:
	 */
	if (inode->i_uid != current_fsuid())
		goto close_fail;

	if (!file->f_op)
		goto close_fail;
	if (!file->f_op->write)
		goto close_fail;
	if (do_truncate(file->f_path.dentry, 0, 0, file) != 0)
		goto close_fail;

	return file;

close_fail:
	filp_close(file, NULL);
fail_unlock:

	return file;
}

static void dump_close(struct file *file)
{
	filp_close(file, NULL);
}

static int dump_write(struct file *file, const void *addr, int nr)
{
	ssize_t ret;

	ret = file->f_op->write(file, addr, nr, &file->f_pos);

	if (IS_ERR((void *)ret))
		return ret;

	return 0;
}

static int dump_write_phdrs(struct file *file)
{
	struct extend *ext;
	int ret;

	list_for_each_entry(ext, &ext_list, ext_list) {
		ret = dump_write(file, &ext->ext_phdr, sizeof(ext->ext_phdr));
		if (ret)
			return ret;
	}

	return 0;
}

static int dump_seek(struct file *file, loff_t off)
{
	loff_t pos;

	if (!file->f_op->llseek || file->f_op->llseek == no_llseek)
		return -ESPIPE;

	pos = file->f_op->llseek(file, off, SEEK_SET);
	if (IS_ERR((void *)pos))
		return pos;

        return 0;
}

static int dump_write_extends(struct file *file)
{
	struct extend *ext;
	struct elf_phdr *ph;
	int ret;

	list_for_each_entry(ext, &ext_list, ext_list) {
		if (!ext->ext_addr)
			continue;

		ph = &ext->ext_phdr;
		ret = dump_seek(file, ph->p_offset);
		if (ret) {
			return ret;
		}

		ret = dump_write(file, ext->ext_addr, ph->p_filesz);
		if (ret)
			return ret;
	}

	return 0;
}

/*
 * The system is mostly locked down.  Do the real snapshot.
 */
static int __init dump_ksnap_locked(const char *sname)
{
	struct elfhdr *hdr;
	struct file *file;
	int res;
	mm_segment_t fs;

	ext_offset = sizeof(struct elfhdr);
	res = ext_add_notes_phdr();
	if (res)
		goto out_free_ext;
	res = ext_create();
	if (res)
		goto out_free_notes;

	hdr = get_elfhdr();
	if (!hdr)
		goto out_free_ext;
	hdr->e_phnum = ext_number;

	ext_layout();

	file = dump_open(sname);
	if (IS_ERR(file))
		goto out_free_hdr;
	fs = get_fs();
	set_fs(KERNEL_DS);
	if (dump_write(file, hdr, sizeof(*hdr)))
		goto out_restore_fs;
	if (dump_write_phdrs(file))
		goto out_restore_fs;
	if (dump_write_extends(file))
		goto out_restore_fs;
	set_fs(fs);
	dump_close(file);

	ext_free();
	kfree(hdr);

	return 0;

out_restore_fs:
	set_fs(fs);
out_free_hdr:
	kfree(hdr);
out_free_notes:
	free_page(notes_page);
out_free_ext:
	ext_free();

	return res;
}

static int prepare_processes(void)
{
	int ret = 0;

	if (freeze_processes()) {
		ret = -EBUSY;
		thaw_processes();
	}

	return ret;
}

static int __init dump_ksnap(const char *sname)
{
	int ret;

	mutex_lock(&pm_mutex);

	ret = pm_notifier_call_chain(PM_HIBERNATION_PREPARE);
	if (ret)
		goto out_post_hibernation;

	ret = prepare_processes();
	if (ret)
		goto out_post_hibernation;

	ret = dump_ksnap_locked(sname);

	thaw_processes();
out_post_hibernation:
	pm_notifier_call_chain(PM_POST_HIBERNATION);
	mutex_unlock(&pm_mutex);

	return ret;
}

struct kobject *ksnap_kobj;

static ssize_t ksnap_store(struct kobject *kobj, struct kobj_attribute *attr,
	const char *buf, size_t n)
{
	int ret;
	char *p;

	if (!capable(CAP_SYS_RAWIO))
		return -EPERM;

	p = memchr(buf, '\n', n);
	if (p)
		*p = '\0';
	ret = dump_ksnap(buf);
	if (ret)
		return ret;

	return n;
}

static struct kobj_attribute ksnap_attr = {
	.attr   = {
		.name = "ksnap",
		.mode = 0200,
	},
	.store  = ksnap_store,
};

static struct attribute *ksnap_group_attr[] = {
	&ksnap_attr.attr,
        NULL,
};

static struct attribute_group attr_group = {
	.attrs = ksnap_group_attr,
};

static int __init ksnap_init(void)
{
	ksnap_kobj = kobject_create_and_add("ksnap", NULL);
	if (!ksnap_kobj)
		return -ENOMEM;
	return sysfs_create_group(ksnap_kobj, &attr_group);
}

core_initcall(ksnap_init);

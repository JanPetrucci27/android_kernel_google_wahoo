/*
 * arch/arm64/kernel/topology.c
 *
 * Copyright (C) 2011,2013,2014 Linaro Limited.
 *
 * Based on the arm32 version written by Vincent Guittot in turn based on
 * arch/sh/kernel/topology.c
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 */

#include <linux/cpu.h>
#include <linux/cpumask.h>
#include <linux/init.h>
#include <linux/percpu.h>
#include <linux/node.h>
#include <linux/nodemask.h>
#include <linux/of.h>
#include <linux/sched.h>
#include <linux/sched.h>
#include <linux/sched_energy.h>

#include <asm/cputype.h>
#include <asm/topology.h>

static DEFINE_PER_CPU(unsigned long, cpu_efficiency) = SCHED_CAPACITY_SCALE;

unsigned long arch_get_cpu_efficiency(int cpu)
{
	return per_cpu(cpu_efficiency, cpu);
}

static DEFINE_PER_CPU(unsigned long, cpu_scale) = SCHED_CAPACITY_SCALE;

unsigned long scale_cpu_capacity(int cpu)
{
	return per_cpu(cpu_scale, cpu);
}

static void set_capacity_scale(unsigned int cpu, unsigned long capacity)
{
	per_cpu(cpu_scale, cpu) = capacity;
}

static DEFINE_PER_CPU(unsigned long, hw_pressure) = 0;

unsigned long topology_get_hw_pressure(int cpu)
{
	return per_cpu(hw_pressure, cpu);
}

/**
 * topology_update_hw_pressure() - Update HW pressure for CPUs
 * @cpus        : The related CPUs for which capacity has been reduced
 * @capped_freq : The maximum allowed frequency that CPUs can run at
 *
 * Update the value of HW pressure for all @cpus in the mask. The
 * cpumask should include all (online+offline) affected CPUs, to avoid
 * operating on stale data when hot-plug is used for some CPUs. The
 * @capped_freq reflects the currently allowed max CPUs frequency due to
 * HW capping. It might be also a boost frequency value, which is bigger
 * than the internal 'capacity_freq_ref' max frequency. In such case the pressure
 * value should simply be removed, since this is an indication that there is
 * no HW throttling. The @capped_freq must be provided in kHz.
 */
void topology_update_hw_pressure(const struct cpumask *cpus,
				      unsigned long capped_freq)
{
	unsigned long max_capacity, capacity, pressure;
	u32 max_freq;
	int cpu;

	cpu = cpumask_first(cpus);
	max_capacity = arch_scale_cpu_capacity(cpu);
	max_freq = arch_scale_freq_ref(cpu);

	if (!capped_freq)
		capped_freq = max_freq;

	/*
	 * Handle properly the boost frequencies, which should simply clean
	 * the HW pressure value.
	 */
	if (max_freq <= capped_freq)
		capacity = max_capacity;
	else
		capacity = mult_frac(max_capacity, capped_freq, max_freq);

	pressure = max_capacity - capacity;

	for_each_cpu(cpu, cpus)
		WRITE_ONCE(per_cpu(hw_pressure, cpu), pressure);
}
EXPORT_SYMBOL_GPL(topology_update_hw_pressure);

static int __init get_cpu_for_node(struct device_node *node)
{
	struct device_node *cpu_node;
	int cpu;

	cpu_node = of_parse_phandle(node, "cpu", 0);
	if (!cpu_node)
		return -1;

	for_each_possible_cpu(cpu) {
		if (of_get_cpu_node(cpu, NULL) == cpu_node) {
			of_node_put(cpu_node);
			return cpu;
		}
	}

	pr_crit("Unable to find CPU node for %s\n", cpu_node->full_name);

	of_node_put(cpu_node);
	return -1;
}

static int __init parse_core(struct device_node *core, int cluster_id,
			     int core_id)
{
	char name[10];
	bool leaf = true;
	int i = 0;
	int cpu;
	struct device_node *t;

	do {
		snprintf(name, sizeof(name), "thread%d", i);
		t = of_get_child_by_name(core, name);
		if (t) {
			leaf = false;
			cpu = get_cpu_for_node(t);
			if (cpu >= 0) {
				cpu_topology[cpu].cluster_id = cluster_id;
				cpu_topology[cpu].core_id = core_id;
				cpu_topology[cpu].thread_id = i;
			} else {
				pr_err("%s: Can't get CPU for thread\n",
				       t->full_name);
				of_node_put(t);
				return -EINVAL;
			}
			of_node_put(t);
		}
		i++;
	} while (t);

	cpu = get_cpu_for_node(core);
	if (cpu >= 0) {
		if (!leaf) {
			pr_err("%s: Core has both threads and CPU\n",
			       core->full_name);
			return -EINVAL;
		}

		cpu_topology[cpu].cluster_id = cluster_id;
		cpu_topology[cpu].core_id = core_id;
	} else if (leaf) {
		pr_err("%s: Can't get CPU for leaf core\n", core->full_name);
		return -EINVAL;
	}

	return 0;
}

static int __init parse_cluster(struct device_node *cluster, int depth)
{
	char name[10];
	bool leaf = true;
	bool has_cores = false;
	struct device_node *c;
	static int cluster_id __initdata;
	int core_id = 0;
	int i, ret;

	/*
	 * First check for child clusters; we currently ignore any
	 * information about the nesting of clusters and present the
	 * scheduler with a flat list of them.
	 */
	i = 0;
	do {
		snprintf(name, sizeof(name), "cluster%d", i);
		c = of_get_child_by_name(cluster, name);
		if (c) {
			leaf = false;
			ret = parse_cluster(c, depth + 1);
			of_node_put(c);
			if (ret != 0)
				return ret;
		}
		i++;
	} while (c);

	/* Now check for cores */
	i = 0;
	do {
		snprintf(name, sizeof(name), "core%d", i);
		c = of_get_child_by_name(cluster, name);
		if (c) {
			has_cores = true;

			if (depth == 0) {
				pr_err("%s: cpu-map children should be clusters\n",
				       c->full_name);
				of_node_put(c);
				return -EINVAL;
			}

			if (leaf) {
				ret = parse_core(c, cluster_id, core_id++);
			} else {
				pr_err("%s: Non-leaf cluster with core %s\n",
				       cluster->full_name, name);
				ret = -EINVAL;
			}

			of_node_put(c);
			if (ret != 0)
				return ret;
		}
		i++;
	} while (c);

	if (leaf && !has_cores)
		pr_warn("%s: empty cluster\n", cluster->full_name);

	if (leaf)
		cluster_id++;

	return 0;
}

static int __init parse_dt_topology(void)
{
	struct device_node *cn, *map;
	int ret = 0;
	int cpu;
	u32 efficiency;

	cn = of_find_node_by_path("/cpus");
	if (!cn) {
		pr_err("No CPU information found in DT\n");
		return 0;
	}

	/*
	 * When topology is provided cpu-map is essentially a root
	 * cluster with restricted subnodes.
	 */
	map = of_get_child_by_name(cn, "cpu-map");
	if (!map)
		goto out;

	ret = parse_cluster(map, 0);
	if (ret != 0)
		goto out_map;

	/*
	 * Check that all cores are in the topology; the SMP code will
	 * only mark cores described in the DT as possible.
	 */
	for_each_possible_cpu(cpu) {
		if (cpu_topology[cpu].cluster_id == -1)
			ret = -EINVAL;

		/* The CPU efficiency value passed from the device tree */
		cn = of_get_cpu_node(cpu, NULL);
		if (of_property_read_u32(cn, "efficiency", &efficiency) < 0) {
			WARN_ON(1);
			continue;
		}
		per_cpu(cpu_efficiency, cpu) = efficiency;
	}

out_map:
	of_node_put(map);
out:
	of_node_put(cn);
	return ret;
}

/*
 * cpu topology table
 */
struct cpu_topology cpu_topology[NR_CPUS];
EXPORT_SYMBOL_GPL(cpu_topology);

/* sd energy functions */
static inline
const struct sched_group_energy * const cpu_cluster_energy(int cpu)
{
	struct sched_group_energy *sge = sge_array[cpu][SD_LEVEL1];

	if (!sge) {
		pr_warn("Invalid sched_group_energy for Cluster%d\n", cpu);
		return NULL;
	}

	return sge;
}

static inline
const struct sched_group_energy * const cpu_core_energy(int cpu)
{
	struct sched_group_energy *sge = sge_array[cpu][SD_LEVEL0];

	if (!sge) {
		pr_warn("Invalid sched_group_energy for CPU%d\n", cpu);
		return NULL;
	}

	return sge;
}

const struct cpumask *cpu_coregroup_mask(int cpu)
{
	const cpumask_t *core_mask = cpumask_of_node(cpu_to_node(cpu));

	/* Find the smaller of NUMA, core or LLC siblings */
	if (cpumask_subset(&cpu_topology[cpu].core_sibling, core_mask)) {
		/* not numa in package, lets use the package siblings */
		core_mask = &cpu_topology[cpu].core_sibling;
	}

	/*
	 * For systems with no shared cpu-side LLC but with clusters defined,
	 * extend core_mask to cluster_siblings. The sched domain builder will
	 * then remove MC as redundant with CLS if SCHED_CLUSTER is enabled.
	 */
	if (IS_ENABLED(CONFIG_SCHED_CLUSTER) &&
		cpumask_subset(core_mask, &cpu_topology[cpu].cluster_sibling))
		core_mask = &cpu_topology[cpu].cluster_sibling;

	return core_mask;
}

const struct cpumask *cpu_clustergroup_mask(int cpu)
{
	/*
	 * Forbid cpu_clustergroup_mask() to span more or the same CPUs as
	 * cpu_coregroup_mask().
	 */
	if (cpumask_subset(cpu_coregroup_mask(cpu),
		&cpu_topology[cpu].cluster_sibling))
		return topology_sibling_cpumask(cpu);

	return &cpu_topology[cpu].cluster_sibling;
}

static struct sched_domain_topology_level arm64_topology[] = {
#ifdef CONFIG_SCHED_CLUSTER
	{ cpu_clustergroup_mask, cpu_cluster_flags, cpu_core_energy, SD_INIT_NAME(CLS) },
#endif

#ifdef CONFIG_SCHED_MC
	{ cpu_coregroup_mask, cpu_core_flags, cpu_core_energy, SD_INIT_NAME(MC) },
#endif
	{ cpu_cpu_mask, NULL, cpu_cluster_energy, SD_INIT_NAME(DIE) },
	{ NULL, },
};

void update_cpu_power_capacity(int cpu)
{
	unsigned long capacity = SCHED_CAPACITY_SCALE;

	if (cpu_core_energy(cpu)) {
		int max_cap_idx = cpu_core_energy(cpu)->nr_cap_states - 1;
		capacity = cpu_core_energy(cpu)->cap_states[max_cap_idx].cap;
	}

	set_capacity_scale(cpu, capacity);

	pr_info("CPU%d: update cpu_capacity %lu\n",
		cpu, arch_scale_cpu_capacity(cpu));
}

static void update_siblings_masks(unsigned int cpuid)
{
	struct cpu_topology *cpu_topo, *cpuid_topo = &cpu_topology[cpuid];
	int cpu;

	/* update core and thread sibling masks */
	for_each_possible_cpu(cpu) {
		cpu_topo = &cpu_topology[cpu];

		cpumask_set_cpu(cpuid, &cpu_topo->core_sibling);
		if (cpu != cpuid)
			cpumask_set_cpu(cpu, &cpuid_topo->core_sibling);

		if (cpuid_topo->cluster_id != cpu_topo->cluster_id)
			continue;

		if (cpuid_topo->cluster_id >= 0) {
			cpumask_set_cpu(cpuid, &cpu_topo->cluster_sibling);
			if (cpu != cpuid)
				cpumask_set_cpu(cpu, &cpuid_topo->cluster_sibling);
		}

		if (cpuid_topo->core_id != cpu_topo->core_id)
			continue;

		cpumask_set_cpu(cpuid, &cpu_topo->thread_sibling);
		if (cpu != cpuid)
			cpumask_set_cpu(cpu, &cpuid_topo->thread_sibling);
	}
}

void store_cpu_topology(unsigned int cpuid)
{
	struct cpu_topology *cpuid_topo = &cpu_topology[cpuid];
	u64 mpidr;

	if (cpuid_topo->cluster_id != -1)
		goto topology_populated;

	mpidr = read_cpuid_mpidr();

	/* Uniprocessor systems can rely on default topology values */
	if (mpidr & MPIDR_UP_BITMASK)
		return;

	/* Create cpu topology mapping based on MPIDR. */
	if (mpidr & MPIDR_MT_BITMASK) {
		/* Multiprocessor system : Multi-threads per core */
		cpuid_topo->thread_id  = MPIDR_AFFINITY_LEVEL(mpidr, 0);
		cpuid_topo->core_id    = MPIDR_AFFINITY_LEVEL(mpidr, 1);
		cpuid_topo->cluster_id = MPIDR_AFFINITY_LEVEL(mpidr, 2) |
					 MPIDR_AFFINITY_LEVEL(mpidr, 3) << 8;
	} else {
		/* Multiprocessor system : Single-thread per core */
		cpuid_topo->thread_id  = -1;
		cpuid_topo->core_id    = MPIDR_AFFINITY_LEVEL(mpidr, 0);
		cpuid_topo->cluster_id = MPIDR_AFFINITY_LEVEL(mpidr, 1) |
					 MPIDR_AFFINITY_LEVEL(mpidr, 2) << 8 |
					 MPIDR_AFFINITY_LEVEL(mpidr, 3) << 16;
	}

	pr_debug("CPU%u: cluster %d core %d thread %d mpidr %#016llx\n",
		 cpuid, cpuid_topo->cluster_id, cpuid_topo->core_id,
		 cpuid_topo->thread_id, mpidr);

topology_populated:
	update_siblings_masks(cpuid);
}

static void __init reset_cpu_topology(void)
{
	unsigned int cpu;

	for_each_possible_cpu(cpu) {
		struct cpu_topology *cpu_topo = &cpu_topology[cpu];

		cpu_topo->thread_id = -1;
		cpu_topo->core_id = 0;
		cpu_topo->cluster_id = -1;

		cpumask_clear(&cpu_topo->cluster_sibling);
		cpumask_set_cpu(cpu, &cpu_topo->cluster_sibling);

		cpumask_clear(&cpu_topo->core_sibling);
		cpumask_set_cpu(cpu, &cpu_topo->core_sibling);
		cpumask_clear(&cpu_topo->thread_sibling);
		cpumask_set_cpu(cpu, &cpu_topo->thread_sibling);
	}
}

void __init init_cpu_topology(void)
{
	reset_cpu_topology();

	/*
	 * Discard anything that was parsed if we hit an error so we
	 * don't use partial information.
	 */
	if (of_have_populated_dt() && parse_dt_topology())
		reset_cpu_topology();
	else
		set_sched_topology(arm64_topology);

	init_sched_energy_costs();
}

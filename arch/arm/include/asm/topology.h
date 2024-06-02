#ifndef _ASM_ARM_TOPOLOGY_H
#define _ASM_ARM_TOPOLOGY_H

#ifdef CONFIG_ARM_CPU_TOPOLOGY

#include <linux/cpufreq.h>
#include <linux/cpumask.h>

struct cputopo_arm {
	int thread_id;
	int core_id;
	int cluster_id;
	cpumask_t thread_sibling;
	cpumask_t core_sibling;
};

extern struct cputopo_arm cpu_topology[NR_CPUS];

#define topology_physical_package_id(cpu)	(cpu_topology[cpu].cluster_id)
#define topology_core_id(cpu)		(cpu_topology[cpu].core_id)
#define topology_core_cpumask(cpu)	(&cpu_topology[cpu].core_sibling)
#define topology_sibling_cpumask(cpu)	(&cpu_topology[cpu].thread_sibling)

void init_cpu_topology(void);
void store_cpu_topology(unsigned int cpuid);
const struct cpumask *cpu_coregroup_mask(int cpu);

#ifdef CONFIG_CPU_FREQ
#define arch_scale_freq_capacity cpufreq_get_freq_scale
extern unsigned long cpufreq_get_freq_scale(int cpu);
extern unsigned long cpufreq_get_pressure(int cpu);

#endif
#define arch_scale_cpu_capacity scale_cpu_capacity
extern unsigned long scale_cpu_capacity(int cpu);

#define arch_get_hw_load_avg hw_load_avg_by_cpu
extern u64 hw_load_avg_by_cpu(int cpu);

#define arch_scale_freq_ref cpufreq_get_freq_ref
extern unsigned long cpufreq_get_freq_ref(int cpu);
#define arch_set_freq_ref cpufreq_set_freq_ref
extern void cpufreq_set_freq_ref(const struct cpumask *cpus,
			       unsigned long max_freq);

/* Replace task scheduler's default HW pressure API */
#define arch_scale_hw_pressure topology_get_hw_pressure
extern unsigned long topology_get_hw_pressure(int cpu);

#define arch_update_hw_pressure	topology_update_hw_pressure
extern void topology_update_hw_pressure(const struct cpumask *cpus,
				      unsigned long capped_freq);

#else

static inline void init_cpu_topology(void) { }
static inline void store_cpu_topology(unsigned int cpuid) { }

#endif

#include <asm-generic/topology.h>

#endif /* _ASM_ARM_TOPOLOGY_H */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <asm/msr.h>
#include <asm/vmx.h>
#include <asm/cpufeature.h>

#define MYPAGE_SIZE 4096
#define X86_CR4_VMXE_BIT 13
#define X86_CR4_VMXE _BITUL(X86_CR4_VMXE_BIT)
#define FEATURE_CONTROL_VMXON_ENABLED_OUTSIDE_SMX (1 << 2)
#define FEATURE_CONTROL_LOCKED (1 << 0)
#define MSR_IA32_FEATURE_CONTROL 0x0000003a
#define MSR_IA32_VMX_BASIC 0x00000480
#define MSR_IA32_VMX_CR0_FIXED0 0x00000486
#define MSR_IA32_VMX_CR0_FIXED1 0x00000487
#define MSR_IA32_VMX_CR4_FIXED0 0x00000488
#define MSR_IA32_VMX_CR4_FIXED1 0x00000489

MODULE_LICENSE("GPL");
MODULE_AUTHOR("misile00");
MODULE_DESCRIPTION("VMSL hypervisor");

#define EAX_EDX_VAL(val, low, high) ((low) | ((uint64_t)(high) << 32))
#define EAX_EDX_RET(val, low, high) "=a" (low), "=d" (high)

static inline int _vmxon(uint64_t phys) {
    uint8_t ret;
    asm volatile("vmxon %[pa]; setna %[ret]"
                 : [ret] "=rm"(ret)
                 : [pa] "m"(phys)
                 : "cc", "memory");
    return ret;
}

static inline unsigned long long notrace __rdmsr1(unsigned int msr) {
    DECLARE_ARGS(val, low, high);
    asm volatile(
        "1: rdmsr\n"
        "2:\n"
        "   .section .fixup,\"ax\"\n"
        "3: jmp 2b\n"
        "   .previous\n"
        "   .section __ex_table,\"a\"\n"
        "   .align 8\n"
        "   .quad 1b, 3b\n"
        "   .previous"
        : EAX_EDX_RET(val, low, high)
        : "c"(msr));
    return EAX_EDX_VAL(val, low, high);
}

static inline uint32_t vmcs_revision_id(void) {
    return __rdmsr1(MSR_IA32_VMX_BASIC);
}

static bool enable_vmx_outside_smx(void) {
    uint64_t feature_control;
    uint64_t required = FEATURE_CONTROL_VMXON_ENABLED_OUTSIDE_SMX | FEATURE_CONTROL_LOCKED;

    feature_control = __rdmsr1(MSR_IA32_FEATURE_CONTROL);

    if ((feature_control & required) != required) {
        wrmsr(MSR_IA32_FEATURE_CONTROL, feature_control | required, 0);
    }

    return true;
}

static bool configure_cr0_cr4(void) {
    unsigned long cr0, cr4;

    asm volatile("mov %%cr0, %0" : "=r"(cr0) : : "memory");
    cr0 &= __rdmsr1(MSR_IA32_VMX_CR0_FIXED1);
    cr0 |= __rdmsr1(MSR_IA32_VMX_CR0_FIXED0);
    asm volatile("mov %0, %%cr0" : : "r"(cr0) : "memory");

    asm volatile("mov %%cr4, %0" : "=r"(cr4) : : "memory");
    cr4 &= __rdmsr1(MSR_IA32_VMX_CR4_FIXED1);
    cr4 |= __rdmsr1(MSR_IA32_VMX_CR4_FIXED0);
    asm volatile("mov %0, %%cr4" : : "r"(cr4) : "memory");

    return true;
}

static bool allocate_vmxon_region(uint32_t **vmxon_region) {
    *vmxon_region = kzalloc(MYPAGE_SIZE, GFP_KERNEL);
    return (*vmxon_region != NULL);
}

static bool initialize_vmxon_region(uint32_t *vmxon_region) {
    *vmxon_region = vmcs_revision_id();
    return true;
}

static bool enter_vmx_mode(uint64_t vmxon_phy_region) {
    return _vmxon(vmxon_phy_region) == 0;
}

static bool vmx_support(void) {
    int get_vmx_support, vmx_bit;

    asm volatile("mov $1, %rax");
    asm volatile("cpuid");
    asm volatile("mov %%ecx, %0\n\t" : "=r"(get_vmx_support));
    vmx_bit = (get_vmx_support >> 5) & 1;

    return (vmx_bit == 1);
}

static int __init start_init(void) {
    uint32_t *vmxon_region;
    uint64_t vmxon_phy_region;

    if (!vmx_support()) {
        printk(KERN_INFO "VMX is not supported. Exiting\n");
        return -ENODEV;
    }

    if (!enable_vmx_outside_smx() || !configure_cr0_cr4()) {
        printk(KERN_INFO "Failed to configure VMX settings. Exiting\n");
        return -EINVAL;
    }

    if (!allocate_vmxon_region(&vmxon_region) || !initialize_vmxon_region(vmxon_region)) {
        printk(KERN_INFO "Error allocating or initializing VMXON region. Exiting\n");
        return -ENOMEM;
    }

    vmxon_phy_region = __pa(vmxon_region);

    if (!enter_vmx_mode(vmxon_phy_region)) {
        printk(KERN_INFO "Failed to enter VMX mode. Exiting\n");
        kfree(vmxon_region);
        return -EINVAL;
    }

    printk(KERN_INFO "VMSL hypervisor started successfully\n");

    return 0;
}

static void __exit end_exit(void) {
    printk(KERN_INFO "VMSL hypervisor stopped\n");
}

module_init(start_init);
module_exit(end_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("misile00");
MODULE_DESCRIPTION("VMSL hypervisor");

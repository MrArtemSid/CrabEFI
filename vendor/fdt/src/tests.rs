// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at https://mozilla.org/MPL/2.0/.

extern crate std;

use crate::{node::RawReg, *};

static TEST: &[u8] = include_bytes!("../dtb/test.dtb");
static ISSUE_3: &[u8] = include_bytes!("../dtb/issue-3.dtb");
static SIFIVE: &[u8] = include_bytes!("../dtb/sifive.dtb");

#[test]
fn returns_fdt() {
    assert!(Fdt::new(TEST).is_ok());
}

#[test]
fn finds_root_node() {
    let fdt = Fdt::new(TEST).unwrap();
    assert!(fdt.find_node("/").is_some(), "couldn't find root node");
}

#[test]
fn finds_root_node_properties() {
    let fdt = Fdt::new(TEST).unwrap();
    let prop = fdt
        .find_node("/")
        .unwrap()
        .properties()
        .any(|p| p.name == "compatible" && p.value == b"riscv-virtio\0");

    assert!(prop);
}

#[test]
fn finds_child_of_root_node() {
    let fdt = Fdt::new(TEST).unwrap();
    assert!(fdt.find_node("/cpus").is_some(), "couldn't find cpus node");
}

#[test]
fn correct_flash_regions() {
    let fdt = Fdt::new(TEST).unwrap();
    let regions = fdt.find_node("/soc/flash").unwrap().reg().unwrap().collect::<std::vec::Vec<_>>();

    assert_eq!(
        regions,
        &[
            MemoryRegion { starting_address: 0x20000000 as *const u8, size: Some(0x2000000) },
            MemoryRegion { starting_address: 0x22000000 as *const u8, size: Some(0x2000000) }
        ]
    );
}

#[test]
fn finds_with_addr() {
    let fdt = Fdt::new(TEST).unwrap();
    assert_eq!(fdt.find_node("/soc/virtio_mmio@10004000").unwrap().name, "virtio_mmio@10004000");
}

#[test]
fn compatibles() {
    let fdt = Fdt::new(TEST).unwrap();
    let res = fdt
        .find_node("/soc/test")
        .unwrap()
        .compatible()
        .unwrap()
        .all()
        .all(|s| ["sifive,test1", "sifive,test0", "syscon"].contains(&s));

    assert!(res);
}

#[test]
fn parent_cell_sizes() {
    let fdt = Fdt::new(TEST).unwrap();
    let regions = fdt.find_node("/memory").unwrap().reg().unwrap().collect::<std::vec::Vec<_>>();

    assert_eq!(
        regions,
        &[MemoryRegion { starting_address: 0x80000000 as *const u8, size: Some(0x20000000) }]
    );
}

#[test]
fn no_properties() {
    let fdt = Fdt::new(TEST).unwrap();
    let regions = fdt.find_node("/emptyproptest").unwrap();
    assert_eq!(regions.properties().count(), 0);
}

#[test]
fn finds_all_nodes() {
    let fdt = Fdt::new(TEST).unwrap();

    let mut all_nodes: std::vec::Vec<_> = fdt.all_nodes().map(|n| n.name).collect();
    all_nodes.sort_unstable();

    assert_eq!(
        all_nodes,
        &[
            "/",
            "chosen",
            "clint@2000000",
            "cluster0",
            "core0",
            "cpu-map",
            "cpu@0",
            "cpus",
            "emptyproptest",
            "flash@20000000",
            "interrupt-controller",
            "memory@80000000",
            "pci@30000000",
            "plic@c000000",
            "poweroff",
            "reboot",
            "rtc@101000",
            "soc",
            "test@100000",
            "uart@10000000",
            "virtio_mmio@10001000",
            "virtio_mmio@10002000",
            "virtio_mmio@10003000",
            "virtio_mmio@10004000",
            "virtio_mmio@10005000",
            "virtio_mmio@10006000",
            "virtio_mmio@10007000",
            "virtio_mmio@10008000"
        ]
    )
}

#[test]
fn required_nodes() {
    let fdt = Fdt::new(TEST).unwrap();
    fdt.cpus().next().unwrap();
    fdt.memory();
    fdt.chosen();
}

#[test]
fn doesnt_exist() {
    let fdt = Fdt::new(TEST).unwrap();
    assert!(fdt.find_node("/this/doesnt/exist").is_none());
}

#[test]
fn raw_reg() {
    let fdt = Fdt::new(TEST).unwrap();
    let regions =
        fdt.find_node("/soc/flash").unwrap().raw_reg().unwrap().collect::<std::vec::Vec<_>>();

    assert_eq!(
        regions,
        &[
            RawReg { address: &0x20000000u64.to_be_bytes(), size: &0x2000000u64.to_be_bytes() },
            RawReg { address: &0x22000000u64.to_be_bytes(), size: &0x2000000u64.to_be_bytes() }
        ]
    );
}

#[test]
fn issue_3() {
    let fdt = Fdt::new(ISSUE_3).unwrap();
    fdt.find_all_nodes("uart").for_each(|n| std::println!("{:?}", n));
}

#[test]
fn issue_4() {
    let fdt = Fdt::new(ISSUE_3).unwrap();
    fdt.all_nodes().for_each(|n| std::println!("{:?}", n));
}

#[test]
fn cpus() {
    let fdt = Fdt::new(TEST).unwrap();
    for cpu in fdt.cpus() {
        cpu.ids().all().for_each(|n| std::println!("{:?}", n));
    }
}

#[test]
fn invalid_node() {
    let fdt = Fdt::new(TEST).unwrap();
    assert!(fdt.find_node("this/is/an invalid node///////////").is_none());
}

#[test]
fn aliases() {
    let fdt = Fdt::new(SIFIVE).unwrap();
    let aliases = fdt.aliases().unwrap();
    for (_, node_path) in aliases.all() {
        assert!(fdt.find_node(node_path).is_some(), "path: {:?}", node_path);
    }
}

#[test]
fn node_property_str_value() {
    let fdt = Fdt::new(TEST).unwrap();
    let cpu0 = fdt.find_node("/cpus/cpu@0").unwrap();
    assert_eq!(cpu0.property("riscv,isa").unwrap().as_str().unwrap(), "rv64imafdcsu");
}

#[test]
fn model_value() {
    let fdt = Fdt::new(TEST).unwrap();
    assert_eq!(fdt.root().model(), "riscv-virtio,qemu");
}

#[test]
fn memory_node() {
    let fdt = Fdt::new(TEST).unwrap();
    assert_eq!(fdt.memory().regions().count(), 1);
}

#[test]
fn interrupt_cells() {
    let fdt = Fdt::new(TEST).unwrap();
    let uart = fdt.find_node("/soc/uart").unwrap();
    std::println!("{:?}", uart.parent_interrupt_cells());
    assert_eq!(uart.interrupts().unwrap().collect::<std::vec::Vec<_>>(), std::vec![0xA]);
}

// Regression tests for FDT_NOP tokens between properties. coreboot patches the
// handoff devicetree with libfdt, which replaces removed data with FDT_NOP
// tokens, including between properties of the /cpus/cpu@N nodes. See
// dts/coreboot-scr9-handoff.dts.
static COREBOOT_SCR9_HANDOFF: &[u8] = include_bytes!("../dtb/coreboot-scr9-handoff.dtb");

#[test]
fn coreboot_handoff_fdt_parses() {
    // skip_current_node previously panicked with
    // `assertion left == right failed: bad node (left: 3, right: 2)`
    // when skipping over a node whose properties were interleaved with NOPs.
    assert!(Fdt::new(COREBOOT_SCR9_HANDOFF).is_ok());
}

#[test]
fn coreboot_handoff_cpu_properties_across_nops() {
    let fdt = Fdt::new(COREBOOT_SCR9_HANDOFF).unwrap();
    let cpu0 = fdt.find_node("/cpus/cpu@0").unwrap();
    assert_eq!(cpu0.property("device_type").unwrap().as_str().unwrap(), "cpu");
    assert_eq!(
        cpu0.property("riscv,isa").unwrap().as_str().unwrap(),
        "rv64imafdc_zicond_zba_zbb_zbc_zbs_sscofpmf"
    );
    assert_eq!(cpu0.property("mmu-type").unwrap().as_str().unwrap(), "riscv,sv39");
}

#[test]
fn coreboot_handoff_cpu_children_across_nops() {
    let fdt = Fdt::new(COREBOOT_SCR9_HANDOFF).unwrap();
    let cpu0 = fdt.find_node("/cpus/cpu@0").unwrap();
    let names: std::vec::Vec<_> = cpu0.children().map(|c| c.name).collect();
    assert_eq!(names, std::vec!["l2-cache", "interrupt-controller"]);
}

#[test]
fn coreboot_handoff_all_cpus_reachable() {
    let fdt = Fdt::new(COREBOOT_SCR9_HANDOFF).unwrap();
    let cpus: std::vec::Vec<_> = fdt
        .all_nodes()
        .filter(|n| n.name.starts_with("cpu@"))
        .map(|n| n.name)
        .collect();
    assert_eq!(cpus.len(), 8, "expected 8 cpu@N nodes, got: {:?}", cpus);

    // all_nodes previously truncated the iterator after the first node with
    // interleaved NOPs; make sure deep nodes are still reachable via find_node.
    for path in ["/soc/serial@1e140000", "/memory@400000000", "/cpus/cpu@7/l2-cache"] {
        assert!(fdt.find_node(path).is_some(), "couldn't find {}", path);
    }
}

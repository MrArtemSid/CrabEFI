//! Virtio-mmio block devices (virtio-blk) behind a polling transport.
//!
//! The transport is discovered from the platform FDT (`compatible =
//! "virtio,mmio"` nodes, collected by [`crate::fdt`] into
//! [`crate::fdt::PlatformInfo`]); QEMU-style machines attach virtio-mmio
//! transports at fixed addresses even when the board device tree carries no
//! nodes for them.
//!
//! Requests are driven by polling the used ring — CrabEFI does no
//! interrupt-driven I/O — so the device tree does not even need to wire an
//! interrupt for the firmware to read the disk. The kernel still wants the
//! IRQ to drive the same device after handoff.
//!
//! Only reads are implemented; writes stay [`BlockError::WriteProtected`].

use crate::cell::Local;
use crate::drivers::block::{BlockDevice, BlockDeviceInfo, BlockError};
use crate::{barrier, handoff, time};

/// Virtio-mmio transports registered at most.
const MAX_DEVICES: usize = 4;
/// Split-virtqueue depth (power of two; every request is one 3-descriptor
/// chain submitted at a time, so a shallow queue is enough).
const QUEUE_DEPTH: usize = 16;
/// Sectors per request at most (bounded by the descriptor 32-bit length and
/// by how much the FAT layer asks for at once anyway).
const MAX_SECTORS_PER_REQUEST: u32 = 128;
const SECTOR_SIZE: u64 = 512;

// virtio-mmio v2 register offsets
const REG_MAGIC: usize = 0x000;
const REG_VERSION: usize = 0x004;
const REG_DEVICE_ID: usize = 0x008;
const REG_DEV_FEATURES: usize = 0x010;
const REG_DEV_FEATURES_SEL: usize = 0x014;
const REG_DRV_FEATURES: usize = 0x020;
const REG_DRV_FEATURES_SEL: usize = 0x024;
const REG_QUEUE_SEL: usize = 0x030;
const REG_QUEUE_NUM_MAX: usize = 0x034;
const REG_QUEUE_NUM: usize = 0x038;
const REG_QUEUE_READY: usize = 0x044;
const REG_QUEUE_NOTIFY: usize = 0x050;
const REG_INTERRUPT_STATUS: usize = 0x060;
const REG_INTERRUPT_ACK: usize = 0x064;
const REG_STATUS: usize = 0x070;
const REG_QUEUE_DESC: usize = 0x080; // low at +0, high at +4; same for avail/used
const REG_QUEUE_AVAIL: usize = 0x090;
const REG_QUEUE_USED: usize = 0x0a0;
const REG_CONFIG: usize = 0x100;

const VIRTIO_MAGIC: u32 = 0x7472_6976; // "virt"
const VIRTIO_MMIO_VERSION_2: u32 = 2;
const VIRTIO_DEVICE_ID_BLK: u32 = 2;

const STATUS_ACKNOWLEDGE: u32 = 1;
const STATUS_DRIVER: u32 = 2;
const STATUS_DRIVER_OK: u32 = 4;
const STATUS_FEATURES_OK: u32 = 8;

/// `VIRTIO_F_VERSION_1` is feature bit 32: selector word 1, bit 0.
const DRV_FEATURES_HI_VERSION_1: u32 = 1;

// Descriptor flags
const DESC_F_NEXT: u16 = 1;
const DESC_F_WRITE: u16 = 2;

// virtio-blk request status bytes
const BLK_STATUS_OK: u8 = 0;

// virtio-blk request types
const BLK_T_IN: u32 = 0;

/// Request timeout in seconds.
const REQUEST_TIMEOUT_SECS: u64 = 5;

#[repr(C)]
#[derive(Clone, Copy)]
struct Desc {
    addr: u64,
    len: u32,
    flags: u16,
    next: u16,
}

const DESC_ZERO: Desc = Desc { addr: 0, len: 0, flags: 0, next: 0 };

#[repr(C)]
struct AvailRing {
    flags: u16,
    idx: u16,
    ring: [u16; QUEUE_DEPTH],
}

#[repr(C)]
struct UsedElem {
    id: u32,
    len: u32,
}

#[repr(C)]
struct UsedRing {
    flags: u16,
    idx: u16,
    ring: [UsedElem; QUEUE_DEPTH],
}

const USED_ELEM_ZERO: UsedElem = UsedElem { id: 0, len: 0 };

/// Queue memory for one device. The virtio-mmio transport requires the
/// descriptor table, available ring and used ring to start on separate
/// 4096-byte pages, so each block is padded out to a full page.
#[repr(C, align(4096))]
struct QueueStorage {
    desc: [Desc; QUEUE_DEPTH],
    _pad0: [u8; 4096 - QUEUE_DEPTH * core::mem::size_of::<Desc>()],
    avail: AvailRing,
    _pad1: [u8; 4096 - 4 - 2 * QUEUE_DEPTH],
    used: UsedRing,
    _pad2: [u8; 4096 - 4 - 8 * QUEUE_DEPTH],
}

impl QueueStorage {
    const ZERO: Self = Self {
        desc: [DESC_ZERO; QUEUE_DEPTH],
        _pad0: [0; 4096 - QUEUE_DEPTH * core::mem::size_of::<Desc>()],
        avail: AvailRing { flags: 0, idx: 0, ring: [0; QUEUE_DEPTH] },
        _pad1: [0; 4096 - 4 - 2 * QUEUE_DEPTH],
        used: UsedRing { flags: 0, idx: 0, ring: [USED_ELEM_ZERO; QUEUE_DEPTH] },
        _pad2: [0; 4096 - 4 - 8 * QUEUE_DEPTH],
    };
}

/// virtio-blk request header (device-readable part of the chain).
#[repr(C)]
struct BlkHeader {
    kind: u32,
    reserved: u32,
    sector: u64,
}

/// A virtio-blk device on one virtio-mmio transport.
///
/// Queue and request buffers live in the module's static storage pool, so
/// their physical addresses stay valid for the firmware's whole lifetime
/// (no paging is enabled: virtual addresses are physical addresses).
pub struct VirtioBlk {
    base: usize,
    /// 512-byte sector count from the device config space.
    capacity: u64,
    /// Raw pointer into [`STORAGES`]; never aliased because the owning
    /// registry slot is exclusively borrowed for every request.
    storage: *mut QueueStorage,
    avail_idx: u16,
    last_used: u16,
    header: BlkHeader,
    status: u8,
}

#[inline]
fn reg_read(base: usize, offset: usize) -> u32 {
    unsafe { core::ptr::read_volatile((base + offset) as *const u32) }
}

#[inline]
fn reg_write(base: usize, offset: usize, value: u32) {
    unsafe { core::ptr::write_volatile((base + offset) as *mut u32, value) }
}

impl VirtioBlk {
    /// Probe one transport and bring the block device up.
    fn probe(base: usize, storage: *mut QueueStorage) -> Result<Self, &'static str> {
        if reg_read(base, REG_MAGIC) != VIRTIO_MAGIC {
            return Err("bad virtio magic");
        }
        if reg_read(base, REG_VERSION) != VIRTIO_MMIO_VERSION_2 {
            return Err("unsupported transport version");
        }
        if reg_read(base, REG_DEVICE_ID) != VIRTIO_DEVICE_ID_BLK {
            // 0 = transport without a device plugged in; anything else is
            // not a disk and not this driver's business.
            return Err("no block device on transport");
        }

        // Reset, then acknowledge and claim the device.
        reg_write(base, REG_STATUS, 0);
        reg_write(base, REG_STATUS, STATUS_ACKNOWLEDGE | STATUS_DRIVER);

        // Negotiate features: accept VIRTIO_F_VERSION_1 only. Declining
        // everything else (RO, BLK_SIZE, EVENT_IDX, ...) keeps the rings in
        // their plain split layout with 512-byte implicit sectors.
        reg_write(base, REG_DEV_FEATURES_SEL, 0);
        let _offered_lo = reg_read(base, REG_DEV_FEATURES);
        reg_write(base, REG_DEV_FEATURES_SEL, 1);
        let offered_hi = reg_read(base, REG_DEV_FEATURES);
        if offered_hi & DRV_FEATURES_HI_VERSION_1 == 0 {
            return Err("device does not offer VIRTIO_F_VERSION_1");
        }
        reg_write(base, REG_DRV_FEATURES_SEL, 0);
        reg_write(base, REG_DRV_FEATURES, 0);
        reg_write(base, REG_DRV_FEATURES_SEL, 1);
        reg_write(base, REG_DRV_FEATURES, DRV_FEATURES_HI_VERSION_1);
        reg_write(
            base,
            REG_STATUS,
            STATUS_ACKNOWLEDGE | STATUS_DRIVER | STATUS_FEATURES_OK,
        );
        if reg_read(base, REG_STATUS) & STATUS_FEATURES_OK == 0 {
            return Err("feature negotiation rejected");
        }

        // Set up queue 0 (the request queue).
        reg_write(base, REG_QUEUE_SEL, 0);
        let num_max = reg_read(base, REG_QUEUE_NUM_MAX) as usize;
        if num_max < QUEUE_DEPTH {
            return Err("queue too small");
        }
        reg_write(base, REG_QUEUE_NUM, QUEUE_DEPTH as u32);
        let desc = unsafe { core::ptr::addr_of!((*storage).desc) as u64 };
        let avail = unsafe { core::ptr::addr_of!((*storage).avail) as u64 };
        let used = unsafe { core::ptr::addr_of!((*storage).used) as u64 };
        reg_write(base, REG_QUEUE_DESC, desc as u32);
        reg_write(base, REG_QUEUE_DESC + 4, (desc >> 32) as u32);
        reg_write(base, REG_QUEUE_AVAIL, avail as u32);
        reg_write(base, REG_QUEUE_AVAIL + 4, (avail >> 32) as u32);
        reg_write(base, REG_QUEUE_USED, used as u32);
        reg_write(base, REG_QUEUE_USED + 4, (used >> 32) as u32);
        reg_write(base, REG_QUEUE_READY, 1);
        if reg_read(base, REG_QUEUE_READY) != 1 {
            return Err("queue did not become ready");
        }

        reg_write(
            base,
            REG_STATUS,
            STATUS_ACKNOWLEDGE | STATUS_DRIVER | STATUS_FEATURES_OK | STATUS_DRIVER_OK,
        );

        // Device config: capacity is a 64-bit little-endian sector count.
        let capacity_lo = reg_read(base, REG_CONFIG);
        let capacity_hi = reg_read(base, REG_CONFIG + 4);
        let capacity = (capacity_lo as u64) | ((capacity_hi as u64) << 32);

        Ok(Self {
            base,
            capacity,
            storage,
            avail_idx: 0,
            last_used: 0,
            header: BlkHeader { kind: 0, reserved: 0, sector: 0 },
            status: 0,
        })
    }

    /// Issue one read request and poll for its completion.
    fn read_request(&mut self, sector: u64, count: u32, buffer: &mut [u8]) -> Result<(), BlockError> {
        let data_len = count as usize * SECTOR_SIZE as usize;
        self.header = BlkHeader { kind: BLK_T_IN, reserved: 0, sector };
        self.status = 0xff;

        let header_addr = core::ptr::addr_of!(self.header) as u64;
        let status_addr = core::ptr::addr_of!(self.status) as u64;
        let data_addr = buffer.as_ptr() as u64;

        // SAFETY: the registry lends this device out exclusively; the queue
        // memory belongs to this device alone.
        let storage = unsafe { &mut *self.storage };
        storage.desc[0] = Desc {
            addr: header_addr,
            len: core::mem::size_of::<BlkHeader>() as u32,
            flags: DESC_F_NEXT,
            next: 1,
        };
        storage.desc[1] = Desc {
            addr: data_addr,
            len: data_len as u32,
            flags: DESC_F_NEXT | DESC_F_WRITE,
            next: 2,
        };
        storage.desc[2] = Desc {
            addr: status_addr,
            len: 1,
            flags: DESC_F_WRITE,
            next: 0,
        };

        let ring_slot = (self.avail_idx as usize) % QUEUE_DEPTH;
        storage.avail.ring[ring_slot] = 0;
        // Publish descriptors and the ring slot before bumping avail.idx.
        barrier::publish_to_device();
        self.avail_idx = self.avail_idx.wrapping_add(1);
        unsafe { core::ptr::write_volatile(&mut storage.avail.idx, self.avail_idx) };

        // Order the published ring before the MMIO doorbell write.
        barrier::mmio_write();
        reg_write(self.base, REG_QUEUE_NOTIFY, 0);

        let freq = time::counter_frequency().max(1);
        let deadline = time::read_counter() + freq * REQUEST_TIMEOUT_SECS;
        loop {
            let used_idx = unsafe { core::ptr::read_volatile(&storage.used.idx) };
            if used_idx != self.last_used {
                self.last_used = self.last_used.wrapping_add(1);
                // Order the device's completion writes before reading them.
                barrier::consume_from_device();
                let interrupt_status = reg_read(self.base, REG_INTERRUPT_STATUS);
                if interrupt_status != 0 {
                    reg_write(self.base, REG_INTERRUPT_ACK, interrupt_status);
                }
                return if self.status == BLK_STATUS_OK {
                    Ok(())
                } else {
                    log::error!(
                        "VirtIO: read at sector {} failed with status {}",
                        sector,
                        self.status
                    );
                    Err(BlockError::DeviceError)
                };
            }
            if time::read_counter() >= deadline {
                log::error!("VirtIO: read at sector {} timed out", sector);
                return Err(BlockError::DeviceError);
            }
        }
    }
}

impl BlockDevice for VirtioBlk {
    fn info(&self) -> BlockDeviceInfo {
        BlockDeviceInfo {
            num_blocks: self.capacity,
            block_size: SECTOR_SIZE as u32,
            media_id: 1,
            removable: false,
            read_only: true,
        }
    }

    fn read_blocks(&mut self, lba: u64, count: u32, buffer: &mut [u8]) -> Result<(), BlockError> {
        self.validate_io(lba, count, buffer)?;
        let mut remaining = count;
        let mut sector = lba;
        let mut offset = 0;
        while remaining > 0 {
            let chunk = remaining.min(MAX_SECTORS_PER_REQUEST);
            let len = chunk as usize * SECTOR_SIZE as usize;
            let request_buf = &mut buffer[offset..offset + len];
            self.read_request(sector, chunk, request_buf)?;
            sector += chunk as u64;
            offset += len;
            remaining -= chunk;
        }
        Ok(())
    }

    fn name(&self) -> &str {
        "VirtIO disk"
    }
}

// ============================================================================
// Device registry
// ============================================================================

struct Registry {
    devices: [Option<VirtioBlk>; MAX_DEVICES],
}

static REGISTRY: Local<Registry> = Local::new(Registry {
    devices: [None, None, None, None],
});

static STORAGES: Local<[QueueStorage; MAX_DEVICES]> =
    Local::new([QueueStorage::ZERO; MAX_DEVICES]);

/// Probe every virtio-mmio transport the platform FDT describes and
/// register the block devices found on them.
pub fn init() {
    let bases: heapless::Vec<u64, 8> = handoff::with_mut(|handoff| {
        handoff
            .fdt_info
            .virtio_mmio_regions()
            .iter()
            .copied()
            .collect()
    });

    if bases.is_empty() {
        return;
    }

    for base in bases {
        register(base);
    }
}

fn register(base: u64) {
    let slot = REGISTRY.with_mut(|registry| registry.devices.iter().position(Option::is_none));
    let Some(slot) = slot else {
        log::warn!("VirtIO: transport at {:#x} skipped: registry full", base);
        return;
    };

    let result = STORAGES.with_mut(|storages| {
        // SAFETY: the storage slot is handed to the device in the registry
        // slot with the same index and is never accessed through any other
        // path; the registry serializes all access for the firmware's
        // lifetime.
        let storage = unsafe { core::ptr::addr_of_mut!(storages[slot]) };
        VirtioBlk::probe(base as usize, storage)
    });

    REGISTRY.with_mut(|registry| match result {
        Ok(device) => {
            log::info!(
                "VirtIO: block device at {:#x}: {} sectors",
                base,
                device.capacity
            );
            registry.devices[slot] = Some(device);
        }
        Err(reason) => log::debug!("VirtIO: transport at {:#x}: {}", base, reason),
    });
}

/// Number of registered virtio block devices.
pub fn device_count() -> usize {
    REGISTRY.with_mut(|registry| registry.devices.iter().filter(|d| d.is_some()).count())
}

/// MMIO base of the transport behind registered device `index`.
pub fn transport_base(index: usize) -> Option<u64> {
    REGISTRY.with_mut(|registry| {
        registry
            .devices
            .get(index)
            .and_then(Option::as_ref)
            .map(|device| device.base as u64)
    })
}

/// Lend registered device `index` to `f`, if present.
pub fn with_device<R>(index: usize, f: impl FnOnce(&mut VirtioBlk) -> R) -> Option<R> {
    REGISTRY.with_mut(|registry| {
        registry
            .devices
            .get_mut(index)
            .and_then(Option::as_mut)
            .map(f)
    })
}

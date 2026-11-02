// SPDX-License-Identifier: MIT

#[cfg(not(target_os = "linux"))]
fn main() {
    eprintln!("ucan-mit-motor-simulator requires Linux SocketCAN");
    std::process::exit(2);
}

#[cfg(target_os = "linux")]
mod linux {
    use libc::{c_int, c_void, pollfd, sockaddr, sockaddr_can, socklen_t};
    use std::env;
    use std::io;
    use std::mem;
    use std::os::fd::RawFd;
    use std::sync::atomic::{AtomicBool, Ordering};
    use std::sync::Arc;
    use std::thread;
    use std::time::Duration;

    const PF_CAN: c_int = 29;
    const AF_CAN: c_int = PF_CAN;
    const CAN_RAW: c_int = 1;
    const SOL_CAN_RAW: c_int = 101;
    const CAN_RAW_FILTER: c_int = 1;
    const CAN_RAW_FD_FRAMES: c_int = 5;
    const CAN_EFF_FLAG: u32 = 0x8000_0000;
    const CAN_EFF_MASK: u32 = 0x1FFF_FFFF;
    const CAN_SFF_MASK: u32 = 0x0000_07FF;
    const CANFD_BRS: u8 = 0x01;
    const POLLIN: i16 = 0x0001;

    #[repr(C)]
    #[derive(Clone, Copy)]
    struct CanFdFrame {
        can_id: u32,
        len: u8,
        flags: u8,
        reserved0: u8,
        reserved1: u8,
        data: [u8; 64],
    }

    #[repr(C)]
    struct CanFilter {
        can_id: u32,
        can_mask: u32,
    }

    #[derive(Clone, Copy)]
    struct Limits {
        p_min: f32,
        p_max: f32,
        v_min: f32,
        v_max: f32,
        kp_min: f32,
        kp_max: f32,
        kd_min: f32,
        kd_max: f32,
        t_min: f32,
        t_max: f32,
    }

    #[derive(Clone, Copy, Default)]
    struct Command {
        position: f32,
        velocity: f32,
        kp: f32,
        kd: f32,
        torque: f32,
    }

    struct Config {
        interface: String,
        command_id: u32,
        feedback_id: u32,
        node_id: u8,
        extended: bool,
        brs: bool,
        start_enabled: bool,
        response: f32,
        reply_delay_us: u64,
        drop_every: u64,
        max_frames: u64,
        idle_timeout_ms: i32,
        limits: Limits,
    }

    impl Default for Config {
        fn default() -> Self {
            Self {
                interface: "can0".into(),
                command_id: 0x001,
                feedback_id: 0x101,
                node_id: 1,
                extended: false,
                brs: true,
                start_enabled: true,
                response: 0.25,
                reply_delay_us: 0,
                drop_every: 0,
                max_frames: 0,
                idle_timeout_ms: 100,
                limits: Limits {
                    p_min: -12.5,
                    p_max: 12.5,
                    v_min: -45.0,
                    v_max: 45.0,
                    kp_min: 0.0,
                    kp_max: 500.0,
                    kd_min: 0.0,
                    kd_max: 5.0,
                    t_min: -18.0,
                    t_max: 18.0,
                },
            }
        }
    }

    fn usage(program: &str) {
        println!("Usage: {program} [options]");
        println!("  --interface can0 --node 1 --command-id 0x1 --feedback-id 0x101");
        println!("  [--extended-id] [--no-brs] [--start-disabled]");
        println!("  [--response 0.25] [--reply-delay-us N] [--drop-every N]");
        println!("  [--max-frames N] [--idle-timeout-ms N]");
        println!("  [--p-min F --p-max F --v-min F --v-max F]");
        println!("  [--kp-min F --kp-max F --kd-min F --kd-max F]");
        println!("  [--torque-min F --torque-max F]");
        println!("The CAN interface must already be configured and UP in CAN FD mode.");
    }

    fn take_value<'a>(args: &'a [String], index: &mut usize) -> Result<&'a str, String> {
        *index += 1;
        args.get(*index)
            .map(String::as_str)
            .ok_or_else(|| format!("{} requires a value", args[*index - 1]))
    }

    fn parse_u32(value: &str) -> Result<u32, String> {
        if let Some(hex) = value
            .strip_prefix("0x")
            .or_else(|| value.strip_prefix("0X"))
        {
            u32::from_str_radix(hex, 16).map_err(|e| e.to_string())
        } else {
            value.parse::<u32>().map_err(|e| e.to_string())
        }
    }

    fn parse_args() -> Result<Option<Config>, String> {
        let args: Vec<String> = env::args().collect();
        let mut cfg = Config::default();
        let mut i = 1;
        while i < args.len() {
            let option = args[i].as_str();
            match option {
                "--interface" => cfg.interface = take_value(&args, &mut i)?.into(),
                "--node" => cfg.node_id = parse_u32(take_value(&args, &mut i)?)? as u8,
                "--command-id" => cfg.command_id = parse_u32(take_value(&args, &mut i)?)?,
                "--feedback-id" => cfg.feedback_id = parse_u32(take_value(&args, &mut i)?)?,
                "--response" => {
                    cfg.response = take_value(&args, &mut i)?
                        .parse()
                        .map_err(|e: std::num::ParseFloatError| e.to_string())?
                }
                "--reply-delay-us" => {
                    cfg.reply_delay_us = take_value(&args, &mut i)?
                        .parse()
                        .map_err(|e: std::num::ParseIntError| e.to_string())?
                }
                "--drop-every" => {
                    cfg.drop_every = take_value(&args, &mut i)?
                        .parse()
                        .map_err(|e: std::num::ParseIntError| e.to_string())?
                }
                "--max-frames" => {
                    cfg.max_frames = take_value(&args, &mut i)?
                        .parse()
                        .map_err(|e: std::num::ParseIntError| e.to_string())?
                }
                "--idle-timeout-ms" => {
                    cfg.idle_timeout_ms = take_value(&args, &mut i)?
                        .parse()
                        .map_err(|e: std::num::ParseIntError| e.to_string())?
                }
                "--p-min" => {
                    cfg.limits.p_min = take_value(&args, &mut i)?
                        .parse()
                        .map_err(|e: std::num::ParseFloatError| e.to_string())?
                }
                "--p-max" => {
                    cfg.limits.p_max = take_value(&args, &mut i)?
                        .parse()
                        .map_err(|e: std::num::ParseFloatError| e.to_string())?
                }
                "--v-min" => {
                    cfg.limits.v_min = take_value(&args, &mut i)?
                        .parse()
                        .map_err(|e: std::num::ParseFloatError| e.to_string())?
                }
                "--v-max" => {
                    cfg.limits.v_max = take_value(&args, &mut i)?
                        .parse()
                        .map_err(|e: std::num::ParseFloatError| e.to_string())?
                }
                "--kp-min" => {
                    cfg.limits.kp_min = take_value(&args, &mut i)?
                        .parse()
                        .map_err(|e: std::num::ParseFloatError| e.to_string())?
                }
                "--kp-max" => {
                    cfg.limits.kp_max = take_value(&args, &mut i)?
                        .parse()
                        .map_err(|e: std::num::ParseFloatError| e.to_string())?
                }
                "--kd-min" => {
                    cfg.limits.kd_min = take_value(&args, &mut i)?
                        .parse()
                        .map_err(|e: std::num::ParseFloatError| e.to_string())?
                }
                "--kd-max" => {
                    cfg.limits.kd_max = take_value(&args, &mut i)?
                        .parse()
                        .map_err(|e: std::num::ParseFloatError| e.to_string())?
                }
                "--torque-min" => {
                    cfg.limits.t_min = take_value(&args, &mut i)?
                        .parse()
                        .map_err(|e: std::num::ParseFloatError| e.to_string())?
                }
                "--torque-max" => {
                    cfg.limits.t_max = take_value(&args, &mut i)?
                        .parse()
                        .map_err(|e: std::num::ParseFloatError| e.to_string())?
                }
                "--extended-id" => cfg.extended = true,
                "--no-brs" => cfg.brs = false,
                "--start-disabled" => cfg.start_enabled = false,
                "--help" | "-h" => {
                    usage(&args[0]);
                    return Ok(None);
                }
                _ => return Err(format!("unknown option: {option}")),
            }
            i += 1;
        }
        let l = cfg.limits;
        if cfg.node_id == 0
            || !(0.0..=1.0).contains(&cfg.response)
            || l.p_min >= l.p_max
            || l.v_min >= l.v_max
            || l.kp_min >= l.kp_max
            || l.kd_min >= l.kd_max
            || l.t_min >= l.t_max
            || (!cfg.extended && (cfg.command_id > CAN_SFF_MASK || cfg.feedback_id > CAN_SFF_MASK))
            || (cfg.extended && (cfg.command_id > CAN_EFF_MASK || cfg.feedback_id > CAN_EFF_MASK))
        {
            return Err("invalid simulator configuration".into());
        }
        Ok(Some(cfg))
    }

    fn float_to_uint(value: f32, min: f32, max: f32, bits: u32) -> u32 {
        let value = value.clamp(min, max);
        let span = (1_u32 << bits) - 1;
        (((value - min) as f64 * span as f64 / (max - min) as f64) + 0.5) as u32
    }

    fn uint_to_float(value: u32, min: f32, max: f32, bits: u32) -> f32 {
        let span = (1_u32 << bits) - 1;
        (value as f64 * (max - min) as f64 / span as f64 + min as f64) as f32
    }

    fn detect_control(data: &[u8]) -> Option<u8> {
        if data.len() == 8 && data[..7].iter().all(|value| *value == 0xff) {
            match data[7] {
                0xfc | 0xfd | 0xfe => Some(data[7]),
                _ => None,
            }
        } else {
            None
        }
    }

    fn unpack_command(data: &[u8], l: Limits) -> Option<Command> {
        if data.len() != 8 || detect_control(data).is_some() {
            return None;
        }
        let p = ((data[0] as u32) << 8) | data[1] as u32;
        let v = ((data[2] as u32) << 4) | (data[3] >> 4) as u32;
        let kp = (((data[3] & 0x0f) as u32) << 8) | data[4] as u32;
        let kd = ((data[5] as u32) << 4) | (data[6] >> 4) as u32;
        let torque = (((data[6] & 0x0f) as u32) << 8) | data[7] as u32;
        Some(Command {
            position: uint_to_float(p, l.p_min, l.p_max, 16),
            velocity: uint_to_float(v, l.v_min, l.v_max, 12),
            kp: uint_to_float(kp, l.kp_min, l.kp_max, 12),
            kd: uint_to_float(kd, l.kd_min, l.kd_max, 12),
            torque: uint_to_float(torque, l.t_min, l.t_max, 12),
        })
    }

    fn pack_feedback(
        node: u8,
        position: f32,
        velocity: f32,
        torque: f32,
        enabled: bool,
        l: Limits,
    ) -> [u8; 8] {
        let p = float_to_uint(position, l.p_min, l.p_max, 16);
        let v = float_to_uint(velocity, l.v_min, l.v_max, 12);
        let t = float_to_uint(torque, l.t_min, l.t_max, 12);
        [
            node,
            (p >> 8) as u8,
            p as u8,
            (v >> 4) as u8,
            (((v & 0x0f) << 4) | (t >> 8)) as u8,
            t as u8,
            if enabled { 1 } else { 0 },
            25,
        ]
    }

    fn errno_result(name: &str) -> io::Error {
        io::Error::new(
            io::ErrorKind::Other,
            format!("{name}: {}", io::Error::last_os_error()),
        )
    }

    fn open_socket(cfg: &Config) -> io::Result<RawFd> {
        let fd = unsafe { libc::socket(PF_CAN, libc::SOCK_RAW, CAN_RAW) };
        if fd < 0 {
            return Err(errno_result("socket(PF_CAN)"));
        }
        let enable: c_int = 1;
        let rc = unsafe {
            libc::setsockopt(
                fd,
                SOL_CAN_RAW,
                CAN_RAW_FD_FRAMES,
                &enable as *const _ as *const c_void,
                mem::size_of_val(&enable) as socklen_t,
            )
        };
        if rc != 0 {
            unsafe { libc::close(fd) };
            return Err(errno_result("setsockopt(CAN_RAW_FD_FRAMES)"));
        }

        let id_flag = if cfg.extended { CAN_EFF_FLAG } else { 0 };
        let id_mask = if cfg.extended {
            CAN_EFF_MASK
        } else {
            CAN_SFF_MASK
        };
        let filter = CanFilter {
            can_id: cfg.command_id | id_flag,
            can_mask: id_mask | CAN_EFF_FLAG,
        };
        let rc = unsafe {
            libc::setsockopt(
                fd,
                SOL_CAN_RAW,
                CAN_RAW_FILTER,
                &filter as *const _ as *const c_void,
                mem::size_of_val(&filter) as socklen_t,
            )
        };
        if rc != 0 {
            unsafe { libc::close(fd) };
            return Err(errno_result("setsockopt(CAN_RAW_FILTER)"));
        }

        let name = std::ffi::CString::new(cfg.interface.as_str())
            .map_err(|_| io::Error::new(io::ErrorKind::InvalidInput, "invalid interface name"))?;
        let ifindex = unsafe { libc::if_nametoindex(name.as_ptr()) };
        if ifindex == 0 {
            unsafe { libc::close(fd) };
            return Err(errno_result("if_nametoindex"));
        }
        let mut address: sockaddr_can = unsafe { mem::zeroed() };
        address.can_family = AF_CAN as libc::sa_family_t;
        address.can_ifindex = ifindex as c_int;
        let rc = unsafe {
            libc::bind(
                fd,
                &address as *const _ as *const sockaddr,
                mem::size_of_val(&address) as socklen_t,
            )
        };
        if rc != 0 {
            unsafe { libc::close(fd) };
            return Err(errno_result("bind(SocketCAN)"));
        }
        Ok(fd)
    }

    pub fn run() -> Result<(), Box<dyn std::error::Error>> {
        let Some(cfg) =
            parse_args().map_err(|message| io::Error::new(io::ErrorKind::InvalidInput, message))?
        else {
            return Ok(());
        };
        let fd = open_socket(&cfg)?;
        let running = Arc::new(AtomicBool::new(true));
        let signal_flag = Arc::clone(&running);
        ctrlc::set_handler(move || signal_flag.store(false, Ordering::Relaxed))?;

        println!(
            "UCAN MIT simulator: interface={} node={} command=0x{:X} feedback=0x{:X} BRS={} enabled={}",
            cfg.interface, cfg.node_id, cfg.command_id, cfg.feedback_id, cfg.brs, cfg.start_enabled
        );
        let mut enabled = cfg.start_enabled;
        let mut position = 0.0_f32;
        let mut velocity = 0.0_f32;
        let mut torque = 0.0_f32;
        let mut received = 0_u64;
        let mut replied = 0_u64;
        let mut dropped = 0_u64;
        let mut malformed = 0_u64;
        let mut controls = 0_u64;

        while running.load(Ordering::Relaxed) && (cfg.max_frames == 0 || received < cfg.max_frames)
        {
            let mut poll_item = pollfd {
                fd,
                events: POLLIN,
                revents: 0,
            };
            let poll_rc = unsafe { libc::poll(&mut poll_item, 1, cfg.idle_timeout_ms) };
            if poll_rc < 0 {
                let error = io::Error::last_os_error();
                if error.kind() == io::ErrorKind::Interrupted {
                    continue;
                }
                unsafe { libc::close(fd) };
                return Err(error.into());
            }
            if poll_rc == 0 {
                continue;
            }
            let mut frame: CanFdFrame = unsafe { mem::zeroed() };
            let size = unsafe {
                libc::read(
                    fd,
                    &mut frame as *mut _ as *mut c_void,
                    mem::size_of::<CanFdFrame>(),
                )
            };
            if size < 0 {
                let error = io::Error::last_os_error();
                if error.kind() == io::ErrorKind::Interrupted {
                    continue;
                }
                unsafe { libc::close(fd) };
                return Err(error.into());
            }
            if size as usize != mem::size_of::<CanFdFrame>() || frame.len != 8 {
                malformed += 1;
                continue;
            }
            received += 1;
            if let Some(control) = detect_control(&frame.data[..8]) {
                controls += 1;
                match control {
                    0xfc => enabled = true,
                    0xfd => enabled = false,
                    0xfe => position = 0.0,
                    _ => {}
                }
            } else if let Some(command) = unpack_command(&frame.data[..8], cfg.limits) {
                if enabled {
                    let old_position = position;
                    position += (command.position - position) * cfg.response;
                    velocity = command.velocity + (position - old_position);
                    torque = (command.torque + command.kp * (command.position - position)
                        - command.kd * velocity)
                        .clamp(cfg.limits.t_min, cfg.limits.t_max);
                }
            } else {
                malformed += 1;
                continue;
            }

            if cfg.drop_every != 0 && received % cfg.drop_every == 0 {
                dropped += 1;
                continue;
            }
            if cfg.reply_delay_us != 0 {
                thread::sleep(Duration::from_micros(cfg.reply_delay_us));
            }
            let data = pack_feedback(cfg.node_id, position, velocity, torque, enabled, cfg.limits);
            let mut reply: CanFdFrame = unsafe { mem::zeroed() };
            reply.can_id = cfg.feedback_id | if cfg.extended { CAN_EFF_FLAG } else { 0 };
            reply.len = 8;
            reply.flags = if cfg.brs { CANFD_BRS } else { 0 };
            reply.data[..8].copy_from_slice(&data);
            let written = unsafe {
                libc::write(
                    fd,
                    &reply as *const _ as *const c_void,
                    mem::size_of::<CanFdFrame>(),
                )
            };
            if written as usize != mem::size_of::<CanFdFrame>() {
                let error = errno_result("write(CAN FD)");
                unsafe { libc::close(fd) };
                return Err(error.into());
            }
            replied += 1;
        }
        unsafe { libc::close(fd) };
        println!(
            "mit_sim_result received={} replied={} dropped={} malformed={} controls={} enabled={} position={:.5} velocity={:.5} torque={:.5}",
            received, replied, dropped, malformed, controls, enabled, position, velocity, torque
        );
        Ok(())
    }
}

#[cfg(target_os = "linux")]
fn main() {
    if let Err(error) = linux::run() {
        eprintln!("ucan-mit-motor-simulator: {error}");
        std::process::exit(1);
    }
}

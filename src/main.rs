#![no_std]
#![no_main]

use core::cell::Cell;
use cortex_m::delay::Delay;
use panic_halt as _;
use rp2040_hal::{
    clocks::{init_clocks_and_plls, Clock},
    gpio::Pins,
    pac,
    pio::PIOExt,
    watchdog::Watchdog,
    Sio,
    Timer,
};
use smart_leds::{brightness, SmartLedsWrite, RGB8};
use ws2812_pio::Ws2812;

//----------------------------------------------------------------------------------------
// 부트 섹션
//----------------------------------------------------------------------------------------
#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

//----------------------------------------------------------------------------------------
// 상수
//----------------------------------------------------------------------------------------
const XTAL_FREQ_HZ: u32 = 12_000_000;
const WIDTH: i32 = 8;
const HEIGHT: i32 = 8;
const NUM_LEDS: usize = (WIDTH * HEIGHT) as usize;

const PLAY_AREA_MIN: i32 = 1;
const PLAY_AREA_MAX: i32 = 6;
const STEP_DELAY_MS: u32 = 100; 

//----------------------------------------------------------------------------------------
// 좌표 -> 인덱스 변환 함수
//----------------------------------------------------------------------------------------
fn coordinate_to_index(x: i32, y: i32) -> usize {
    y as usize * WIDTH as usize + x as usize
}

//----------------------------------------------------------------------------------------
// 단순 LCG 난수
//----------------------------------------------------------------------------------------
pub struct Random {
    seed: u32,
}

impl Random {
    fn new(seed: u32) -> Self {
        Self { seed }
    }
    fn next(&mut self) -> u32 {
        self.seed = self
            .seed
            .wrapping_mul(1664525)
            .wrapping_add(1013904223);
        self.seed
    }
    fn range(&mut self, min: i32, max: i32) -> i32 {
        let range = (max - min + 1) as u32;
        min + (self.next() % range) as i32
    }
}

//----------------------------------------------------------------------------------------
// BresenhamPath
//----------------------------------------------------------------------------------------
struct BresenhamPath {
    x: i32,
    y: i32,
    dx: i32,
    dy: i32,
    sx: i32,
    sy: i32,
    err: i32,
    target: (i32, i32),
}

impl BresenhamPath {
    fn new(start: (i32, i32), end: (i32, i32)) -> Self {
        let dx = (end.0 - start.0).abs();
        let dy = (end.1 - start.1).abs();
        let sx = if start.0 < end.0 { 1 } else { -1 };
        let sy = if start.1 < end.1 { 1 } else { -1 };
        let err = dx - dy;

        Self {
            x: start.0,
            y: start.1,
            dx,
            dy,
            sx,
            sy,
            err,
            target: end,
        }
    }

    fn next(&mut self) -> Option<(i32, i32)> {
        if (self.x, self.y) == self.target {
            return None;
        }
        let e2 = 2 * self.err;
        if e2 > -self.dy {
            self.err -= self.dy;
            self.x += self.sx;
        }
        if e2 < self.dx {
            self.err += self.dx;
            self.y += self.sy;
        }

        // 이동 범위를 벗어나지 않도록 보정
        self.x = self.x.clamp(PLAY_AREA_MIN, PLAY_AREA_MAX);
        self.y = self.y.clamp(PLAY_AREA_MIN, PLAY_AREA_MAX);

        Some((self.x, self.y))
    }
}

//----------------------------------------------------------------------------------------
// Ball
//----------------------------------------------------------------------------------------
pub struct Ball {
    path: BresenhamPath,
    random: Random,
    pub prev_pos: (i32, i32),
    pub current_pos: (i32, i32),
}

impl Ball {
    pub fn new(timer: &Timer) -> Self {
        let mut random = Random::new(timer.get_counter().ticks() as u32);
        let start = (
            random.range(PLAY_AREA_MIN, PLAY_AREA_MAX),
            random.range(PLAY_AREA_MIN, PLAY_AREA_MAX),
        );
        let target = Self::random_opposite_edge(start, &mut random);

        Self {
            path: BresenhamPath::new(start, target),
            random,
            prev_pos: start,
            current_pos: start,
        }
    }

    // 매 프레임(루프)마다 위치 갱신
    // 리턴: false 면 유지, true면 목적지 도달 (벽과의 충돌)
    pub fn update(&mut self) -> bool {
        self.prev_pos = self.current_pos;
        if let Some(next_pos) = self.path.next() {
            self.current_pos = next_pos;
            false
        } else {
            // 목적지 도달 시, 새로운 목적지 생성
            let new_target = Self::random_opposite_edge(self.current_pos, &mut self.random);
            self.path = BresenhamPath::new(self.current_pos, new_target);

            // 한 번 더 next()로 초기 한 스텝 이동
            if let Some(next_pos) = self.path.next() {
                self.current_pos = next_pos;
            }

            true
        }
    }

    // 현재 위치 기준, 반대 쪽 벽에 가까운 곳을 다음 목적지로
    fn random_opposite_edge(pos: (i32, i32), random: &mut Random) -> (i32, i32) {
        let mut target_x = random.range(PLAY_AREA_MIN, PLAY_AREA_MAX);
        let mut target_y = random.range(PLAY_AREA_MIN, PLAY_AREA_MAX);

        // 약간의 확률로 x 또는 y 우선으로 반대 편 벽 선택
        // (pos가 절반 이하이면 상/좌측, 아니면 하/우측)
        if random.range(0, 1) == 0 {
            target_x = if pos.0 <= (PLAY_AREA_MAX / 2) {
                PLAY_AREA_MAX
            } else {
                PLAY_AREA_MIN
            };
        } else {
            target_y = if pos.1 <= (PLAY_AREA_MAX / 2) {
                PLAY_AREA_MAX
            } else {
                PLAY_AREA_MIN
            };
        }

        // clamp은 BresenhamPath 내부에서도 해주지만,
        // 혹시 모르니 여기서도 안전하게 클램핑
        (
            target_x.clamp(PLAY_AREA_MIN, PLAY_AREA_MAX),
            target_y.clamp(PLAY_AREA_MIN, PLAY_AREA_MAX),
        )
    }
}

//-----------------------------------------------
// Screen
//----------------------------------------------------------------------------------------
pub struct Screen<'a> {
    leds: &'a [Cell<RGB8>],
    ball: Ball,
    color: RGB8,
    random: Random,
}

impl<'a> Screen<'a> {
    pub fn new(leds: &'a [Cell<RGB8>], timer: &Timer) -> Self {
        let ball = Ball::new(timer);
        let mut random: Random  = Random::new(timer.get_counter_low());
        let color = RGB8::new(random.range(0, 255) as u8, random.range(0, 255) as u8, random.range(0, 255) as u8);
        Self { leds, ball, color, random }
    }

    pub fn initialize(&self) {
        for led in self.leds.iter() {
            led.set(RGB8::default());
        }
    }

    pub fn draw_border(&self, color: RGB8) {
        for x in 0..WIDTH {
            for y in 0..HEIGHT {
                if x == 0 || x == WIDTH - 1 || y == 0 || y == HEIGHT - 1 {
                    let idx = coordinate_to_index(x, y);
                    self.leds[idx].set(color);
                }
            }
        }
    }

    // 볼을 지우고 새 위치로 그린다
    // - update()로 ball 위치 갱신 후 draw()로 새 위치 표시
    pub fn update_ball(&mut self) {
        // 1) 이전 위치 지우기
        let (px, py) = self.ball.current_pos;
        let prev_idx = coordinate_to_index(px, py);
        self.leds[prev_idx].set(RGB8::default());

        // 2) 볼 위치 업데이트
        //    충돌 시 공의 색깔 랜덤하게 변경
        let is_collision = self.ball.update();
        if is_collision {
            self.color = RGB8::new(self.random.range(0, 255) as u8, self.random.range(0, 255) as u8, self.random.range(0, 255) as u8);
        }

        // 3) 새 위치 그리기
        let (cx, cy) = self.ball.current_pos;
        let curr_idx = coordinate_to_index(cx, cy);
        self.leds[curr_idx].set(self.color);
    }
}

//----------------------------------------------------------------------------------------
// 메인
//----------------------------------------------------------------------------------------
#[rp2040_hal::entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // 시계 초기화
    let clocks = init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());
    let timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);
    let pins = Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let mut ws = Ws2812::new(
        pins.gpio16.into_function(),
        &mut pio,
        sm0,
        clocks.peripheral_clock.freq(),
        timer.count_down(),
    );

    let leds: [Cell<RGB8>; NUM_LEDS] = {
        const DEFAULT: Cell<RGB8> = Cell::new(RGB8 { r: 0, g: 0, b: 0 });
        [DEFAULT; NUM_LEDS]
    };

    let mut screen = Screen::new(&leds, &timer);
    screen.initialize();
    screen.draw_border(RGB8::new(0, 0, 255));

    loop {
        screen.update_ball();

        ws.write(brightness(leds.iter().map(|c| c.get()), 32)).unwrap();

        delay.delay_ms(STEP_DELAY_MS);
    }
}
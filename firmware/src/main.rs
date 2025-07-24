#![cfg_attr(not(test), no_std)]
#![cfg_attr(not(test), no_main)]

#[cfg(target_arch = "arm")]
pub mod program {
    use embassy_executor::Spawner;
    #[embassy_executor::main]
    async fn main(spawner: Spawner) {
        firmware::program::main(spawner).await;
    }
}

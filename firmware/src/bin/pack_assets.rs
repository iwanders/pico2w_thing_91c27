use firmware::static_files;
use postcard::{from_bytes, to_vec};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let files = [
        "../../cyw43-firmware/43439A0.bin",
        "../../cyw43-firmware/43439A0_btfw.bin",
        "../../cyw43-firmware/43439A0_clm.bin",
    ];

    let mut collected = vec![];

    for f in files.iter() {
        let p = std::path::PathBuf::from(&f);
        if !p.is_file() {
            eprintln!("{p:?} is not a file");
        }
        let z = println!("processing: {p:?}");

        let file_name = p
            .file_name()
            .expect("can only handle files")
            .to_str()
            .unwrap()
            .to_string();
        let data = std::fs::read(&p)?;
        collected.push((file_name, data));
    }

    let mut flash: Vec<u8> = vec![];
    flash.resize(10_000_000, 0);
    let input = collected
        .iter()
        .map(|(x, y)| (x.as_str(), &y[..]))
        .collect::<Vec<_>>();
    let used_slice = firmware::static_files::write_static_files(&mut flash, &input)?;

    let mut file = std::fs::File::create("/tmp/static_files.bin")?;
    use std::io::Write;
    file.write_all(used_slice)?;
    Ok(())
}

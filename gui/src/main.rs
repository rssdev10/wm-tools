mod app;
mod canvas;
mod flash;
mod logging;
mod range_slider;
mod serial;
mod settings;

use app::App;
use iced::window;

fn main() -> iced::Result {
    logging::init();
    log::info!("starting dso3d12-gui");

    let icon = window::icon::from_file_data(
        include_bytes!("../img/dso3d12_ico.png"),
        None,
    )
    .ok();

    iced::application(App::new, App::update, App::view)
        .title(App::title)
        .theme(App::theme)
        .subscription(App::subscription)
        .window(window::Settings {
            icon,
            ..Default::default()
        })
        .run()
}

mod app;
mod canvas;
mod flash;
mod logging;
mod range_slider;
mod serial;
mod settings;

use app::App;

fn main() -> iced::Result {
    logging::init();
    log::info!("starting dso3d12-gui");

    iced::application(App::new, App::update, App::view)
        .title(App::title)
        .theme(App::theme)
        .subscription(App::subscription)
        .run()
}

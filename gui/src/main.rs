// `rust_i18n::i18n!` MUST be invoked at the crate root so that the generated
// helper symbol `_rust_i18n_t` is available to callers of `rust_i18n::t!`.
rust_i18n::i18n!("locales", fallback = "en");

mod app;
mod canvas;
mod flash;
mod i18n;
mod logging;
mod range_slider;
mod serial;
mod settings;

use app::App;
use iced::window;

fn main() -> iced::Result {
    logging::init();
    log::info!("starting dso3d12-gui");

    // Set language: use persisted if non-empty, otherwise detect from system.
    {
        let settings = crate::settings::Settings::load();
        let lang = if settings.language.is_empty() {
            let detected = crate::i18n::detect_system_language();
            log::info!("detected system language: {detected}");
            detected
        } else {
            settings.language.clone()
        };
        crate::i18n::set_language(&lang);
    }

    let icon = window::icon::from_file_data(
        include_bytes!("../img/dso3d12_ico.png"),
        None,
    )
    .ok();

    let settings = crate::settings::Settings::load();
    let (w, h) = settings.window_size;

    iced::application(App::new, App::update, App::view)
        .title(App::title)
        .theme(App::theme)
        .subscription(App::subscription)
        .window(window::Settings {
            icon,
            size: iced::Size::new(w, h),
            min_size: Some(iced::Size::new(800.0, 500.0)),
            ..Default::default()
        })
        .run()
}

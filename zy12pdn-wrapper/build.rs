use std::{env, path::PathBuf};

fn main() {
    println!("cargo:rerun-if-changed=wrapper.hpp");
    println!("cargo:rerun-if-changed=../zy12pdn-oss");

    cc::Build::new()
        .cpp(true)
        .flag("-std=gnu++20")
        .flag("-xc++")
        .flag("-nodefaultlibs")
        .flag("-fno-exceptions")
        .flag("-fno-common")
        .flag("-DSTM32F0")
        .flag("-DSTM32F030x6")
        .flag("-L/Users/ferdiamckeogh/.platformio/packages/framework-libopencm3/lib")
        .file("wrapper.hpp")
        .file("../zy12pdn-oss/src/pd_sink.cpp")
        .file("../zy12pdn-oss/src/hal.cpp")
        .file("../zy12pdn-oss/src/fusb302.cpp")
        .file("../zy12pdn-oss/src/i2c_bit_bang.cpp")
        .include("../zy12pdn-oss/include")
        .include("/Users/ferdiamckeogh/.platformio/packages/framework-libopencm3")
        .include("/Users/ferdiamckeogh/.platformio/packages/framework-libopencm3/include")
        .cpp_link_stdlib(None)
        .warnings(false)
        .compile("pd_sink");

    let bindings = bindgen::Builder::default()
        // clang args
        .clang_arg("-std=gnu++20")
        .clang_arg("-xc++")
        .clang_arg("-DSTM32F0")
        .clang_arg("-DSTM32F030x6")
        .clang_arg("-I/Users/ferdiamckeogh/.platformio/packages/framework-libopencm3")
        .clang_arg("-I/Users/ferdiamckeogh/.platformio/packages/framework-libopencm3/include")
        .clang_arg(
            "-I/Users/ferdiamckeogh/.platformio/packages/toolchain-gccarmnoneeabi/arm-none-eabi/include/c++/7.2.1",
        )
        .clang_arg(
            "-I/Users/ferdiamckeogh/.platformio/packages/toolchain-gccarmnoneeabi/arm-none-eabi/include/c++/7.2.1/arm-none-eabi/thumb/v6-m",
        )

        // header file
        .header("wrapper.hpp")
        .allowlist_file("wrapper.hpp")

        // options
        .use_core()
        .enable_cxx_namespaces()
        .parse_callbacks(Box::new(bindgen::CargoCallbacks))
        .layout_tests(false)
        .generate()
        .expect("Unable to generate bindings");

    bindings
        .write_to_file(PathBuf::from(env::var("OUT_DIR").unwrap()).join("bindings.rs"))
        .expect("Couldn't write bindings!");
}

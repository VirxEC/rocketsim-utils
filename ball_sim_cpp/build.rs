fn main() {
    cxx_build::bridge("src/lib.rs")
        .std("c++14")
        .compile("ball_sim_cpp");

    println!("cargo:rerun-if-changed=src/lib.rs");
}

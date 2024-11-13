set working-directory := 'src/rust_pub'

run:
    cargo run

lint:
    cargo fmt --check
    cargo clippy

fix:
    cargo fmt
    cargo clippy --fix

fix-force:
    cargo fmt
    cargo clippy --fix --allow-dirty --allow-staged
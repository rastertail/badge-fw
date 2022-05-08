{
  inputs = {
    nixpkgs.url = "github:nixos/nixpkgs/nixos-unstable";
    rust-overlay.url = "github:oxalica/rust-overlay";
    flake-utils.url = "github:numtide/flake-utils";
  };

  outputs = { nixpkgs, rust-overlay, flake-utils, ... }:
    flake-utils.lib.eachDefaultSystem (system: let
        pkgs = import nixpkgs { inherit system; overlays = [ rust-overlay.overlay ]; };
      in {
        devShell = pkgs.mkShell rec {
          nativeBuildInputs = let
            rust = pkgs.rust-bin.nightly.latest.default.override { 
              extensions = [ "rust-src" ];
              targets = [ "thumbv6m-none-eabi" ];
            }; 
          in [ rust pkgs.gcc-arm-embedded pkgs.openssl pkgs.pkg-config ];
        };
      }
    );
}

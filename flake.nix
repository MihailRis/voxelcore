{
  description = "VoxelCore – voxel game engine in C++";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixpkgs-unstable";
    flake-utils.url = "github:numtide/flake-utils";
  };

  outputs =
    {
      self,
      nixpkgs,
      flake-utils,
    }:
    flake-utils.lib.eachDefaultSystem (
      system:
      let
        pkgs = import nixpkgs { inherit system; };
        voxel-core = pkgs.stdenv.mkDerivation {
          name = "voxel-core";

          src = ./.;

          nativeBuildInputs = with pkgs; [
            cmake
            pkg-config
          ];

          buildInputs = with pkgs; [
            glm
            glfw
            glew
            zlib
            libpng
            libvorbis
            openal
            luajit
            curl
            entt
            mesa
            freeglut
          ]; # libglvnd

          packages = with pkgs; [
            glfw
            mesa
            freeglut
            entt
          ];
          cmakeFlags = [
            "-DCMAKE_PREFIX_PATH=${pkgs.entt}"
            "-DCMAKE_INCLUDE_PATH=${pkgs.entt}/include"
          ];

          installPhase = ''
            mkdir -p $out/bin
            cp VoxelEngine $out/bin/
          '';
        };
      in
      {
        packages.default = voxel-core;
        apps.default = {
          type = "app";
          program = "${voxel-core}/bin/VoxelCore";
        };
      }
    );
}

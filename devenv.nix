{
  pkgs,
  lib,
  inputs,
  ...
}:

let
  pkgs-stable = import inputs.stable { system = pkgs.stdenv.system; };
in
{
  packages =
    with pkgs;
    lib.mkBefore [
      # c++ stuff
      clang-tools
      cmake

      # dependencies
      libGL # for the OpenGL requirement
      glew
      pkgs-stable.glm
      nlohmann_json
      bullet
      libwebp
      libtiff
      SDL2
      SDL2_image

      pkg-config
    ];
}

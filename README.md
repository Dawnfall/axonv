# axonv

## Build
 - uses CMAKE with presets & VCPKG for PCL, VCPKG_ROOT must be set

```
cmake --preset [option]
cmake --build --preset [option]
./build/[option]/axonv [arguments]
```

## Preset option
 
 - debug
 - release

## Arguments

 - empty(uses default input.pcd)
 - path/to/cloud.pcd lineOffset
 - random lineOffset

## Use

 - Shift+click to select points (for some reason sometimes selecting doesnt work correctly)
 - "r" to reset
 - "c" to calculate


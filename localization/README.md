## Building files
Ensure vcpkg is installed already.
```bash
external/vcpkg/vcpkg install
```

To compile (first time or after adding dependencies in `vcpkg.json`)
```bash
cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_TOOLCHAIN_FILE=${PWD}/external/vcpkg/scripts/buildsystems/vcpkg.cmake \
  -DVCPKG_FEATURE_FLAGS=manifests \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

Build the package
```bash
cmake --build build -j
```

## Adding New Package
Add your package name in the dependencies array
```json
{
  "name": "autoronto-localization",
  "version-string": "0.1.0",
  "dependencies": [
    "opencv4",
    "eigen3"
  ]
}
```

Then, add the dependency inside the `CMAKELists.txt` for static type checkers.
```
find_package(Eigen3 CONFIG REQUIRED)

target_link_libraries(app PRIVATE Eigen3::Eigen)
```

After that, don't forget to rebuild the `vkpkg` & `cmake`.

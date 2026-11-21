# C++17 Coding Conventions

## Member Initialization

* **Primitives & Pointers:** Always use empty brace initialization `{}` (e.g., `double velocity_{};`, `bool armed_{};`,
  `int* ptr_{};`) to guarantee safe zero/null defaults.
* **Smart Pointers & STL Containers:** Omit `{}` on types with safe default constructors (`std::unique_ptr`,
  `std::vector`, `std::string`) to avoid redundant initialization warnings.
* **Large Hardware Buffers:** Omit `{}` on large fixed-size DMA/sensor buffers (`std::array<uint8_t, 4096> rx_buffer_;`)
  when immediately overwritten by hardware reads.

## Accessors & Const Correctness

* **New / Refactored Classes:** Return primitives by value and heavy objects by `const` reference. Mark non-throwing
  getters `const noexcept` and `[[nodiscard]]`:
    * `[[nodiscard]] double position() const noexcept { return position_rad_; }`
    * `[[nodiscard]] const Trajectory& trajectory() const noexcept { return trajectory_; }`
* **Legacy Code:** Older files use `const double&` for primitive getters. Leave working legacy signatures unchanged
  unless refactoring the class, and isolate pure style updates in dedicated commits.

## Safety & Resource Management

* **RAII:** Prefer `std::unique_ptr` over raw `new`/`delete`. Ensure hardware classes stop motors/actuators in their
  destructors.
* **Move Operations:** Mark move constructors and move assignment operators `noexcept` so `std::vector` reallocates
  without copying.

## Source Tree Organization

* **Libraries:** a library has its public headers in `include/<lib>/` and its sources in `src/<lib>/`
  (`pendule_pi`, `pendule_cli`). Header-only libraries (`yamlcfg`, `digital_filters`) live in `include/` and are
  declared as `INTERFACE` targets in `CMakeLists.txt`.
* **Executable-specific code:** a header used by a single executable stays next to its `.cpp` and is not published in
  `include/` (ex: `src/bin/demo/demo_config.hpp`). Code shared by several executables goes in a library.
* **Executables:** an executable made of several files has its own folder in `src/bin/` (`demo/`,
  `low_level_interface/`). Small single-file tools are grouped in `src/bin/tools/`. Helpers shared by executables only
  go in `src/bin/common/`.
* **Third-party code:** dependencies tracked by git go in `third_party/`. `deps/` is git-ignored and reserved to
  dependencies built locally.
* **Not C++ sources:** notebooks and measurement data go in `analysis/`, Python scripts in `python/`. Nothing but C++
  code belongs in `src/`.
* **Moving files:** after moving or deleting a source, delete the old path on the build target as well, otherwise the
  stale copy remains there.

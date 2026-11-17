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

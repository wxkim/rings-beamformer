#ifndef APP_MAIN_H_SHIM
#define APP_MAIN_H_SHIM

// This project is CubeMX-based. The canonical application-wide `main.h` lives in
// `Core/Inc/main.h`, but `App/Inc` is first on the include path. Provide a thin
// shim here so `#include "main.h"` always resolves to the CubeMX header.
#include "../../Core/Inc/main.h"

#endif // APP_MAIN_H_SHIM

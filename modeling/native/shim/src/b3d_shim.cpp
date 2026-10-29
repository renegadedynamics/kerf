#include "b3d_shim.h"

#include <OSD.hxx>
#include <Standard_Version.hxx>

#include <new>
#include <string>

struct b3d_error
{
  std::string type;
  std::string message;
};

b3d_error* b3d_make_error(const char* type, const char* message) noexcept
{
  try
  {
    return new b3d_error{type != nullptr ? type : "", message != nullptr ? message : ""};
  }
  catch (...)
  {
    // Out of memory while reporting an error: nothing better to hand back than a static one
    static b3d_error oom{"std::bad_alloc", "out of memory while reporting an OCCT error"};
    return &oom;
  }
}

B3D_API const char* b3d_error_type(const b3d_error* e)
{
  return e->type.c_str();
}

B3D_API const char* b3d_error_message(const b3d_error* e)
{
  return e->message.c_str();
}

B3D_API void b3d_error_free(b3d_error* e)
{
  if (e != nullptr && e->type != "std::bad_alloc")
    delete e;
}

B3D_API const char* b3d_occt_version()
{
  return OCC_VERSION_COMPLETE;
}

void b3d_install_thread_signals() noexcept
{
  try
  {
    // On Windows, a translator that turns a fault on this thread (an access violation, say) into a typed
    // Standard_Failure: OCCT's algorithms catch those and recover, and the shim reports the rest as errors.
    // Floating-point traps stay off, as .NET expects. On Linux and macOS OCCT's handlers are process-wide
    // signal handlers, which would take over the runtime's own, so nothing is installed there.
    OSD::SetThreadLocalSignal(OSD_SignalMode_Set, false);
  }
  catch (...)
  {
    // Without the translator, faults are still caught (the shim is built with /EHa), only untyped
  }
}

//! Reads an int at `address`: a deliberate fault, for testing that faults come back as errors.
B3D_API int32_t b3d_selftest_read(const volatile int32_t* address, b3d_error** err)
{
  *err = nullptr;
  B3D_TRY
  {
    return *address;
  }
  B3D_CATCH(err)
  return 0;
}

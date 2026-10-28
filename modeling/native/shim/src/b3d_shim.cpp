#include "b3d_shim.h"

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
